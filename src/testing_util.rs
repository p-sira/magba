/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Testing utilities.

use csv::ReaderBuilder;
use nalgebra::{DMatrix, Point3, RealField, UnitQuaternion, Vector3, distance, point, vector};
use std::{
    fmt::{Debug, LowerExp},
    fs::File,
    io::BufReader,
    path::Path,
    str::FromStr,
};

use crate::base::Source;

/// Calculate the relative Euclidean distance
pub fn relative_vec_distance<T: RealField + Copy>(a: Vector3<T>, b: Vector3<T>) -> T {
    let dist = distance(&Point3::from(a), &Point3::from(b));
    (dist / a.magnitude()).max(dist / b.magnitude())
}

pub fn quat_from_rotvec<T: RealField>(x: T, y: T, z: T) -> UnitQuaternion<T> {
    UnitQuaternion::from_scaled_axis(vector![x, y, z])
}

pub fn load_matrix_from_csv<T: RealField + FromStr + Debug>(path: &Path) -> DMatrix<T> {
    let file =
        File::open(path).unwrap_or_else(|e| panic!("Cannot open file {}: {}", path.display(), e));
    let mut reader = ReaderBuilder::new()
        .has_headers(false)
        .from_reader(BufReader::new(file));

    let mut data = Vec::new();
    let mut ncols = 0;
    let mut nrows = 0;

    for result in reader.records() {
        let record = result.unwrap_or_else(|e| panic!("Cannot read file: {}", e));
        if ncols == 0 {
            ncols = record.len();
        } else if ncols != record.len() {
            panic!(
                "CSV row {} has inconsistent number of columns: expected {}, got {}",
                nrows + 1,
                ncols,
                record.len()
            );
        }

        for field in record.iter() {
            let value: T = field.parse().unwrap_or_else(|_| {
                panic!("Failed to parse '{}' as f64 on row {}", field, nrows + 1)
            });
            data.push(value);
        }

        nrows += 1;
    }

    DMatrix::from_row_slice(nrows, ncols, &data)
}

pub fn matrix_to_point_vec<T: RealField + Copy>(matrix: &DMatrix<T>) -> Vec<Point3<T>> {
    matrix
        .row_iter()
        .map(|row| point![row[0], row[1], row[2]])
        .collect()
}

pub fn matrix_to_vector_vec<T: RealField + Copy>(matrix: &DMatrix<T>) -> Vec<Vector3<T>> {
    matrix
        .row_iter()
        .map(|row| vector![row[0], row[1], row[2]])
        .collect()
}

pub fn assert_close_vec_vector<T: RealField + Copy + LowerExp>(
    vecs1: &Vec<Vector3<T>>,
    vecs2: &Vec<Vector3<T>>,
    rtol: T,
) {
    let len = vecs1.len();
    if len != vecs2.len() {
        panic!("assert_close_vector fails. Two vecs of Vector3 must be the same length.")
    }
    // Use parallel comparison for large vectors
    #[cfg(feature = "rayon")]
    if len > 1000 {
        use rayon::prelude::*;
        let failures: Vec<_> = vecs1
            .par_iter()
            .zip(vecs2)
            .enumerate()
            .filter_map(|(n, (vec1, vec2))| {
                let rdist = relative_vec_distance(*vec1, *vec2);
                if rdist > rtol {
                    Some((n, *vec1, *vec2, rdist))
                } else {
                    None
                }
            })
            .collect();
        if !failures.is_empty() {
            let (worst_n, worst_v1, worst_v2, worst_rdist) = failures
                .iter()
                .max_by(|a, b| a.3.partial_cmp(&b.3).unwrap())
                .unwrap();
            let percent_fail = failures.len() as f64 / len as f64 * 100.0;
            eprintln!(
                "Failed {}/{} vectors ({}%).",
                failures.len(),
                len,
                percent_fail
            );
            eprintln!(
                "Worst on vector {}: actual={:?}, expected={:?}, rdist={:e}, rtol={:e}.",
                worst_n, worst_v1, worst_v2, worst_rdist, rtol
            );
            panic!("assert_close_vec_vector");
        }
        return;
    }

    // Serial fallback for small vectors or no parallel feature
    let mut n_fail: usize = 0;
    let mut worst_rdist = T::zero();
    let mut worst_params: (usize, Vector3<T>, Vector3<T>, T) =
        (0, Vector3::zeros(), Vector3::zeros(), T::zero());
    vecs1
        .iter()
        .zip(vecs2)
        .enumerate()
        .for_each(|(n, (&vec1, &vec2))| {
            let rdist = relative_vec_distance(vec1, vec2);
            if rdist > rtol {
                if rdist > worst_rdist {
                    worst_rdist = rdist;
                    worst_params = (n, vec1, vec2, rdist);
                }
                eprintln!(
                    "Vector {} mismatched. actual={:?}, expected={:?}, rdist={:e}, rtol={:e}.",
                    n, vec1, vec2, rdist, rtol
                );
                n_fail += 1;
            }
        });
    if n_fail > 0 {
        let percent_fail = n_fail as f64 / len as f64 * 100.0;
        eprintln!("Failed {n_fail}/{len} vectors ({percent_fail}%).");
        eprintln!(
            "Worst on vector {}: actual={:?}, expected={:?}, rdist={:e}, rtol={:e}.",
            worst_params.0, worst_params.1, worst_params.2, worst_params.3, rtol
        );
        panic!("assert_close_vec_vector")
    }
}

pub fn mask_long_floats(s: &str) -> String {
    let re = regex::Regex::new(r"-?\d+\.(\d+)(?:e-?\d+)?").unwrap();

    re.replace_all(s, |caps: &regex::Captures| {
        let frac_len = caps[1].len();
        if frac_len > 6 {
            "<float>".to_string()
        } else {
            caps[0].to_string()
        }
    })
    .to_string()
}

#[allow(non_snake_case)]
pub fn compare_B_with_file<S: Source<T>, T: RealField + Copy + LowerExp + FromStr>(
    source: &S,
    points_path_str: &str,
    ref_path_str: &str,
    rtol: T,
) {
    let points_path = Path::new(points_path_str);
    let ref_path = Path::new(ref_path_str);
    if !points_path.is_file() {
        println!(
            "Test data {points_path:?} not found. Download from https://github.com/p-sira/magba/tree/main/tests/test-data."
        );
        return;
    }
    if !ref_path.is_file() {
        println!(
            "Test data {ref_path:?} not found. Download from https://github.com/p-sira/magba/tree/main/tests/test-data."
        );
        return;
    }

    let expected = matrix_to_vector_vec(&load_matrix_from_csv(ref_path));
    let points = matrix_to_point_vec(&load_matrix_from_csv(points_path));

    let b_fields = source.compute_B_batch(&points);

    assert_close_vec_vector(&b_fields, &expected, rtol);
}

#[allow(non_snake_case)]
macro_rules! test_B_magnet {
    (@small, $magnet: expr, $ref_path_str: expr, $rtol: expr) => {
        compare_B_with_file(
            $magnet,
            "./tests/test-data/points-small.csv",
            &format!("./tests/test-data/{}", $ref_path_str),
            $rtol,
        )
    };
    (@large, $magnet: expr, $ref_path_str: expr, $rtol: expr) => {
        compare_B_with_file(
            $magnet,
            "./tests/test-data/points-large.csv",
            &format!("./tests/test-data/{}", $ref_path_str),
            $rtol,
        )
    };
    ($magnet: expr, $ref_path_str: expr, $rtol: expr) => {
        compare_B_with_file(
            $magnet,
            "./tests/test-data/points.csv",
            &format!("./tests/test-data/{}", $ref_path_str),
            $rtol,
        )
    };
}
pub(crate) use test_B_magnet;

/// Generate basic tests for magnetic sources.
/// Tests compute_B, compute_B for small magnets, translate, and rotate.
///
/// ```text
/// generate_tests! {
///     Magnet
///     filename: magnet
///     params: { polarization: Vector3::z() }
///     rtols: {
///         static: 1e-10,
///         static_small: 1e-10,
///         translate: 1e-10,
///         rotate: 1e-10,
///     }
///     f32_rtols: {
///         static: 1e-6,
///         static_small: 1e-6,
///         translate: 1e-6,
///         rotate: 1e-6,
///     }
/// }
/// ```
///
/// ```text
/// $source_type: ident
/// filename: $filename: ident
/// params: { $($pname:ident : $params: expr),* $(,)? }
/// rtols: {
///     static: $rtol_static:expr,
///     static_small: $rtol_static_small:expr,
///     translate: $rtol_translate:expr,
///     rotate: $rtol_rotate:expr $(,)?
/// }
/// [f32_rtols: { ... }]
/// ```
/// - $source_type: Type identifier for the magnet.
/// - $filename: Base name of the test file.
/// - $pname: Name of the parameter. Unused in the test generation but serves as a reminder for the coder.
/// - $params: The base parameter value for the test. Must implement division operator as it will be
///   divided by 10 in small workspace tests.
/// - rtols: Relative tolerance for corresponding tests.
macro_rules! generate_tests {
    // Internal helper to generate f32 tests
    (@f32_mod $source_type: ident, $filename: ident, [$($params: expr),*], $f32_rtol_static: expr, $f32_rtol_static_small: expr, $f32_rtol_translate: expr, $f32_rtol_rotate: expr) => {
        mod f32_tests {
            use std::f32::consts::PI;
            use super::*;

            fn magnet() -> $source_type<f32> {
                $source_type::new(
                    point![0.1f32, 0.2, 0.3],
                    quat_from_rotvec(PI / 7.0, PI / 6.0, PI / 5.0),
                    $($params),*
                )
            }

            #[test]
            fn test_static() {
                test_B_magnet!(&magnet(), &format!("{}.csv", stringify!($filename)), $f32_rtol_static)
            }

            #[test]
            fn test_static_small() {
                let magnet = $source_type::new(
                    point![0.03f32, 0.02, 0.01],
                    quat_from_rotvec(PI / 7.0, PI / 6.0, PI / 5.0),
                    $(($params) / 10.0),*
                );
                test_B_magnet!(@small, &magnet, &format!("{}-small.csv", stringify!($filename)), $f32_rtol_static_small)
            }

            #[test]
            fn test_translate() {
                let mut magnet = magnet();
                magnet.translate(Translation3::new(-0.1f32, -0.2, -0.3));
                test_B_magnet!(&magnet, &format!("{}-translate.csv", stringify!($filename)), $f32_rtol_translate)

            }

            #[test]
            fn test_rotate() {
                let mut magnet = magnet();
                magnet.rotate(quat_from_rotvec(PI / 7.0, PI / 6.0, PI / 5.0).inverse());
                test_B_magnet!(&magnet, &format!("{}-rotate.csv", stringify!($filename)), $f32_rtol_rotate)
            }
        }
    };

    // Internal helper to generate f64 tests
    (@f64_mod $source_type: ident, $filename: ident, [$($params: expr),*], $rtol_static: expr, $rtol_static_small: expr, $rtol_translate: expr, $rtol_rotate: expr) => {
        mod f64_tests {
            use std::f64::consts::PI;
            use super::*;

            fn magnet() -> $source_type<f64> {
                $source_type::new(
                    point![0.1, 0.2, 0.3],
                    quat_from_rotvec(PI / 7.0, PI / 6.0, PI / 5.0),
                    $($params),*
                )
            }

            #[test]
            fn test_static() {
                test_B_magnet!(&magnet(), &format!("{}.csv", stringify!($filename)), $rtol_static)
            }

            #[test]
            fn test_static_small() {
                let magnet = $source_type::new(
                    point![0.03, 0.02, 0.01],
                    quat_from_rotvec(PI / 7.0, PI / 6.0, PI / 5.0),
                    $(($params) / 10.0),*
                );
                test_B_magnet!(@small, &magnet, &format!("{}-small.csv", stringify!($filename)), $rtol_static_small)
            }

            #[test]
            fn test_translate() {
                let mut magnet = magnet();
                magnet.translate(Translation3::new(-0.1, -0.2, -0.3));
                test_B_magnet!(&magnet, &format!("{}-translate.csv", stringify!($filename)), $rtol_translate)

            }

            #[test]
            fn test_rotate() {
                let mut magnet = magnet();
                magnet.rotate(quat_from_rotvec(PI / 7.0, PI / 6.0, PI / 5.0).inverse());
                test_B_magnet!(&magnet, &format!("{}-rotate.csv", stringify!($filename)), $rtol_rotate)
            }
        }
    };

    // Pattern with f32_rtols
    {
        $source_type: ident
        filename: $filename: ident
        params: { $($pname:ident : $params: expr),* $(,)? }
        rtols: {
            static: $rtol_static:expr,
            static_small: $rtol_static_small:expr,
            translate: $rtol_translate:expr,
            rotate: $rtol_rotate:expr $(,)?
        }
        f32_rtols: {
            static: $f32_rtol_static:expr,
            static_small: $f32_rtol_static_small:expr,
            translate: $f32_rtol_translate:expr,
            rotate: $f32_rtol_rotate:expr $(,)?
        }
    } => {
        mod generated_tests {
            use nalgebra::*;

            use crate::testing_util::*;
            use super::*;

            crate::testing_util::generate_tests!(@f64_mod $source_type, $filename, [$($params),*], $rtol_static, $rtol_static_small, $rtol_translate, $rtol_rotate);
            crate::testing_util::generate_tests!(@f32_mod $source_type, $filename, [$($params),*], $f32_rtol_static, $f32_rtol_static_small, $f32_rtol_translate, $f32_rtol_rotate);
        }
    };

    // Pattern without f32_rtols (legacy/default)
    {
        $source_type: ident
        filename: $filename: ident
        params: { $($pname:ident : $params: expr),* $(,)? }
        rtols: {
            static: $rtol_static:expr,
            static_small: $rtol_static_small:expr,
            translate: $rtol_translate:expr,
            rotate: $rtol_rotate:expr $(,)?
        }
    } => {
        mod generated_tests {
            use nalgebra::*;

            use crate::testing_util::*;
            use super::*;

            crate::testing_util::generate_tests!(@f64_mod $source_type, $filename, [$($params),*], $rtol_static, $rtol_static_small, $rtol_translate, $rtol_rotate);
        }
    };
}
pub(crate) use generate_tests;

/// Macro to verify `sum_multiple_*` field calculation functions.
///
/// It ensures that the sum of multiple sources matches the sum of individual field calculations.
macro_rules! impl_test_sum_multiple {
    (
        $sum_multiple_func:ident,
        $epsilon:expr,
        $points:expr,
        $positions:expr,
        $orientations:expr,
        ($($other_vecs:expr),*),
        |$p:ident, $pos:ident, $ori:ident, $($other_args:ident),*| $calc:expr
    ) => {{
        let mut out = vec![Vector3::zeros(); $points.len()];
        $sum_multiple_func(
            $points,
            $positions,
            $orientations,
            $($other_vecs,)*
            &mut out,
        );

        let mut expected = vec![Vector3::zeros(); $points.len()];
        for (i, p) in $points.iter().enumerate() {
            let $p = *p;
            let mut sum = Vector3::zeros();
            for (j, pos) in $positions.iter().enumerate() {
                let $pos = *pos;
                let $ori = $orientations[j];
                $(
                    let $other_args = $other_vecs[j];
                )*
                sum += $calc;
            }
            expected[i] = sum;
        }

        for (actual, expected) in out.iter().zip(expected.iter()) {
            approx::assert_relative_eq!(actual, expected, epsilon = $epsilon);
        }
    }};
}
pub(crate) use impl_test_sum_multiple;
