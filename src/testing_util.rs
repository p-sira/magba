/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Testing utilities.

use std::{
    fmt::{Debug, LowerExp},
    fs::File,
    io::BufReader,
    path::Path,
    str::FromStr,
};

use csv::ReaderBuilder;
use nalgebra::{DMatrix, Point3, RealField, UnitQuaternion, Vector3};

use crate::crate_util::relative_vec_distance;

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
        .map(|row| Point3::new(row[0], row[1], row[2]))
        .collect()
}

pub fn matrix_to_vector_vec<T: RealField + Copy>(matrix: &DMatrix<T>) -> Vec<Vector3<T>> {
    matrix
        .row_iter()
        .map(|row| Vector3::new(row[0], row[1], row[2]))
        .collect()
}

pub fn quat_from_rotvec<T: RealField + Copy>(x: T, y: T, z: T) -> UnitQuaternion<T> {
    UnitQuaternion::from_scaled_axis(Vector3::new(x, y, z))
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
    #[cfg(feature = "parallel")]
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

#[cfg(feature = "sources")]
pub use source_testing_util::*;

#[cfg(feature = "sources")]
pub mod source_testing_util {
    use super::*;
    use crate::sources::*;

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
            println!("Test data {points_path:?} not found. Download from https://github.com/p-sira/magba/tree/main/tests/test-data.");
            return;
        }
        if !ref_path.is_file() {
            println!("Test data {ref_path:?} not found. Download from https://github.com/p-sira/magba/tree/main/tests/test-data.");
            return;
        }

        let expected = matrix_to_vector_vec(&load_matrix_from_csv(ref_path));
        let points = matrix_to_point_vec(&load_matrix_from_csv(points_path));

        let b_fields = source.get_B(&points);

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
    use nalgebra::RealField;
    pub(crate) use test_B_magnet;

    macro_rules! generate_tests {
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
            mod tests {
                use std::f64::consts::PI;

                use nalgebra::{Point3, Translation3};

                use crate::geometry::Transform;
                use crate::testing_util::*;
                use super::*;

                fn magnet() -> $source_type<f64> {
                    $source_type::new(
                        Point3::new(0.1, 0.2, 0.3),
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
                        Point3::new(0.03, 0.02, 0.01),
                        quat_from_rotvec(PI / 7.0, PI / 6.0, PI / 5.0),
                        $(($params) / 10.0),*
                    );
                    test_B_magnet!(@small, &magnet, &format!("{}-small.csv", stringify!($filename)), $rtol_static_small)
                }

                #[test]
                fn test_translate() {
                    let mut magnet = magnet();
                    magnet.translate(&Translation3::new(-0.1, -0.2, -0.3));
                    test_B_magnet!(&magnet, &format!("{}-translate.csv", stringify!($filename)), $rtol_translate)

                }

                #[test]
                fn test_rotate() {
                    let mut magnet = magnet();
                    magnet.rotate(&quat_from_rotvec(PI / 7.0, PI / 6.0, PI / 5.0).inverse());
                    test_B_magnet!(&magnet, &format!("{}-rotate.csv", stringify!($filename)), $rtol_rotate)
                }
            }
        };
    }
    pub(crate) use generate_tests;
}
