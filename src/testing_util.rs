/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Testing utilities.

use std::path::Path;

use nalgebra::{DMatrix, Point3, UnitQuaternion, Vector3};
use nalgebra_sparse::io::load_coo_from_matrix_market_file;

use crate::util::{is_close, relative_error, relative_vec_distance};

pub fn load_matrix(path_str: &str) -> DMatrix<f64> {
    let path = Path::new(path_str);
    let points: DMatrix<f64> =
        (&load_coo_from_matrix_market_file(path).expect("can parse matrix")).into();
    points
}

pub fn matrix_to_point_vec(matrix: &DMatrix<f64>) -> Vec<Point3<f64>> {
    matrix
        .row_iter()
        .map(|row| Point3::new(row[0], row[1], row[2]))
        .collect()
}

pub fn matrix_to_vector_vec(matrix: &DMatrix<f64>) -> Vec<Vector3<f64>> {
    matrix
        .row_iter()
        .map(|row| Vector3::new(row[0], row[1], row[2]))
        .collect()
}

pub fn quat_from_rotvec(x: f64, y: f64, z: f64) -> UnitQuaternion<f64> {
    UnitQuaternion::from_scaled_axis(Vector3::new(x, y, z))
}

pub fn assert_close_vector(vec1: &Vector3<f64>, vec2: &Vector3<f64>, rtol: f64) {
    let mut n_fail: usize = 0;
    vec1.iter().zip(vec2).enumerate().for_each(|(n, (&a, &b))| {
        if !is_close(a, b, rtol) {
            eprintln!(
                "Element {} mismatch. actual={}, expected={}, relative={:e}, rtol={:e}",
                n,
                a,
                b,
                relative_error(a, b),
                rtol
            );
            n_fail += 1
        }
    });
    if n_fail > 0 {
        let percent_fail = n_fail as f64 / 3.0 * 100.0;
        panic!("assert_close_vector fails. Difference={percent_fail}% ({n_fail}/3)")
    }
}

pub fn assert_close_vec_vector(vecs1: &Vec<Vector3<f64>>, vecs2: &Vec<Vector3<f64>>, rtol: f64) {
    let len = vecs1.len();
    if len != vecs2.len() {
        panic!("assert_close_vector fails. Two vector of Vector3 must be the same length.")
    }

    let mut n_fail: usize = 0;
    let mut worst_rdist = 0.0;
    let mut worst_params: (usize, Vector3<f64>, Vector3<f64>, f64) =
        (0, Vector3::default(), Vector3::default(), 0.0);
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
                    "Vector {} mismatch. actual={:?}, expected={:?}, rdist={:e}, rtol={:e}.",
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
    pub fn compare_B_with_file<T: Source>(
        source: &T,
        points_path_str: &str,
        ref_path_str: &str,
        rtol: f64,
    ) {
        let expected = matrix_to_vector_vec(&load_matrix(ref_path_str));
        let points = matrix_to_point_vec(&load_matrix(points_path_str));

        let b_fields = source.get_B(&points).expect("cannot calculate b field");

        assert_close_vec_vector(&b_fields, &expected, rtol);
    }
}
