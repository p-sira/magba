/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use std::path::Path;

use nalgebra::{DMatrix, Point3, UnitQuaternion, Vector3};
use nalgebra_sparse::io::load_coo_from_matrix_market_file;

use crate::{sources::Source, util::is_close};

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
    vec1.iter()
        .zip(vec2)
        .for_each(|(a, b)| assert!(is_close(*a, *b, rtol)));
}

#[allow(non_snake_case)]
pub fn compare_B_with_file<T: Source>(
    source: &T,
    points_path_str: &str,
    ref_path_str: &str,
    rtol: f64,
) {
    let expected = matrix_to_vector_vec(&load_matrix(ref_path_str));
    let points = matrix_to_point_vec(&load_matrix(points_path_str));

    let b_fields = source.get_B(&points).expect("can calculate b field");

    b_fields
        .iter()
        .zip(&expected)
        .for_each(|(b_field, reference)| assert_close_vector(b_field, reference, rtol));
}
