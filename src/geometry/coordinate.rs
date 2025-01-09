/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use nalgebra::{Point3, UnitQuaternion, Vector3};

/// Convert Cartesian coordinates (x, y) to cylindrical coordinates (r, phi)
pub fn cart2cyl(x: f64, y: f64) -> (f64, f64) {
    let r = (x * x + y * y).sqrt();
    let phi = y.atan2(x);
    (r, phi)
}

pub fn vec_cyl2cart(r: f64, phi: f64, theta: f64) -> (f64, f64) {
    let x = r * theta.cos() - phi * theta.sin();
    let y = r * theta.sin() + phi * theta.cos();
    (x, y)
}

/// Convert global point to the local frame of the object
pub fn local_point(
    point: &Point3<f64>,
    position: &Point3<f64>,
    orientation: &UnitQuaternion<f64>,
) -> Point3<f64> {
    orientation.inverse() * Point3::from(point.coords - position.coords)
}

pub fn local_points(
    points: &[Point3<f64>],
    position: &Point3<f64>,
    orientation: &UnitQuaternion<f64>,
) -> Vec<Point3<f64>> {
    points
        .iter()
        .map(|point| local_point(point, position, orientation))
        .collect()
}

// pub fn local_points(points: &MatrixXx3<f64>, object: &impl Transform) -> MatrixXx3<f64> {
//     let position = DMatrix::from_column_slice(3, 1, object.position().coords.as_slice());
//     let inv_orientation = object
//         .orientation()
//         .to_rotation_matrix()
//         .matrix()
//         .transpose();

//     let translated_points = points.transpose() - position;
//     let rotated_points = (inv_orientation * translated_points).transpose();

//     rotated_points
// }

/// Convert local vector to the global frame
pub fn global_vector(
    vector: &Vector3<f64>,
    position: &Point3<f64>,
    orientation: &UnitQuaternion<f64>,
) -> Vector3<f64> {
    orientation * vector + position.coords
}

pub fn global_vectors(
    local_vectors: &[Vector3<f64>],
    position: &Point3<f64>,
    orientation: &UnitQuaternion<f64>,
) -> Vec<Vector3<f64>> {
    local_vectors
        .iter()
        .map(|local_vector| global_vector(local_vector, position, orientation))
        .collect()
}
