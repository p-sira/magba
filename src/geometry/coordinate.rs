/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use nalgebra::{Point3, Vector3};

use super::Transform;

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
pub fn local_point(point: Point3<f64>, object: &impl Transform) -> Point3<f64> {
    object.orientation().inverse() * Point3::from(point.coords - object.position().coords)
}

/// Convert local vector to the global frame
pub fn global_vector(vector: Vector3<f64>, object: &impl Transform) -> Vector3<f64> {
    object.orientation() * vector + object.position().coords
}
