/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Coordinate conversion and calculation.

use nalgebra::{Point3, UnitQuaternion, Vector3};

/// Convert Cartesian coordinates *(x, y)* to cylindrical coordinates *(r, phi)*.
pub fn cart2cyl(x: f64, y: f64) -> (f64, f64) {
    let r = (x * x + y * y).sqrt();
    let phi = y.atan2(x);
    (r, phi)
}

/// Convert vector with component *(r, phi)* in cylindrical to Cartesian CS.
///
/// *theta*: angle of the vector on XY plane
pub fn vec_cyl2cart(r: f64, phi: f64, theta: f64) -> (f64, f64) {
    let x = r * theta.cos() - phi * theta.sin();
    let y = r * theta.sin() + phi * theta.cos();
    (x, y)
}

/// Convenience macro for transforming function arguments to local frame
/// and convert back the result to global frame.
#[macro_export]
macro_rules! compute_in_local {
    ($func: ident, $points: expr, ($($func_args:expr),*), $position: expr, $orientation: expr) => {
        global_vectors(&$func(&local_points($points, $position, $orientation), $($func_args),*)?, $orientation)
    };
}

/// Transform global point to the local frame of the object.
pub fn local_point(
    point: &Point3<f64>,
    position: &Point3<f64>,
    orientation: &UnitQuaternion<f64>,
) -> Point3<f64> {
    orientation.inverse() * Point3::from(point.coords - position.coords)
}

/// Transfrom multiple points in global frame to the local frame of the object.
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

/// Transform local vector to the global frame.
pub fn global_vector(vector: &Vector3<f64>, orientation: &UnitQuaternion<f64>) -> Vector3<f64> {
    orientation * vector
}

/// Transform local vectors to the global frame.
pub fn global_vectors(
    local_vectors: &[Vector3<f64>],
    orientation: &UnitQuaternion<f64>,
) -> Vec<Vector3<f64>> {
    local_vectors
        .iter()
        .map(|local_vector| global_vector(local_vector, orientation))
        .collect()
}
