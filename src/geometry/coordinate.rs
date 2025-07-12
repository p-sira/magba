/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Coordinate conversion and calculation utilities for 3D geometry.

use nalgebra::{Point3, RealField, UnitQuaternion, Vector3};

/// Convert Cartesian coordinates *(x, y)* to cylindrical coordinates *(r, phi)*.
pub fn cart2cyl<T: RealField + Copy>(x: T, y: T) -> (T, T) {
    let r = (x * x + y * y).sqrt();
    let phi = y.atan2(x);
    (r, phi)
}

/// Convert vector with component *(r, phi)* in cylindrical to Cartesian CS.
/// - `theta`: Angle of the vector on XY plane
pub fn vec_cyl2cart<T: RealField + Copy>(r: T, phi: T, theta: T) -> (T, T) {
    let x = r * theta.cos() - phi * theta.sin();
    let y = r * theta.sin() + phi * theta.cos();
    (x, y)
}

/// Convenience macro for transforming function arguments to local frame
/// and convert back the result to global frame.
#[macro_export]
macro_rules! compute_in_local {
    ($func: ident, $points: expr, ($($func_args:expr),*), $position: expr, $orientation: expr) => {
        global_vectors(&$func(&local_points($points, $position, $orientation), $($func_args),*), $orientation)
    };
}

/// Transform global point to the local frame of the object.
pub fn local_point<T: RealField + Copy>(
    point: &Point3<T>,
    position: &Point3<T>,
    orientation: &UnitQuaternion<T>,
) -> Point3<T> {
    orientation.inverse() * Point3::from(point.coords - position.coords)
}

/// Transform multiple points in global frame to the local frame of the object.
pub fn local_points<T: RealField + Copy>(
    points: &[Point3<T>],
    position: &Point3<T>,
    orientation: &UnitQuaternion<T>,
) -> Vec<Point3<T>> {
    points
        .iter()
        .map(|point| local_point(point, position, orientation))
        .collect()
}

/// Transform local vector to the global frame.
pub fn global_vector<T: RealField + Copy>(
    vector: &Vector3<T>,
    orientation: &UnitQuaternion<T>,
) -> Vector3<T> {
    orientation * vector
}

/// Transform local vectors to the global frame.
pub fn global_vectors<T: RealField + Copy>(
    local_vectors: &[Vector3<T>],
    orientation: &UnitQuaternion<T>,
) -> Vec<Vector3<T>> {
    local_vectors
        .iter()
        .map(|local_vector| global_vector(local_vector, orientation))
        .collect()
}
