/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Coordinate conversion and calculation utilities for 3D geometry.

use nalgebra::{Point3, RealField, UnitQuaternion, Vector3};

/// Convenience macro for transforming function arguments to local frame
/// and convert back the result to global frame.
macro_rules! compute_in_local {
    ($func: ident, $point: expr, $position: expr, $orientation: expr, ($($func_args:expr),*),) => {
        {
            let local_point = crate::base::coordinate::global_to_local_point($point, $position, $orientation);
            let local_result_vector = $func(local_point, $($func_args),*);
            crate::base::coordinate::local_to_global_vector(local_result_vector, $orientation)
        }
    };
}
pub(crate) use compute_in_local;

/// Transform global point to the local frame of the object.
pub(crate) fn global_to_local_point<T: RealField + Copy>(
    point: Point3<T>,
    position: Point3<T>,
    orientation: UnitQuaternion<T>,
) -> Point3<T> {
    orientation.inverse() * Point3::from(point.coords - position.coords)
}

/// Transform local vector to the global frame.
pub(crate) fn local_to_global_vector<T: RealField + Copy>(
    vector: Vector3<T>,
    orientation: UnitQuaternion<T>,
) -> Vector3<T> {
    orientation * vector
}
