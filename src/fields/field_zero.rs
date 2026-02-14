/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use nalgebra::{Point3, RealField, UnitQuaternion, Vector3};

pub fn zero_field<T: RealField>(
    points: &[Point3<T>],
    _: &Point3<T>,
    _: &UnitQuaternion<T>,
) -> Vec<Vector3<T>> {
    vec![Vector3::zeros(); points.len()]
}
