/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use nalgebra::{Point3, RealField, UnitQuaternion, Vector3};

/// Return zero vectors for all points.
///
/// Stores the results in the mutable slice `out`.
pub fn zero_field<T: RealField>(
    points: &[Point3<T>],
    _: &Point3<T>,
    _: &UnitQuaternion<T>,
    out: &mut [Vector3<T>],
) {
    assert_eq!(
        out.len(),
        points.len(),
        "Output slice length must match points length."
    );
    out.fill(Vector3::zeros());
}
