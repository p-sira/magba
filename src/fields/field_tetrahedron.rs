/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Analytical B-field computation for homogeneously magnetized tetrahedron.

use nalgebra::{Matrix3, Point3, UnitQuaternion, Vector3};

use crate::{
    base::{Float, coordinate::compute_in_local},
    crate_utils::{impl_parallel, impl_parallel_sum},
    fields::field_triangle::local_triangle_B,
};

/// Computes B-field of a homogeneously magnetized tetrahedron at point in local frame.
///
/// # Arguments
///
/// - `point`: Observer position (m)
/// - `polarization`: Polarization vector (T)
/// - `vertices`: Tetrahedron vertices `[P1, P2, P3, P4]` in local coords (m)
///
/// # Returns
///
/// - B-field vector (T) at point (x, y, z)
#[inline]
#[allow(non_snake_case)]
pub fn local_tetrahedron_B<T: Float>(
    point: Point3<T>,
    polarization: Vector3<T>,
    mut vertices: [Vector3<T>; 4],
) -> Vector3<T> {
    // Check chirality and ensure a right-handed system
    let v01 = vertices[1] - vertices[0];
    let v02 = vertices[2] - vertices[0];
    let v03 = vertices[3] - vertices[0];
    let det = Matrix3::from_columns(&[v01, v02, v03]).determinant();
    if det < T::zero() {
        vertices.swap(2, 3);
    }

    // Sum over 4 triangular faces
    let b1 = local_triangle_B(point, polarization, [vertices[0], vertices[2], vertices[1]]);
    let b2 = local_triangle_B(point, polarization, [vertices[0], vertices[1], vertices[3]]);
    let b3 = local_triangle_B(point, polarization, [vertices[1], vertices[2], vertices[3]]);
    let b4 = local_triangle_B(point, polarization, [vertices[0], vertices[3], vertices[2]]);

    let mut b_total = b1 + b2 + b3 + b4;

    // Check if observer is inside the tetrahedron using barycentric coordinates
    if let Some(mat_inv) = Matrix3::from_columns(&[
        vertices[1] - vertices[0],
        vertices[2] - vertices[0],
        vertices[3] - vertices[0],
    ]).try_inverse() {
        let p_rel = point.coords - vertices[0];
        let new_p = mat_inv * p_rel;

        if new_p.x >= T::zero()
            && new_p.y >= T::zero()
            && new_p.z >= T::zero()
            && new_p.x <= T::one()
            && new_p.y <= T::one()
            && new_p.z <= T::one()
            && new_p.sum() <= T::one()
        {
            b_total += polarization;
        }
    }

    b_total
}

/// Computes B-field of a homogeneously magnetized tetrahedron at point (x, y, z).
///
/// # Arguments
///
/// - `point`: Observer position (m)
/// - `position`: Element center/position (m)
/// - `orientation`: Element orientation in unit quaternion
/// - `polarization`: Polarization vector (T)
/// - `vertices`: Tetrahedron vertices in local coords (m)
///
/// # Returns
///
/// - B-field vector (T) at point (x, y, z)
#[inline]
#[allow(non_snake_case)]
pub fn tetrahedron_B<T: Float>(
    point: Point3<T>,
    position: Point3<T>,
    orientation: UnitQuaternion<T>,
    polarization: Vector3<T>,
    vertices: [Vector3<T>; 4],
) -> Vector3<T> {
    compute_in_local!(
        local_tetrahedron_B,
        point,
        position,
        orientation,
        (polarization, vertices),
    )
}

/// Computes B-field at points in global frame for a tetrahedron.
///
/// # Arguments
///
/// - `points`: Observer positions (m)
/// - `position`: Element position (m)
/// - `orientation`: Element orientation in unit quaternion
/// - `polarization`: Polarization vector (T)
/// - `vertices`: Tetrahedron vertices in local coords (m)
/// - `out`: Mutable slice to store the B-field vectors at each observer (T)
#[allow(non_snake_case)]
pub fn tetrahedron_B_batch<T: Float>(
    points: &[Point3<T>],
    position: Point3<T>,
    orientation: UnitQuaternion<T>,
    polarization: Vector3<T>,
    vertices: [Vector3<T>; 4],
    out: &mut [Vector3<T>],
) {
    impl_parallel!(
        tetrahedron_B,
        rayon_threshold: 1550, // Derived from triangle having 3100 and we do 4 of them
        input: points,
        output: out,
        args: [position, orientation, polarization, vertices]
    )
}

/// Computes B-field at each given points in global frame for multiple tetrahedrons.
///
/// # Arguments
///
/// - `points`: Observer positions (m)
/// - `positions`: Element positions (m)
/// - `orientations`: Element orientations in unit quaternion
/// - `polarizations`: Polarization vectors (T)
/// - `vertices_list`: List of tetrahedron vertices arrays in local coords (m)
/// - `out`: Mutable slice to store the net B-field vectors at each observer (T)
#[allow(non_snake_case)]
pub fn sum_multiple_tetrahedron_B<T: Float>(
    points: &[Point3<T>],
    positions: &[Point3<T>],
    orientations: &[UnitQuaternion<T>],
    polarizations: &[Vector3<T>],
    vertices_list: &[[Vector3<T>; 4]],
    out: &mut [Vector3<T>],
) {
    impl_parallel_sum!(
        out,
        points,
        15, // Derived from triangle having 60
        [positions, orientations, polarizations, vertices_list],
        |pos, p, o, pol, vert| tetrahedron_B(*pos, *p, *o, *pol, *vert)
    )
}

#[cfg(test)]
mod tests {
    use nalgebra::{point, vector};

    use super::*;

    #[test]
    fn test_sum_multiple_tetrahedron_b() {
        use crate::testing_util::impl_test_sum_multiple;
        let points = &[
            point![5.0, 6.0, 7.0],
            point![4.0, 3.0, 2.0],
            point![0.5, 0.25, 0.125],
        ];
        let positions = &[point![1.0, 2.0, 3.0], point![0.0, 0.0, 0.0]];
        let orientations = &[
            UnitQuaternion::from_scaled_axis(vector![1.0, 0.6, 0.4]),
            UnitQuaternion::identity(),
        ];
        let polarizations = &[vector![0.45, 0.3, 0.15], vector![1.0, 2.0, 3.0]];
        let vertices_list = &[
            [
                vector![0.0, 0.0, 0.0],
                vector![0.0, 0.0, 1.0],
                vector![1.0, 0.0, 0.0],
                vector![0.0, 1.0, 0.0],
            ],
            [
                vector![-1.0, -1.0, -1.0],
                vector![1.0, -1.0, -1.0],
                vector![0.0, 1.0, -1.0],
                vector![0.0, 0.0, 1.0],
            ],
        ];

        impl_test_sum_multiple!(
            sum_multiple_tetrahedron_B,
            1e-15,
            points,
            positions,
            orientations,
            (polarizations, vertices_list),
            |p, pos, ori, pol, vert| tetrahedron_B(p, pos, ori, pol, vert)
        );
    }
}
