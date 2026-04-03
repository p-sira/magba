/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Analytical B-field computation for homogeneously magnetized triangular mesh.

use nalgebra::{Point3, UnitQuaternion, Vector3};
use rayx::{Ray, intersect::intersect_moller_trumbore};

use crate::{
    base::{Float, coordinate::compute_in_local},
    crate_utils::{impl_parallel, impl_parallel_sum},
    fields::field_triangle::local_triangle_B,
};

/// Computes B-field of a homogeneously magnetized mesh at point in local frame.
///
/// # Arguments
///
/// - `point`: Observer position (m)
/// - `polarization`: Polarization vector (T)
/// - `vertices`: Mesh vertices in local coords (m)
/// - `faces`: Triangles forming the mesh
///
/// # Returns
///
/// - B-field vector (T) at point (x, y, z)
#[inline]
#[allow(non_snake_case)]
pub fn local_mesh_B<T: Float>(
    point: Point3<T>,
    polarization: Vector3<T>,
    vertices: &[Vector3<T>],
    faces: &[[usize; 3]],
) -> Vector3<T> {
    let mut b_total = Vector3::zeros();

    let ray = Ray::new(point.coords, Vector3::new(T::one(), T::zero(), T::zero()));
    let mut intersections = 0;

    for face in faces {
        let v1 = vertices[face[0]];
        let v2 = vertices[face[1]];
        let v3 = vertices[face[2]];

        b_total += local_triangle_B(point, polarization, [v1, v2, v3]);

        if let Some(_) = intersect_moller_trumbore(v1, v2, v3, ray, T::zero(), T::infinity()) {
            intersections += 1;
        }
    }

    if intersections % 2 != 0 {
        b_total += polarization;
    }

    b_total
}

/// Computes B-field of a homogeneously magnetized mesh at point (x, y, z).
///
/// # Arguments
///
/// - `point`: Observer position (m)
/// - `position`: Element center/position (m)
/// - `orientation`: Element orientation in unit quaternion
/// - `polarization`: Polarization vector (T)
/// - `vertices`: Mesh vertices in local coords (m)
/// - `faces`: Triangles forming the mesh
///
/// # Returns
///
/// - B-field vector (T) at point (x, y, z)
#[inline]
#[allow(non_snake_case)]
pub fn mesh_B<T: Float>(
    point: Point3<T>,
    position: Point3<T>,
    orientation: UnitQuaternion<T>,
    polarization: Vector3<T>,
    vertices: &[Vector3<T>],
    faces: &[[usize; 3]],
) -> Vector3<T> {
    compute_in_local!(
        local_mesh_B,
        point,
        position,
        orientation,
        (polarization, vertices, faces),
    )
}

/// Computes B-field at points in global frame for a mesh.
///
/// # Arguments
///
/// - `points`: Observer positions (m)
/// - `position`: Element position (m)
/// - `orientation`: Element orientation in unit quaternion
/// - `polarization`: Polarization vector (T)
/// - `vertices`: Mesh vertices in local coords (m)
/// - `faces`: Triangles forming the mesh
/// - `out`: Mutable slice to store the B-field vectors at each observer (T)
#[allow(non_snake_case)]
pub fn mesh_B_batch<T: Float>(
    points: &[Point3<T>],
    position: Point3<T>,
    orientation: UnitQuaternion<T>,
    polarization: Vector3<T>,
    vertices: &[Vector3<T>],
    faces: &[[usize; 3]],
    out: &mut [Vector3<T>],
) {
    impl_parallel!(
        mesh_B,
        rayon_threshold: 400,
        input: points,
        output: out,
        args: [position, orientation, polarization, vertices, faces]
    )
}

/// Computes B-field at each given points in global frame for multiple meshes.
///
/// # Arguments
///
/// - `points`: Observer positions (m)
/// - `positions`: Element positions (m)
/// - `orientations`: Element orientations in unit quaternion
/// - `polarizations`: Polarization vectors (T)
/// - `vertices_list`: List of mesh vertices arrays in local coords (m)
/// - `faces_list`: List of mesh faces arrays
/// - `out`: Mutable slice to store the net B-field vectors at each observer (T)
#[allow(non_snake_case)]
pub fn sum_multiple_mesh_B<T: Float>(
    points: &[Point3<T>],
    positions: &[Point3<T>],
    orientations: &[UnitQuaternion<T>],
    polarizations: &[Vector3<T>],
    vertices_list: &[&[Vector3<T>]],
    faces_list: &[&[[usize; 3]]],
    out: &mut [Vector3<T>],
) {
    impl_parallel_sum!(
        out,
        points,
        10,
        [
            positions,
            orientations,
            polarizations,
            vertices_list,
            faces_list
        ],
        |pos, p, o, pol, vert, face| mesh_B(*pos, *p, *o, *pol, *vert, *face)
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{point, vector};

    #[test]
    fn test_sum_multiple_mesh_b() {
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
        let vertices1 = [
            vector![0.0, 0.0, 0.0],
            vector![0.0, 0.0, 1.0],
            vector![1.0, 0.0, 0.0],
            vector![0.0, 1.0, 0.0],
        ];
        let faces1 = [[0, 1, 2], [0, 1, 3], [0, 2, 3], [1, 2, 3]];
        let vertices2 = [
            vector![-1.0, -1.0, -1.0],
            vector![1.0, -1.0, -1.0],
            vector![0.0, 1.0, -1.0],
            vector![0.0, 0.0, 1.0],
        ];
        let faces2 = [[0, 1, 2], [0, 1, 3], [0, 2, 3], [1, 2, 3]];

        let vertices_list: &[&[Vector3<f64>]] = &[&vertices1, &vertices2];
        let faces_list: &[&[[usize; 3]]] = &[&faces1, &faces2];

        impl_test_sum_multiple!(
            sum_multiple_mesh_B,
            1e-15,
            points,
            positions,
            orientations,
            (polarizations, vertices_list, faces_list),
            |p, pos, ori, pol, vert, face| mesh_B(p, pos, ori, pol, vert, face)
        );
    }
}
