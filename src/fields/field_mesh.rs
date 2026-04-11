/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Analytical B-field computation for homogeneously magnetized triangular mesh.

use nalgebra::{Point3, UnitQuaternion, Vector3};

use crate::{
    base::{
        Float,
        coordinate::compute_in_local,
        mesh::{TriMesh, Triangle, is_ray_hit},
    },
    crate_utils::{impl_parallel, impl_parallel_sum},
    fields::field_triangle::local_triangle_B,
};

/// Computes B-field of a homogeneously magnetized mesh at point in local frame.
///
/// # Arguments
///
/// - `point`: Observer position (m)
/// - `polarization`: Polarization vector (T)
/// - `triangles`: Triangles forming the mesh in local coords (m)
///
/// # Returns
///
/// - B-field vector (T) at point (x, y, z)
#[inline]
#[allow(non_snake_case)]
pub fn local_mesh_B<T: Float>(
    point: Point3<T>,
    polarization: Vector3<T>,
    triangles: &[Triangle<T>],
) -> Vector3<T> {
    let mut b_total = Vector3::zeros();

    let ray_origin = point.coords;
    let ray_dir = Vector3::new(T::one(), T::zero(), T::zero());

    let mut intersections = 0;
    triangles.iter().for_each(|&triangle| {
        b_total += local_triangle_B(point, polarization, triangle.vertices());

        if is_ray_hit(triangle, ray_origin, ray_dir, T::zero(), T::infinity()) {
            intersections += 1;
        }
    });

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
/// - `triangles`: Triangles forming the mesh
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
    mesh: &TriMesh<T>,
) -> Vector3<T> {
    compute_in_local!(
        local_mesh_B,
        point,
        position,
        orientation,
        (polarization, mesh.triangles()),
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
/// - `triangles`: Triangles forming the mesh
/// - `out`: Mutable slice to store the B-field vectors at each observer (T)
#[allow(non_snake_case)]
pub fn mesh_B_batch<T: Float>(
    points: &[Point3<T>],
    position: Point3<T>,
    orientation: UnitQuaternion<T>,
    polarization: Vector3<T>,
    mesh: &TriMesh<T>,
    out: &mut [Vector3<T>],
) {
    impl_parallel!(
        mesh_B,
        rayon_threshold: 400,
        input: points,
        output: out,
        args: [position, orientation, polarization, mesh]
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
/// - `triangles_list`: List of mesh triangles arrays in local coords (m)
/// - `out`: Mutable slice to store the net B-field vectors at each observer (T)
#[allow(non_snake_case)]
pub fn sum_multiple_mesh_B<T: Float>(
    points: &[Point3<T>],
    positions: &[Point3<T>],
    orientations: &[UnitQuaternion<T>],
    polarizations: &[Vector3<T>],
    meshes: &[&TriMesh<T>],
    out: &mut [Vector3<T>],
) {
    impl_parallel_sum!(
        out,
        points,
        10,
        [positions, orientations, polarizations, meshes],
        |pos, p, o, pol, mesh| mesh_B(*pos, *p, *o, *pol, mesh)
    )
}
