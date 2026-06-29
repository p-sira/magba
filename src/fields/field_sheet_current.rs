/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Analytical B-field computation for a triangular mesh carrying surface current.

use nalgebra::{Point3, UnitQuaternion, Vector3};

use crate::{
    base::{Float, coordinate::compute_in_local, mesh::TriMesh},
    crate_utils::{impl_parallel, impl_parallel_sum},
    fields::field_triangle_current::local_triangle_current_B,
};

/// Computes B-field of a current sheet mesh at point in local frame.
#[inline]
#[allow(non_snake_case)]
pub fn local_sheet_current_B<T: Float>(
    point: Point3<T>,
    current_densities: &[Vector3<T>],
    mesh: &TriMesh<T>,
) -> Vector3<T> {
    let mut b_total = Vector3::zeros();

    let count = core::cmp::min(current_densities.len(), mesh.triangles().len());
    for (i, &triangle) in mesh.triangles().iter().take(count).enumerate() {
        let j = current_densities[i];
        if j != Vector3::zeros() {
            b_total += local_triangle_current_B(point, j, &triangle.vertices());
        }
    }

    b_total
}

/// Computes B-field of a current sheet mesh at point (x, y, z).
#[inline]
#[allow(non_snake_case)]
pub fn sheet_current_B<T: Float>(
    point: Point3<T>,
    position: Point3<T>,
    orientation: UnitQuaternion<T>,
    current_densities: &[Vector3<T>],
    mesh: &TriMesh<T>,
) -> Vector3<T> {
    compute_in_local!(
        local_sheet_current_B,
        point,
        position,
        orientation,
        (current_densities, mesh),
    )
}

/// Computes B-field at points in global frame for a current sheet mesh.
#[allow(non_snake_case)]
pub fn sheet_current_B_batch<T: Float>(
    points: &[Point3<T>],
    position: Point3<T>,
    orientation: UnitQuaternion<T>,
    current_densities: &[Vector3<T>],
    mesh: &TriMesh<T>,
    out: &mut [Vector3<T>],
) {
    impl_parallel!(
        sheet_current_B,
        rayon_threshold: 100,
        input: points,
        output: out,
        args: [position, orientation, current_densities, mesh]
    )
}

/// Computes B-field at each given points in global frame for multiple current sheet meshes.
#[allow(non_snake_case)]
pub fn sum_multiple_sheet_current_B<T: Float>(
    points: &[Point3<T>],
    positions: &[Point3<T>],
    orientations: &[UnitQuaternion<T>],
    current_densities_list: &[alloc::vec::Vec<Vector3<T>>],
    meshes: &[&TriMesh<T>],
    out: &mut [Vector3<T>],
) {
    impl_parallel_sum!(
        out,
        points,
        10,
        [positions, orientations, current_densities_list, meshes],
        |pos, p, o, cds, mesh| sheet_current_B(*pos, *p, *o, cds, mesh)
    )
}
