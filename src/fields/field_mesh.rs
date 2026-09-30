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
        mesh::{TriMesh, Triangle},
    },
    crate_utils::{impl_parallel, impl_parallel_sum},
    fields::field_triangle::{local_triangle_B, solid_angle},
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

    let mut total_solid_angle = T::zero();
    triangles.iter().for_each(|&triangle| {
        let vertices = triangle.vertices();
        b_total += local_triangle_B(point, polarization, vertices);

        let r_vecs = vertices.map(|vertex| vertex - point.coords);
        let r_mags = r_vecs.map(|r| r.norm());
        total_solid_angle += solid_angle(&r_vecs, &r_mags);
    });

    if num_traits::Float::abs(total_solid_angle) > T::pi() * T::from(2.0).unwrap() {
        b_total += polarization;
    }

    b_total
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use nalgebra::{point, vector};

    #[test]
    fn cube_center_is_classified_inside_when_ray_crosses_shared_edge() {
        let vertices = vec![
            vector![-1.0, -1.0, -1.0],
            vector![1.0, -1.0, -1.0],
            vector![1.0, 1.0, -1.0],
            vector![-1.0, 1.0, -1.0],
            vector![-1.0, -1.0, 1.0],
            vector![1.0, -1.0, 1.0],
            vector![1.0, 1.0, 1.0],
            vector![-1.0, 1.0, 1.0],
        ];
        let faces = vec![
            [0, 2, 1],
            [0, 3, 2],
            [4, 5, 6],
            [4, 6, 7],
            [0, 1, 5],
            [0, 5, 4],
            [3, 7, 6],
            [3, 6, 2],
            [0, 4, 7],
            [0, 7, 3],
            [1, 2, 6],
            [1, 6, 5],
        ];
        let mesh = TriMesh::new(vertices, faces).unwrap();

        let actual = mesh_B(
            point![0.0, 0.0, 0.0],
            Point3::origin(),
            UnitQuaternion::identity(),
            Vector3::z(),
            &mesh,
        );

        assert_relative_eq!(actual, vector![0.0, 0.0, 2.0 / 3.0], epsilon = 1e-12);
    }

    #[test]
    fn f32_millimeter_mesh_matches_tetrahedron() {
        let vertices = [
            vector![0.0_f32, 0.0, 0.0],
            vector![0.001, 0.0, 0.0],
            vector![0.0, 0.001, 0.0],
            vector![0.0, 0.0, 0.001],
        ];
        let mesh = TriMesh::new_unchecked(vertices, [[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2]]);
        let point = point![0.0001, 0.0002, 0.0003];

        let actual = mesh_B(
            point,
            Point3::origin(),
            UnitQuaternion::identity(),
            Vector3::z(),
            &mesh,
        );
        let expected = crate::fields::tetrahedron_B(
            point,
            Point3::origin(),
            UnitQuaternion::identity(),
            Vector3::z(),
            vertices,
        );

        assert_relative_eq!(actual, expected, epsilon = 1e-5);
    }
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
        rayon_threshold: 100,
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
