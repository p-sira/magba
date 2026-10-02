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
    fields::field_triangle::local_triangle_B_with_solid_angle,
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
        let (b_face, omega) = local_triangle_B_with_solid_angle(point, polarization, vertices);
        b_total += b_face;
        total_solid_angle += omega;
    });

    if num_traits::Float::abs(total_solid_angle) > T::pi() * T::from(2.0).unwrap() {
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
    let inv_orientation = orientation.inverse();
    let triangles = mesh.triangles();
    impl_parallel!(
        rayon_threshold: 100,
        input: points,
        output: out,
        |p| {
            let local_point = inv_orientation * Point3::from(p.coords - position.coords);
            let local_b = local_mesh_B(local_point, polarization, triangles);
            orientation * local_b
        }
    );
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

    #[test]
    fn test_mesh_batch_and_sum_multiple() {
        let vertices = vec![
            vector![0.0, 0.0, 0.0],
            vector![0.001, 0.0, 0.0],
            vector![0.0, 0.001, 0.0],
            vector![0.0, 0.0, 0.001],
        ];
        let faces = vec![[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2]];
        let mesh = TriMesh::new(vertices, faces).unwrap();

        // Batch > 100 points for Rayon threshold
        let points = vec![point![0.0, 0.0, 1.0]; 120];
        let mut out = vec![Vector3::zeros(); 120];
        mesh_B_batch(
            &points,
            Point3::origin(),
            UnitQuaternion::identity(),
            vector![0.0, 0.0, 1.0],
            &mesh,
            &mut out,
        );
        assert_eq!(out.len(), 120);

        // Batch with <= 100 points for serial threshold
        let mut small_out = vec![Vector3::zeros(); 5];
        mesh_B_batch(
            &points[..5],
            Point3::origin(),
            UnitQuaternion::identity(),
            vector![0.0, 0.0, 1.0],
            &mesh,
            &mut small_out,
        );
        assert_eq!(small_out.len(), 5);
        assert!(small_out[0].z != 0.0);

        // sum_multiple_mesh_B with < 10 points and > 10 points
        let positions = [point![0.0, 0.0, 0.0], point![0.01, 0.0, 0.0]];
        let orientations = [UnitQuaternion::identity(), UnitQuaternion::identity()];
        let polarizations = [vector![0.0, 0.0, 1.0], vector![0.0, 0.0, 1.0]];
        let meshes = [&mesh, &mesh];

        let mut out_small = vec![Vector3::zeros(); 2];
        sum_multiple_mesh_B(
            &points[..2],
            &positions,
            &orientations,
            &polarizations,
            &meshes,
            &mut out_small,
        );
        assert_eq!(out_small.len(), 2);

        let mut out_large = vec![Vector3::zeros(); 15];
        sum_multiple_mesh_B(
            &points[..15],
            &positions,
            &orientations,
            &polarizations,
            &meshes,
            &mut out_large,
        );
        assert_eq!(out_large.len(), 15);
    }
}
