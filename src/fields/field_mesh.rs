/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Analytical B-field computation for homogeneously magnetized triangular mesh.

use nalgebra::{Point3, UnitQuaternion, Vector3};

#[cfg(feature = "rayon")]
use crate::crate_utils::adaptive_rayon_threshold;
use crate::{
    base::{
        Float,
        coordinate::compute_in_local,
        mesh::{TriMesh, Triangle},
    },
    crate_utils::impl_parallel,
    fields::field_triangle::{PrecomputedMagnetizedTriangle, local_triangle_B_with_solid_angle},
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

/// Evaluate a mesh whose source-invariant face geometry was prepared once.
#[inline]
#[allow(non_snake_case)]
fn local_mesh_B_precomputed<T: Float>(
    point: Point3<T>,
    polarization: Vector3<T>,
    faces: &[Option<PrecomputedMagnetizedTriangle<T>>],
) -> Vector3<T> {
    let mut b_total = Vector3::zeros();
    let mut total_solid_angle = T::zero();

    for face in faces.iter().flatten() {
        let (b_face, omega) = face.compute_B_with_solid_angle(point);
        b_total += b_face;
        total_solid_angle += omega;
    }

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
    let faces: alloc::vec::Vec<_> = mesh
        .triangles()
        .iter()
        .map(|triangle| PrecomputedMagnetizedTriangle::new(polarization, triangle.vertices()))
        .collect();
    // Tuned with a 698-triangle mesh at a 10-observer crossover.
    #[cfg(feature = "rayon")]
    const PARALLEL_WORK_THRESHOLD: usize = 6_980;
    impl_parallel!(
        rayon_threshold: adaptive_rayon_threshold(PARALLEL_WORK_THRESHOLD, faces.len()),
        input: points,
        output: out,
        |p| {
            let local_point = inv_orientation * Point3::from(p.coords - position.coords);
            let local_b = local_mesh_B_precomputed(local_point, polarization, &faces);
            orientation * local_b
        }
    );
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
    fn precomputed_mesh_batch_matches_scalar_with_degenerate_face_and_rotation() {
        let vertices = [
            vector![0.0_f64, 0.0, 0.0],
            vector![1.0, 0.0, 0.0],
            vector![0.0, 1.0, 0.0],
            vector![0.0, 0.0, 1.0],
        ];
        let mesh = TriMesh::new_unchecked(
            vertices,
            [[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2], [0, 1, 1]],
        );
        let polarization = vector![0.7, -0.2, 0.4];
        let position = point![0.2, -0.5, 0.7];
        let orientation = UnitQuaternion::from_scaled_axis(vector![-0.3, 0.2, 0.1]);
        let local_points = [
            point![0.1, 0.1, 0.1],
            point![2.0, -1.0, 3.0],
            point![0.25, 0.25, 0.0],
        ];
        let points = local_points.map(|p| position + orientation * p.coords);
        let mut out = [Vector3::zeros(); 3];
        mesh_B_batch(
            &points,
            position,
            orientation,
            polarization,
            &mesh,
            &mut out,
        );

        for (&point, actual) in points.iter().zip(out.iter()) {
            assert_relative_eq!(
                *actual,
                mesh_B(point, position, orientation, polarization, &mesh),
                epsilon = 1e-13,
                max_relative = 1e-12
            );
        }
    }

    #[test]
    fn test_mesh_batch() {
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

        // Batch with <= 100 points for serial threshold (verified against magpylib)
        let small_points = [
            point![0.05, 0.05, 0.05],
            point![0.1, 0.2, 0.3],
            point![0.01, -0.02, 0.03],
        ];
        let mut small_out = vec![Vector3::zeros(); 3];
        mesh_B_batch(
            &small_points,
            Point3::origin(),
            UnitQuaternion::identity(),
            vector![0.0, 0.0, 1.0],
            &mesh,
            &mut small_out,
        );
        let expected = [
            vector![
                2.0728340382113465e-08,
                2.0728340382113465e-08,
                1.393369198233324e-19
            ],
            vector![
                1.6309465566011645e-10,
                3.265976692984299e-10,
                2.360953478678851e-10
            ],
            vector![
                1.6018157241643468e-07,
                -3.3266181333030693e-07,
                2.3286089750517369e-07
            ],
        ];
        for (o, e) in small_out.iter().zip(expected.iter()) {
            approx::assert_relative_eq!(o, e, epsilon = 1e-14, max_relative = 1e-10);
        }
    }
}
