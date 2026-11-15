/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Analytical B-field computation for a homogeneously charged triangular current sheet.

use nalgebra::{Point3, UnitQuaternion, Vector3};

use crate::{
    base::{Float, coordinate::compute_in_local},
    crate_utils::impl_parallel,
};

use super::triangle_surface::TriangleSurface;

#[derive(Clone, Copy)]
pub(crate) struct PrecomputedTriangleCurrent<T: Float> {
    surface: TriangleSurface<T>,
    tangential_current: Vector3<T>,
}

#[allow(non_snake_case)]
impl<T: Float> PrecomputedTriangleCurrent<T> {
    pub fn new(current_density: Vector3<T>, vertices: &[Vector3<T>; 3]) -> Option<Self> {
        if current_density == Vector3::zeros() {
            return None;
        }
        let surface = TriangleSurface::new(*vertices)?;
        let tangential_current =
            current_density - surface.normal * current_density.dot(&surface.normal);
        Some(Self {
            surface,
            tangential_current,
        })
    }

    #[inline]
    pub fn compute_B(&self, point: Point3<T>) -> Vector3<T> {
        // Biot–Savart and magnetic surface charge share the same geometric
        // integral of (observer - surface) / distance^3 over the triangle.
        let (field, _) = self.surface.field_with_solid_angle(point);
        let field = self.tangential_current.cross(&field) * T::mu0();
        if field.iter().any(|v| v.is_nan()) {
            Vector3::zeros()
        } else {
            field
        }
    }
}

/// Computes B-field of a triangular current sheet at point in local frame.
#[inline]
#[allow(non_snake_case)]
#[num_lazy_proc::generic_float(T)]
pub fn local_triangle_current_B<T: Float>(
    point: Point3<T>,
    current_density: Vector3<T>,
    vertices: &[Vector3<T>; 3],
) -> Vector3<T> {
    match PrecomputedTriangleCurrent::new(current_density, vertices) {
        Some(pre) => pre.compute_B(point),
        None => Vector3::zeros(),
    }
}

/// Computes B-field of a triangular current sheet at point (x, y, z).
#[inline]
#[allow(non_snake_case)]
pub fn triangle_current_B<T: Float>(
    point: Point3<T>,
    position: Point3<T>,
    orientation: UnitQuaternion<T>,
    current_density: Vector3<T>,
    vertices: [Vector3<T>; 3],
) -> Vector3<T> {
    compute_in_local!(
        local_triangle_current_B,
        point,
        position,
        orientation,
        (current_density, &vertices),
    )
}

/// Computes B-field at points in global frame for a triangular current sheet.
#[allow(non_snake_case)]
pub fn triangle_current_B_batch<T: Float>(
    points: &[Point3<T>],
    position: Point3<T>,
    orientation: UnitQuaternion<T>,
    current_density: Vector3<T>,
    vertices: [Vector3<T>; 3],
    out: &mut [Vector3<T>],
) {
    let pre = match PrecomputedTriangleCurrent::new(current_density, &vertices) {
        Some(p) => p,
        None => {
            out.fill(Vector3::zeros());
            return;
        }
    };
    let inv_orientation = orientation.inverse();
    impl_parallel!(
        rayon_threshold: 110,
        input: points,
        output: out,
        |p| {
            let local_point = inv_orientation * Point3::from(p.coords - position.coords);
            let local_b = pre.compute_B(local_point);
            orientation * local_b
        }
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{point, vector};

    #[test]
    fn nonfinite_observers_preserve_zero_field_convention() {
        let vertices = [Vector3::zeros(), Vector3::x(), Vector3::y()];
        let field = local_triangle_current_B(point![f64::NAN, 0.0, 1.0], Vector3::x(), &vertices);
        assert_eq!(field, Vector3::zeros());
    }

    #[test]
    fn test_triangle_current_edge_cases_and_batch() {
        let verts = [
            vector![0.0_f64, 0.0, 0.0],
            vector![1.0, 0.0, 0.0],
            vector![0.0, 1.0, 0.0],
        ];

        // Small negative z (|z| < 1e-15, z < 0)
        let b_neg_z =
            local_triangle_current_B(point![0.2, 0.2, -1e-16], vector![1.0, 0.0, 0.0], &verts);
        assert!(b_neg_z.z.is_finite());

        // Zero current density
        let b_zero_curr =
            local_triangle_current_B(point![0.2, 0.2, 1.0], vector![0.0, 0.0, 0.0], &verts);
        assert_eq!(b_zero_curr, Vector3::zeros());

        // Duplicate vertices 0 and 1 (u1 < 1e-15)
        let b_dup_v = local_triangle_current_B(
            point![0.2, 0.2, 1.0],
            vector![1.0, 0.0, 0.0],
            &[
                vector![0.0, 0.0, 0.0],
                vector![0.0, 0.0, 0.0],
                vector![0.0, 1.0, 0.0],
            ],
        );
        assert_eq!(b_dup_v, Vector3::zeros());

        // Collinear vertices (n_norm < 1e-15)
        let b_collinear = local_triangle_current_B(
            point![0.2, 0.2, 1.0],
            vector![1.0, 0.0, 0.0],
            &[
                vector![0.0, 0.0, 0.0],
                vector![1.0, 0.0, 0.0],
                vector![2.0, 0.0, 0.0],
            ],
        );
        assert_eq!(b_collinear, Vector3::zeros());

        // Batch with zero current density
        let mut out_zero = vec![Vector3::zeros(); 5];
        triangle_current_B_batch(
            &[point![0.0, 0.0, 5.0]; 5],
            Point3::origin(),
            UnitQuaternion::identity(),
            Vector3::zeros(),
            verts,
            &mut out_zero,
        );
        assert_eq!(out_zero[0], Vector3::zeros());

        // Batch with <= 100 points for serial threshold (verified against magpylib)
        let small_points = [
            point![0.0, 0.0, 2.0],
            point![0.5, 0.0, 2.0],
            point![0.0, 0.5, 2.0],
        ];
        let mut small_out = vec![Vector3::zeros(); 3];
        triangle_current_B_batch(
            &small_points,
            Point3::origin(),
            UnitQuaternion::identity(),
            vector![1.0, 0.0, 0.0],
            verts,
            &mut small_out,
        );
        let expected = [
            vector![0.0, -1.1134101432626334e-08, -1.8127381590571603e-09],
            vector![0.0, -1.1474146463975746e-08, -1.8188154146576577e-09],
            vector![0.0, -1.1474146463975736e-08, 8.925329226670037e-10],
        ];
        for (o, e) in small_out.iter().zip(expected.iter()) {
            approx::assert_relative_eq!(o, e, epsilon = 1e-14, max_relative = 1e-12);
        }

        // Batch with > 100 points for Rayon threshold
        let points = vec![point![0.0, 0.0, 5.0]; 120];
        let mut out = vec![Vector3::zeros(); 120];
        triangle_current_B_batch(
            &points,
            Point3::origin(),
            UnitQuaternion::identity(),
            vector![1.0, 0.0, 0.0],
            verts,
            &mut out,
        );
        assert_eq!(out.len(), 120);
        assert!(out[0].z != 0.0);
    }
}
