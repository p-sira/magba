/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Analytical B-field computation for a current path (a sequence of straight wire segments).

use nalgebra::{Point3, UnitQuaternion, Vector3};

#[cfg(feature = "rayon")]
use crate::crate_utils::adaptive_rayon_threshold;
use crate::{
    base::{Float, coordinate::compute_in_local},
    crate_utils::impl_parallel,
};

/// Computes B-field of a current path (sequence of straight wire segments) at point in local frame.
///
/// # Arguments
///
/// - `point`: Observer position (m)
/// - `current`: Current (A)
/// - `vertices`: Vertices defining the current path `[P1, P2, ...]` in local coords (m)
///
/// # Returns
///
/// - B-field vector (T) at point (x, y, z)
///
/// # References
///
/// - Ortner, Michael, and Lucas Gabriel Coliado Bandeira. “Magpylib: A Free Python Package for Magnetic Field Computation.” SoftwareX 11 (January 1, 2020): 100466. <https://doi.org/10.1016/j.softx.2020.100466>.
#[derive(Clone, Copy)]
struct PrecomputedSegment<T: Float> {
    start: Vector3<T>,
    direction: Vector3<T>,
    inv_length: T,
}

impl<T: Float> PrecomputedSegment<T> {
    fn from_points(start: Vector3<T>, end: Vector3<T>) -> Option<Self> {
        let delta = end - start;
        let length = delta.norm();
        if length == T::zero() {
            return None;
        }
        let inv_length = T::one() / length;
        Some(Self {
            start,
            direction: delta * inv_length,
            inv_length,
        })
    }

    #[inline]
    #[allow(non_snake_case)]
    #[num_lazy_proc::generic_float(T)]
    fn compute_B(&self, po: Vector3<T>, current_term: T) -> Vector3<T> {
        // Subtract before scaling so large vertex offsets do not consume the
        // precision of a short segment's geometry.
        let displacement = (po - self.start) * self.inv_length;
        let along = displacement.dot(&self.direction);
        let cross = self.direction.cross(&displacement);
        let perpendicular_squared = cross.norm_squared();
        if perpendicular_squared < 1e-30 {
            return Vector3::zeros();
        }
        let first = num_traits::Float::sqrt(along * along + perpendicular_squared);
        let other = along - 1.0;
        let second = num_traits::Float::sqrt(other * other + perpendicular_squared);
        let factor = if along < 0.0 || along > 1.0 {
            // Beyond an endpoint the cosines nearly agree. Rationalize their
            // difference; far² - near² = far + near for a unit-length segment.
            let (near, far, a, b) = if along < 0.0 {
                (-along, 1.0 - along, first, second)
            } else {
                (along - 1.0, along, second, first)
            };
            (far + near) / (a * b * (far * a + near * b))
        } else {
            (along / first + (1.0 - along) / second) / perpendicular_squared
        };
        cross * (factor * self.inv_length * current_term)
    }
}

/// Computes B-field of a current path (sequence of straight wire segments) at point in local frame.
///
/// # Arguments
///
/// - `point`: Observer position (m)
/// - `current`: Current (A)
/// - `vertices`: Vertices defining the current path `[P1, P2, ...]` in local coords (m)
///
/// # Returns
///
/// - B-field vector (T) at point (x, y, z)
///
/// # References
///
/// - Ortner, Michael, and Lucas Gabriel Coliado Bandeira. “Magpylib: A Free Python Package for Magnetic Field Computation.” SoftwareX 11 (January 1, 2020): 100466. <https://doi.org/10.1016/j.softx.2020.100466>.
#[inline]
#[allow(non_snake_case)]
#[num_lazy_proc::generic_float(T)]
pub fn local_path_current_B<T: Float>(
    point: Point3<T>,
    current: T,
    vertices: &[Vector3<T>],
) -> Vector3<T> {
    if vertices.len() < 2 || current == T::zero() {
        return Vector3::zeros();
    }

    let po = Vector3::from(point.coords);
    let current_term = current * T::mu0_4pi();
    let mut b_total = Vector3::zeros();

    for i in 0..vertices.len() - 1 {
        if let Some(seg) = PrecomputedSegment::from_points(vertices[i], vertices[i + 1]) {
            b_total += seg.compute_B(po, current_term);
        }
    }

    if b_total.x.is_nan() || b_total.y.is_nan() || b_total.z.is_nan() {
        Vector3::zeros()
    } else {
        b_total
    }
}

/// Computes B-field of a current path at point (x, y, z).
///
/// # Arguments
///
/// - `point`: Observer position (m)
/// - `position`: Element position (m)
/// - `orientation`: Element orientation in unit quaternion
/// - `current`: Current (A)
/// - `vertices`: Vertices defining the current path in local coords (m)
///
/// # Returns
///
/// - B-field vector (T) at point (x, y, z)
#[inline]
#[allow(non_snake_case)]
pub fn path_current_B<T: Float>(
    point: Point3<T>,
    position: Point3<T>,
    orientation: UnitQuaternion<T>,
    current: T,
    vertices: &[Vector3<T>],
) -> Vector3<T> {
    compute_in_local!(
        local_path_current_B,
        point,
        position,
        orientation,
        (current, vertices),
    )
}

/// Computes B-field at points in global frame for a current path.
///
/// # Arguments
///
/// - `points`: Observer positions (m)
/// - `position`: Element position (m)
/// - `orientation`: Element orientation in unit quaternion
/// - `current`: Current (A)
/// - `vertices`: Vertices defining the current path in local coords (m)
/// - `out`: Mutable slice to store the B-field vectors at each observer (T)
#[allow(non_snake_case)]
pub fn path_current_B_batch<T: Float>(
    points: &[Point3<T>],
    position: Point3<T>,
    orientation: UnitQuaternion<T>,
    current: T,
    vertices: &[Vector3<T>],
    out: &mut [Vector3<T>],
) {
    assert_eq!(
        out.len(),
        points.len(),
        "Output slice length must match input vectors length."
    );
    if vertices.len() < 2 || current == T::zero() {
        out.fill(Vector3::zeros());
        return;
    }

    let segments: alloc::vec::Vec<_> = vertices
        .windows(2)
        .filter_map(|w| PrecomputedSegment::from_points(w[0], w[1]))
        .collect();
    let current_term = current * T::mu0_4pi();
    let inv_orientation = orientation.inverse();
    // Tuned with three active segments at a 185-observer crossover.
    #[cfg(feature = "rayon")]
    const PARALLEL_WORK_THRESHOLD: usize = 555;
    impl_parallel!(
        rayon_threshold: adaptive_rayon_threshold(PARALLEL_WORK_THRESHOLD, segments.len()),
        input: points,
        output: out,
        |p| {
            let local_p = inv_orientation * Point3::from(p.coords - position.coords);
            let mut b_total = Vector3::zeros();
            for seg in &segments {
                b_total += seg.compute_B(local_p.coords, current_term);
            }
            if b_total.x.is_nan() || b_total.y.is_nan() || b_total.z.is_nan() {
                Vector3::zeros()
            } else {
                orientation * b_total
            }
        }
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{point, vector};

    #[test]
    fn test_path_current_edge_cases_and_batch() {
        // Less than 2 vertices
        let b_less = local_path_current_B(point![0.0, 0.0, 1.0], 1.0, &[vector![0.0, 0.0, 0.0]]);
        assert_eq!(b_less, Vector3::zeros());

        // Zero-length segment (duplicate vertex)
        let b_zero_seg = local_path_current_B(
            point![0.0, 0.0, 1.0],
            1.0,
            &[vector![0.0, 0.0, 0.0], vector![0.0, 0.0, 0.0]],
        );
        assert_eq!(b_zero_seg, Vector3::zeros());

        // Collinear observer with segment (norm_o4 < 1e-15)
        let b_collinear = local_path_current_B(
            point![0.0, 0.0, 0.5],
            1.0,
            &[vector![0.0, 0.0, 0.0], vector![0.0, 0.0, 1.0]],
        );
        assert_eq!(b_collinear, Vector3::zeros());

        // Collinear observer on extension of segment
        let b_ext = local_path_current_B(
            point![0.0, 0.0, 2.0],
            1.0,
            &[vector![0.0, 0.0, 0.0], vector![0.0, 0.0, 1.0]],
        );
        assert_eq!(b_ext, Vector3::zeros());

        // NaN observer
        let b_nan = local_path_current_B(
            point![f64::NAN, 0.0, 0.0],
            1.0,
            &[vector![0.0, 0.0, 0.0], vector![0.0, 0.0, 1.0]],
        );
        assert_eq!(b_nan, Vector3::zeros());

        let verts = [vector![0.0, 0.0, 0.0], vector![0.0, 0.0, 1.0]];

        // Batch with NaN observer
        let mut out_nan = vec![Vector3::zeros(); 5];
        path_current_B_batch(
            &[point![f64::NAN, 0.0, 0.0]; 5],
            Point3::origin(),
            UnitQuaternion::identity(),
            1.0,
            &verts,
            &mut out_nan,
        );
        assert_eq!(out_nan[0], Vector3::zeros());

        // Batch with current == 0.0
        let mut out_zero = vec![Vector3::zeros(); 5];
        path_current_B_batch(
            &[point![0.0, 1.0, 0.0]; 5],
            Point3::origin(),
            UnitQuaternion::identity(),
            0.0,
            &verts,
            &mut out_zero,
        );
        assert_eq!(out_zero[0], Vector3::zeros());

        // Batch with < 2 vertices
        let mut out_less = vec![Vector3::zeros(); 5];
        path_current_B_batch(
            &[point![0.0, 1.0, 0.0]; 5],
            Point3::origin(),
            UnitQuaternion::identity(),
            1.0,
            &[vector![0.0, 0.0, 0.0]],
            &mut out_less,
        );
        assert_eq!(out_less[0], Vector3::zeros());

        // Batch with <= 200 points to trigger serial threshold (verified against magpylib)
        let small_points = [
            point![0.0, 1.0, 0.0],
            point![0.5, 1.0, 0.0],
            point![0.0, 1.0, 0.5],
        ];
        let mut small_out = vec![Vector3::zeros(); 3];
        path_current_B_batch(
            &small_points,
            Point3::origin(),
            UnitQuaternion::identity(),
            1.0,
            &verts,
            &mut small_out,
        );
        let expected = [
            vector![-7.071067810931863e-08, 0.0, 0.0],
            vector![-5.333333332629158e-08, 2.666666666314579e-08, 0.0],
            vector![-8.944271908818222e-08, 0.0, 0.0],
        ];
        for (o, e) in small_out.iter().zip(expected.iter()) {
            approx::assert_relative_eq!(o, e, epsilon = 1e-14, max_relative = 1e-12);
        }

        // Batch with > 200 points to trigger Rayon threshold
        let points = vec![point![0.0, 1.0, 0.0]; 250];
        let mut out = vec![Vector3::zeros(); 250];
        path_current_B_batch(
            &points,
            Point3::origin(),
            UnitQuaternion::identity(),
            1.0,
            &verts,
            &mut out,
        );
        assert_eq!(out.len(), 250);
        assert!(out[0].x != 0.0);
    }
}
