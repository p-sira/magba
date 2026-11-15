/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Analytical B-field computation for homogeneously magnetized triangular surface.

use nalgebra::{Point3, UnitQuaternion, Vector3};

use crate::{
    base::{Float, coordinate::compute_in_local},
    crate_utils::impl_parallel,
};

use super::triangle_surface::TriangleSurface;

/// Observer-independent geometry and surface charge for a magnetized triangle.
#[derive(Clone, Copy)]
pub(crate) struct PrecomputedMagnetizedTriangle<T: Float> {
    surface: TriangleSurface<T>,
    sigma: T,
}

impl<T: Float> PrecomputedMagnetizedTriangle<T> {
    #[inline]
    pub(crate) fn new(polarization: Vector3<T>, vertices: [Vector3<T>; 3]) -> Option<Self> {
        let surface = TriangleSurface::new(vertices)?;
        let sigma = surface.normal.dot(&polarization);
        Some(Self { surface, sigma })
    }

    #[inline]
    #[allow(non_snake_case)]
    pub(crate) fn compute_B(&self, point: Point3<T>) -> Vector3<T> {
        if self.sigma == T::zero() {
            Vector3::zeros()
        } else {
            self.compute_B_with_solid_angle(point).0
        }
    }

    #[inline]
    #[allow(non_snake_case)]
    pub(crate) fn compute_B_with_solid_angle(&self, point: Point3<T>) -> (Vector3<T>, T) {
        let (field, omega) = self.surface.field_with_solid_angle(point);
        let field = field * self.sigma;
        let field = if field.iter().any(|v| v.is_nan()) {
            Vector3::zeros()
        } else {
            field
        };
        (field, omega)
    }
}

/// Computes B-field of a homogeneously magnetized triangular surface at point in local frame.
///
/// The charge is proportional to the projection of the polarization vectors onto the
/// triangle surfaces. The order of the triangle vertices defines the sign of the
/// surface normal vector (right-hand-rule).
///
/// # Arguments
///
/// - `point`: Observer position (m)
/// - `polarization`: Polarization vector (T)
/// - `vertices`: Triangle vertices `[P1, P2, P3]` in local coords (m)
///
/// # Returns
///
/// - B-field vector (T) at point (x, y, z)
///
/// # References
///
/// - Guptasarma, D., and B. Singh. "New scheme for computing the magnetic field of a flat triangular surface." Geophysics 64.1 (1999): 70-74.
/// - Ortner, Michael, and Lucas Gabriel Coliado Bandeira. “Magpylib: A Free Python Package for Magnetic Field Computation.” SoftwareX 11 (January 1, 2020): 100466. <https://doi.org/10.1016/j.softx.2020.100466>.
#[inline]
#[allow(non_snake_case)]
#[num_lazy_proc::generic_float(T)]
pub(crate) fn local_triangle_B_with_solid_angle<T: Float>(
    point: Point3<T>,
    polarization: Vector3<T>,
    vertices: [Vector3<T>; 3],
) -> (Vector3<T>, T) {
    match PrecomputedMagnetizedTriangle::new(polarization, vertices) {
        Some(precomputed) => precomputed.compute_B_with_solid_angle(point),
        None => (Vector3::zeros(), T::zero()),
    }
}

/// Computes B-field of a homogeneously magnetized triangular surface at point in local frame.
///
/// The charge is proportional to the projection of the polarization vectors onto the
/// triangle surfaces. The order of the triangle vertices defines the sign of the
/// surface normal vector (right-hand-rule).
///
/// # Arguments
///
/// - `point`: Observer position (m)
/// - `polarization`: Polarization vector (T)
/// - `vertices`: Triangle vertices `[P1, P2, P3]` in local coords (m)
///
/// # Returns
///
/// - B-field vector (T) at point (x, y, z)
///
/// # References
///
/// - Guptasarma, D., and B. Singh. "New scheme for computing the magnetic field of a flat triangular surface." Geophysics 64.1 (1999): 70-74.
/// - Ortner, Michael, and Lucas Gabriel Coliado Bandeira. “Magpylib: A Free Python Package for Magnetic Field Computation.” SoftwareX 11 (January 1, 2020): 100466. <https://doi.org/10.1016/j.softx.2020.100466>.
#[inline]
#[allow(non_snake_case)]
pub fn local_triangle_B<T: Float>(
    point: Point3<T>,
    polarization: Vector3<T>,
    vertices: [Vector3<T>; 3],
) -> Vector3<T> {
    local_triangle_B_with_solid_angle(point, polarization, vertices).0
}

/// Computes B-field of a homogeneously magnetized triangular surface at point (x, y, z).
///
/// # Arguments
///
/// - `point`: Observer position (m)
/// - `position`: Element center/position (m) (defaults to zero in Magnet struct)
/// - `orientation`: Element orientation in unit quaternion
/// - `polarization`: Polarization vector (T)
/// - `vertices`: Triangle vertices in local coords (m)
///
/// # Returns
///
/// - B-field vector (T) at point (x, y, z)
#[inline]
#[allow(non_snake_case)]
pub fn triangle_B<T: Float>(
    point: Point3<T>,
    position: Point3<T>,
    orientation: UnitQuaternion<T>,
    polarization: Vector3<T>,
    vertices: [Vector3<T>; 3],
) -> Vector3<T> {
    compute_in_local!(
        local_triangle_B,
        point,
        position,
        orientation,
        (polarization, vertices),
    )
}

/// Computes B-field at points in global frame for a triangular surface.
///
/// # Arguments
///
/// - `points`: Observer positions (m)
/// - `position`: Element position (m)
/// - `orientation`: Element orientation in unit quaternion
/// - `polarization`: Polarization vector (T)
/// - `vertices`: Triangle vertices in local coords (m)
/// - `out`: Mutable slice to store the B-field vectors at each observer (T)
#[allow(non_snake_case)]
pub fn triangle_B_batch<T: Float>(
    points: &[Point3<T>],
    position: Point3<T>,
    orientation: UnitQuaternion<T>,
    polarization: Vector3<T>,
    vertices: [Vector3<T>; 3],
    out: &mut [Vector3<T>],
) {
    let precomputed = match PrecomputedMagnetizedTriangle::new(polarization, vertices) {
        Some(precomputed) => precomputed,
        None => {
            assert_eq!(
                out.len(),
                points.len(),
                "Output slice length must match input vectors length."
            );
            out.fill(Vector3::zeros());
            return;
        }
    };
    let inv_orientation = orientation.inverse();
    impl_parallel!(
        rayon_threshold: 1085,
        input: points,
        output: out,
        |p| {
            let local_point = inv_orientation * Point3::from(p.coords - position.coords);
            let local_b = precomputed.compute_B(local_point);
            orientation * local_b
        }
    );
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;
    use nalgebra::{point, vector};

    use super::*;

    #[test]
    fn test_local_triangle_b() {
        // Test values compared with Magpylib examples
        let vertices = [
            vector![0.0, 0.0, 0.0],
            vector![0.0, 0.0, 1.0],
            vector![1.0, 0.0, 0.0],
        ];

        let p1 = point![2.0, 1.0, 1.0];
        let p2 = point![2.0, 2.0, 2.0];

        let b1 = local_triangle_B(p1, vector![1000.0, 1000.0, 1000.0], vertices);
        let b2 = local_triangle_B(p2, vector![1000.0, 1000.0, 0.0], vertices);

        // Values from `triangle_Bfield` magpylib docstring example:
        // [[7.452 4.62  3.136]
        //  [2.213 2.677 2.213]]
        assert_relative_eq!(
            b1,
            vector![7.451589646328714, 4.619948660698552, 3.1361413170448813],
            epsilon = 1e-12
        );
        assert_relative_eq!(
            b2,
            vector![2.2134561841724194, 2.677101477982004, 2.2134561841724327],
            epsilon = 1e-12
        );
    }

    #[test]
    fn precomputed_triangle_batch_preserves_rotated_f32_and_zero_area() {
        let vertices = [
            vector![0.0_f32, 0.0, 0.0],
            vector![0.3, 0.0, 0.0],
            vector![0.05, 0.4, 0.0],
        ];
        let polarization = vector![0.2_f32, -0.4, 0.9];
        let position = point![0.02_f32, -0.03, 0.04];
        let orientation = UnitQuaternion::from_scaled_axis(vector![0.3_f32, -0.2, 0.4]);
        let points = [
            point![0.2_f32, 0.3, 0.5],
            point![-0.2, 0.1, 0.04],
            point![0.04, -0.03, -0.4],
            point![f32::NAN, 0.0, 0.0],
        ];
        let mut out = [Vector3::zeros(); 4];
        triangle_B_batch(
            &points,
            position,
            orientation,
            polarization,
            vertices,
            &mut out,
        );

        for (&point, actual) in points.iter().zip(out.iter()) {
            assert_relative_eq!(
                *actual,
                triangle_B(point, position, orientation, polarization, vertices),
                epsilon = 1e-7,
                max_relative = 1e-5
            );
        }

        let zero_area = [vertices[0], vertices[1], vertices[1] * 2.0];
        triangle_B_batch(
            &points,
            position,
            orientation,
            polarization,
            zero_area,
            &mut out,
        );
        assert!(out.iter().all(|value| *value == Vector3::zeros()));
    }

    #[test]
    fn invalid_geometry_keeps_original_nan_solid_angle_behavior() {
        let vertices = [
            vector![f64::NAN, 0.0, 0.0],
            vector![1.0, 0.0, 0.0],
            vector![0.0, 1.0, 0.0],
        ];
        let (field, angle) =
            local_triangle_B_with_solid_angle(point![0.0, 0.0, 1.0], Vector3::z(), vertices);
        assert_eq!(field, Vector3::zeros());
        assert!(angle.is_nan());
    }

    #[test]
    fn zero_surface_charge_keeps_solid_angle_for_meshes() {
        let vertices = [
            vector![0.0_f64, 0.0, 0.0],
            vector![1.0, 0.0, 0.0],
            vector![0.0, 1.0, 0.0],
        ];
        let triangle = PrecomputedMagnetizedTriangle::new(Vector3::x(), vertices).unwrap();
        let point = point![0.2, 0.2, 0.5];
        let (field, angle) = triangle.compute_B_with_solid_angle(point);
        assert_eq!(field, Vector3::zeros());
        assert_eq!(triangle.compute_B(point), Vector3::zeros());
        assert!(angle.abs() > 0.0);
    }

    #[test]
    fn test_triangle_edge_cases_and_batch() {
        // Colinear vertices (degenerate triangle)
        let b_collinear = local_triangle_B(
            point![0.0, 0.0, 1.0],
            vector![0.0, 0.0, 1.0],
            [
                vector![0.0, 0.0, 0.0],
                vector![1.0, 0.0, 0.0],
                vector![2.0, 0.0, 0.0],
            ],
        );
        assert_eq!(b_collinear, Vector3::zeros());

        // Observer collinear with an edge ray
        let b_edge_ext = local_triangle_B::<f64>(
            point![2.0, 0.0, 0.0],
            vector![0.0, 0.0, 1.0],
            [
                vector![0.0, 0.0, 0.0],
                vector![1.0, 0.0, 0.0],
                vector![0.0, 1.0, 0.0],
            ],
        );
        assert!(b_edge_ext.x.is_finite());

        // Observer with NaN
        let b_nan = local_triangle_B(
            point![f64::NAN, 0.0, 0.0],
            vector![0.0, 0.0, 1.0],
            [
                vector![0.0, 0.0, 0.0],
                vector![1.0, 0.0, 0.0],
                vector![0.0, 1.0, 0.0],
            ],
        );
        assert_eq!(b_nan, Vector3::zeros());

        // Batch with <= 300 points for serial threshold (verified against magpylib)
        let small_points = [
            point![0.0, 0.0, 2.0],
            point![0.5, 0.0, 2.0],
            point![0.0, 0.5, 2.0],
        ];
        let mut small_out = vec![Vector3::zeros(); 3];
        triangle_B_batch(
            &small_points,
            Point3::origin(),
            UnitQuaternion::identity(),
            vector![0.0, 0.0, 1.0],
            [
                vector![0.0, 0.0, 0.0],
                vector![1.0, 0.0, 0.0],
                vector![0.0, 1.0, 0.0],
            ],
            &mut small_out,
        );
        let expected = [
            vector![
                -0.001_442_531_192_916_713,
                -0.0014425311929167218,
                0.008860236400615005
            ],
            vector![
                0.0007102551326673291,
                -0.0014473673192635988,
                0.009130835638716228
            ],
            vector![
                -0.0014473673192635767,
                0.0007102551326673512,
                0.009130835638716228
            ],
        ];
        for (o, e) in small_out.iter().zip(expected.iter()) {
            approx::assert_relative_eq!(o, e, epsilon = 1e-14, max_relative = 1e-12);
        }

        // Batch with > 300 points for Rayon threshold
        let points_large = vec![point![0.0, 0.0, 5.0]; 350];
        let mut out_large = vec![Vector3::zeros(); 350];
        triangle_B_batch(
            &points_large,
            Point3::origin(),
            UnitQuaternion::identity(),
            vector![0.0, 0.0, 1.0],
            [
                vector![0.0, 0.0, 0.0],
                vector![1.0, 0.0, 0.0],
                vector![0.0, 1.0, 0.0],
            ],
            &mut out_large,
        );
        assert_eq!(out_large.len(), 350);
        assert!(out_large[0].z != 0.0);
    }
}
