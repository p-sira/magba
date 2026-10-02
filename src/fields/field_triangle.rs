/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Analytical B-field computation for homogeneously magnetized triangular surface.

use nalgebra::{Point3, UnitQuaternion, Vector3};
use num_traits::Float as NumFloat;
use numeric_literals::replace_float_literals;

use crate::{
    base::{Float, coordinate::compute_in_local},
    crate_utils::{impl_parallel, impl_parallel_sum},
};

/// Computes solid angle of a triangle given its local R vectors from observer to vertices.
/// Vector elements are (R1, R2, R3).
///
/// Implements vectorized solid angle computation based on Magpylib's implementation.
#[inline]
#[allow(non_snake_case)]
#[replace_float_literals(T::from_f64(literal).unwrap())]
pub(crate) fn solid_angle<T: Float>(r_vecs: &[Vector3<T>; 3], r_mags: &[T; 3]) -> T {
    let N = r_vecs[2].dot(&r_vecs[1].cross(&r_vecs[0]));

    let D = r_mags[0] * r_mags[1] * r_mags[2]
        + r_vecs[2].dot(&r_vecs[1]) * r_mags[0]
        + r_vecs[2].dot(&r_vecs[0]) * r_mags[1]
        + r_vecs[1].dot(&r_vecs[0]) * r_mags[2];

    let result = 2.0 * NumFloat::atan2(N, D);

    // Modulus 2pi to avoid jumps on edges in line
    if NumFloat::abs(result) > 2.0 * T::pi() {
        T::zero()
    } else {
        result
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
#[replace_float_literals(T::from_f64(literal).unwrap())]
pub(crate) fn local_triangle_B_with_solid_angle<T: Float>(
    point: Point3<T>,
    polarization: Vector3<T>,
    vertices: [Vector3<T>; 3],
) -> (Vector3<T>, T) {
    let p = Vector3::from(point.coords);

    // Normal vector
    let a = vertices[1] - vertices[0];
    let b = vertices[2] - vertices[0];
    let n_cross = a.cross(&b);
    let n_norm = n_cross.norm();

    if n_norm == T::zero() {
        return (Vector3::zeros(), T::zero());
    }
    let n = n_cross / n_norm;

    let sigma = n.dot(&polarization);

    // vertex <-> observer
    let r_vecs = [vertices[0] - p, vertices[1] - p, vertices[2] - p];
    let r_sq = [
        r_vecs[0].norm_squared(),
        r_vecs[1].norm_squared(),
        r_vecs[2].norm_squared(),
    ];
    let r_mags = [
        NumFloat::sqrt(r_sq[0]),
        NumFloat::sqrt(r_sq[1]),
        NumFloat::sqrt(r_sq[2]),
    ];

    let omega = solid_angle(&r_vecs, &r_mags);

    // vertex <-> vertex
    let L = [
        vertices[1] - vertices[0],
        vertices[2] - vertices[1],
        vertices[0] - vertices[2],
    ];
    let l_sq = [
        L[0].norm_squared(),
        L[1].norm_squared(),
        L[2].norm_squared(),
    ];
    let l_mags = [
        NumFloat::sqrt(l_sq[0]),
        NumFloat::sqrt(l_sq[1]),
        NumFloat::sqrt(l_sq[2]),
    ];

    let b_vals = [
        r_vecs[0].dot(&L[0]),
        r_vecs[1].dot(&L[1]),
        r_vecs[2].dot(&L[2]),
    ];

    let mut PQR = Vector3::zeros();

    for i in 0..3 {
        let bl_val = b_vals[i] / l_mags[i];
        let ind = NumFloat::abs(r_mags[i] + bl_val);

        let I = if ind > 1.0e-12 {
            (1.0 / l_mags[i])
                * NumFloat::ln(
                    (NumFloat::sqrt(l_sq[i] + 2.0 * b_vals[i] + r_sq[i]) + l_mags[i] + bl_val)
                        / ind,
                )
        } else {
            -(1.0 / l_mags[i]) * NumFloat::ln(NumFloat::abs(l_mags[i] - r_mags[i]) / r_mags[i])
        };

        // Accumulate I * L
        PQR += L[i] * I;
    }

    let mut B = (n * omega - n.cross(&PQR)) * sigma;
    B /= 4.0 * T::pi();

    let B = if B.x.is_nan() || B.y.is_nan() || B.z.is_nan() {
        Vector3::zeros()
    } else {
        B
    };

    (B, omega)
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
    let inv_orientation = orientation.inverse();
    impl_parallel!(
        rayon_threshold: 250,
        input: points,
        output: out,
        |p| {
            let local_point = inv_orientation * Point3::from(p.coords - position.coords);
            let local_b = local_triangle_B(local_point, polarization, vertices);
            orientation * local_b
        }
    );
}

/// Computes B-field at each given points in global frame for multiple triangles.
///
/// # Arguments
///
/// - `points`: Observer positions (m)
/// - `positions`: Element positions (m)
/// - `orientations`: Element orientations in unit quaternion
/// - `polarizations`: Polarization vectors (T)
/// - `vertices_list`: List of triangle vertices arrays `[[P1, P2, P3], ...]` in local coords (m)
/// - `out`: Mutable slice to store the net B-field vectors at each observer (T)
#[allow(non_snake_case)]
pub fn sum_multiple_triangle_B<T: Float>(
    points: &[Point3<T>],
    positions: &[Point3<T>],
    orientations: &[UnitQuaternion<T>],
    polarizations: &[Vector3<T>],
    vertices_list: &[[Vector3<T>; 3]],
    out: &mut [Vector3<T>],
) {
    impl_parallel_sum!(
        out,
        points,
        60,
        [positions, orientations, polarizations, vertices_list],
        |pos, p, o, pol, vert| triangle_B(*pos, *p, *o, *pol, *vert)
    )
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
    fn test_sum_multiple_triangle_b() {
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
            ],
            [
                vector![0.0, 0.0, 0.0],
                vector![0.0, 1.0, 0.0],
                vector![0.0, 0.0, 1.0],
            ],
        ];

        impl_test_sum_multiple!(
            sum_multiple_triangle_B,
            1e-15,
            points,
            positions,
            orientations,
            (polarizations, vertices_list),
            |p, pos, ori, pol, vert| triangle_B(p, pos, ori, pol, vert)
        );
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
        let b_edge_ext = local_triangle_B(
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
