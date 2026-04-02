/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Analytical B-field computation for homogeneously magnetized triangular surface.

use nalgebra::{Point3, UnitQuaternion, Vector3};
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
fn solid_angle<T: Float>(r_vecs: &[Vector3<T>; 3], r_mags: &[T; 3]) -> T {
    let N = r_vecs[2].dot(&r_vecs[1].cross(&r_vecs[0]));

    let D = r_mags[0] * r_mags[1] * r_mags[2]
        + r_vecs[2].dot(&r_vecs[1]) * r_mags[0]
        + r_vecs[2].dot(&r_vecs[0]) * r_mags[1]
        + r_vecs[1].dot(&r_vecs[0]) * r_mags[2];

    let result = 2.0 * num_traits::Float::atan2(N, D);

    // Modulus 2pi to avoid jumps on edges in line
    if num_traits::Float::abs(result) > 2.0 * T::pi() {
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
pub fn local_triangle_B<T: Float>(
    point: Point3<T>,
    polarization: Vector3<T>,
    vertices: [Vector3<T>; 3],
) -> Vector3<T> {
    let p = Vector3::from(point.coords);

    // Normal vector
    let a = vertices[1] - vertices[0];
    let b = vertices[2] - vertices[0];
    let n_cross = a.cross(&b);
    let n_norm = n_cross.norm();

    if n_norm == T::zero() {
        return Vector3::zeros();
    }
    let n = n_cross / n_norm;

    let sigma = n.dot(&polarization);

    // vertex <-> observer
    let r_vecs = [vertices[0] - p, vertices[1] - p, vertices[2] - p];
    let r_mags = [r_vecs[0].norm(), r_vecs[1].norm(), r_vecs[2].norm()];

    // vertex <-> vertex
    let L = [
        vertices[1] - vertices[0],
        vertices[2] - vertices[1],
        vertices[0] - vertices[2],
    ];
    let l_mags = [L[0].norm(), L[1].norm(), L[2].norm()];

    let b_vals = [
        r_vecs[0].dot(&L[0]),
        r_vecs[1].dot(&L[1]),
        r_vecs[2].dot(&L[2]),
    ];

    let mut PQR = Vector3::zeros();

    for i in 0..3 {
        let bl_val = b_vals[i] / l_mags[i];
        let ind = num_traits::Float::abs(r_mags[i] + bl_val);

        let I;
        if ind > 1.0e-12 {
            I = (1.0 / l_mags[i])
                * num_traits::Float::ln(
                    (num_traits::Float::sqrt(
                        l_mags[i] * l_mags[i] + 2.0 * b_vals[i] + r_mags[i] * r_mags[i],
                    ) + l_mags[i]
                        + bl_val)
                        / ind,
                );
        } else {
            I = -(1.0 / l_mags[i])
                * num_traits::Float::ln(num_traits::Float::abs(l_mags[i] - r_mags[i]) / r_mags[i]);
        }

        // Accumulate I * L
        PQR += L[i] * I;
    }

    let mut B = (n * solid_angle(&r_vecs, &r_mags) - n.cross(&PQR)) * sigma;
    B /= 4.0 * T::pi();

    if B.x.is_nan() || B.y.is_nan() || B.z.is_nan() {
        Vector3::zeros()
    } else {
        B
    }
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
    impl_parallel!(
        triangle_B,
        rayon_threshold: 3100, // Roughly similar to others
        input: points,
        output: out,
        args: [position, orientation, polarization, vertices]
    )
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
}
