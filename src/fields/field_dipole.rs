/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Analytical B-field computation for magnet dipole moment.

use nalgebra::{Point3, UnitQuaternion, Vector3};
use numeric_literals::replace_float_literals;

use crate::{
    base::Float,
    crate_utils::{impl_parallel, impl_parallel_sum},
};

/// Computes B-field of a magnetic dipole moment at point (x, y, z) in local frame.
///
/// # Arguments
///
/// - `point`: Observer position (m)
/// - `moment`: Magnetic dipole moment vector (A·m²)
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
#[replace_float_literals(T::from_f64(literal).unwrap())]
pub fn local_dipole_B<T: Float>(point: Point3<T>, moment: Vector3<T>) -> Vector3<T> {
    let p = Vector3::from(point.coords);
    let r2 = p.norm_squared();

    if r2 == T::zero() {
        return Vector3::from_iterator(moment.iter().map(|&m| {
            if m > 0.0 {
                T::infinity()
            } else if m == 0.0 {
                T::zero()
            } else {
                T::neg_infinity()
            }
        }));
    }

    let r = num_traits::Float::sqrt(r2);
    let inv_r3 = 1.0 / (r2 * r);
    let inv_r5 = inv_r3 / r2;

    (p * (3.0 * moment.dot(&p) * inv_r5) - moment * inv_r3) * T::mu0_4pi()
}

/// Computes B-field of a magnetic dipole moment at point (x, y, z).
///
/// # Arguments
///
/// - `points`: Observer positions (m)
/// - `position`: Magnet position (m)
/// - `orientation`: Magnet orientation in unit quaternion
/// - `moment`: Magnetic dipole moment vector (A·m²)
/// - `out`: Mutable slice to store the B-field vectors at each observer (T)
///
/// # Examples
///
/// ```
/// # use approx::assert_relative_eq;
/// # use magba::fields::dipole_B;
/// # use nalgebra::*;
/// let b_field = dipole_B(
///     point![5.0, 6.0, 7.0],
///     point![1.0, 2.0, 3.0],
///     UnitQuaternion::from_scaled_axis(
///         [1.0471975511965976, 0.6283185307179586, 0.4487989505128276].into(),
///     ),
///     vector![0.45, 0.3, 0.15],
/// );
/// let expected = vector![1.5509430032394472e-10, 1.8780091679184128e-10, 2.1982579999135383e-10];
/// assert_relative_eq!(b_field, expected, epsilon = 2e-10);
/// ```
///
/// # References
///
/// - Ortner, Michael, and Lucas Gabriel Coliado Bandeira. “Magpylib: A Free Python Package for Magnetic Field Computation.” SoftwareX 11 (January 1, 2020): 100466. <https://doi.org/10.1016/j.softx.2020.100466>.
#[inline]
#[allow(non_snake_case)]
pub fn dipole_B<T: Float>(
    point: Point3<T>,
    position: Point3<T>,
    orientation: UnitQuaternion<T>,
    moment: Vector3<T>,
) -> Vector3<T> {
    let moment_global = orientation * moment;
    let p = Point3::from(point - position);
    local_dipole_B(p, moment_global)
}

/// Computes B-field at points in global frame for a magnetic dipole moment.
///
/// # Arguments
///
/// - `points`: Observer positions (m)
/// - `position`: Magnet position (m)
/// - `orientation`: Magnet orientation in unit quaternion
/// - `moment`: Magnetic dipole moment vector (A·m²)
/// - `out`: Mutable slice to store the B-field vectors at each observer (T)
///
/// # Examples
///
/// ```
/// # use approx::assert_relative_eq;
/// # use magba::fields::dipole_B_batch;
/// # use nalgebra::*;
/// let mut out = [Vector3::zeros(); 3];
/// dipole_B_batch(
///     &[
///         point![5.0, 6.0, 7.0],
///         point![4.0, 3.0, 2.0],
///         point![0.5, 0.25, 0.125],
///     ],
///     point![1.0, 2.0, 3.0],
///     UnitQuaternion::from_scaled_axis(
///         [1.0471975511965976, 0.6283185307179586, 0.4487989505128276].into(),
///     ),
///     vector![0.45, 0.3, 0.15],
///     &mut out,
/// );
///
/// let expected_fields = [
///     vector![
///         1.5509430032394459e-10,
///         1.8780091679184123e-10,
///         2.1982579999135385e-10,
///     ],
///     vector![
///         1.9129643501453957e-9,
///         1.6848048130527745e-10,
///         -1.5822176174352087e-9,
///     ],
///     vector![
///         -6.242720906265745e-10,
///         7.557286975903545e-10,
///         2.0195830850542665e-9,
///     ],
/// ];
///
/// out.iter()
///     .zip(expected_fields.iter())
///     .for_each(|(actual, expected)| assert_relative_eq!(actual, expected, epsilon = 1e-14));
/// ```
///
/// # References
///
/// - Ortner, Michael, and Lucas Gabriel Coliado Bandeira. “Magpylib: A Free Python Package for Magnetic Field Computation.” SoftwareX 11 (January 1, 2020): 100466. <https://doi.org/10.1016/j.softx.2020.100466>.
#[allow(non_snake_case)]
pub fn dipole_B_batch<T: Float>(
    points: &[Point3<T>],
    position: Point3<T>,
    orientation: UnitQuaternion<T>,
    moment: Vector3<T>,
    out: &mut [Vector3<T>],
) {
    let moment_global = orientation * moment;
    impl_parallel!(
        rayon_threshold: 24575,
        input: points,
        output: out,
        |p| {
            let disp = Point3::from(p - position);
            local_dipole_B(disp, moment_global)
        }
    );
}

/// Computes B-field at each given points in global frame for multiple magnetic dipole moments.
///
/// # Arguments
///
/// - `points`: Observer positions (m)
/// - `positions`: Magnet positions (m)
/// - `orientations`: Magnet orientations in unit quaternion
/// - `moments`: Magnetic dipole moment vectors (A·m²)
/// - `out`: Mutable slice to store the net B-field vectors at each observer (T)
///
/// # References
///
/// - Ortner, Michael, and Lucas Gabriel Coliado Bandeira. “Magpylib: A Free Python Package for Magnetic Field Computation.” SoftwareX 11 (January 1, 2020): 100466. <https://doi.org/10.1016/j.softx.2020.100466>.
#[allow(non_snake_case)]
pub fn sum_multiple_dipole_B<T: Float>(
    points: &[Point3<T>],
    positions: &[Point3<T>],
    orientations: &[UnitQuaternion<T>],
    moments: &[Vector3<T>],
    out: &mut [Vector3<T>],
) {
    impl_parallel_sum!(
        out,
        points,
        60,
        [positions, orientations, moments],
        |pos, p, o, m| dipole_B(*pos, *p, *o, *m)
    )
}

#[cfg(test)]
mod tests {
    use nalgebra::{point, vector};

    use super::*;

    #[test]
    fn test_sum_multiple_dipole_b() {
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
        let moments = &[vector![0.45, 0.3, 0.15], vector![1.0, 2.0, 3.0]];

        impl_test_sum_multiple!(
            sum_multiple_dipole_B,
            2e-10,
            points,
            positions,
            orientations,
            (moments),
            |p, pos, ori, m| dipole_B(p, pos, ori, m)
        );
    }

    #[test]
    fn test_dipole_at_origin() {
        let b = local_dipole_B(point![0.0, 0.0, 0.0], vector![-1.0, 0.0, 1.0]);
        assert_eq!(b.x, f64::NEG_INFINITY);
        assert_eq!(b.y, 0.0);
        assert_eq!(b.z, f64::INFINITY);
    }
}
