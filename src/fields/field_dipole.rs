/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Analytical B-field computation for magnet dipole moment.

use nalgebra::{Point3, RealField, UnitQuaternion, Vector3};
use numeric_literals::replace_float_literals;

use crate::{
    crate_util::{impl_parallel, impl_parallel_sum},
    geometry::compute_in_local,
};

/// Compute B-field of a magnetic dipole moment at point (x, y, z) in local frame.
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
pub fn local_dipole_B<T: RealField + num_traits::Float + Copy>(
    point: Point3<T>,
    moment: Vector3<T>,
) -> Vector3<T> {
    let p = Vector3::from(point.coords);
    let r = p.norm();

    if r == T::zero() {
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

    (p * (3.0 * moment.component_mul(&p).sum() * (1.0 / num_traits::Float::powi(r, 5)))
        - moment * (1.0 / num_traits::Float::powi(r, 3)))
        * 1e-7
}

/// Compute B-field of a magnetic dipole moment at point (x, y, z).
///
/// Wrapper of [local_dipole_B].
#[inline]
#[allow(non_snake_case)]
pub fn dipole_B<T: RealField + num_traits::Float + Copy>(
    point: Point3<T>,
    position: Point3<T>,
    orientation: UnitQuaternion<T>,
    moment: Vector3<T>,
) -> Vector3<T> {
    compute_in_local!(local_dipole_B, point, position, orientation, (moment),)
}

/// Compute B-field at points in global frame for a magnetic dipole moment.
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
/// # use magba::assert_close_vec;
/// # use magba::fields::dipole_B;
/// # use nalgebra::*;
/// let mut out = [Vector3::zeros(); 1];
/// dipole_B(
///     &[point![5.0, 6.0, 7.0]],
///     &point![1.0, 2.0, 3.0],
///     &UnitQuaternion::from_scaled_axis(
///         [1.0471975511965976, 0.6283185307179586, 0.4487989505128276].into(),
///     ),
///     &vector![0.45, 0.3, 0.15],
///     &mut out,
/// );
/// assert_close_vec!(
///     out[0],
///     vector![
///         1.5509430032394472e-10,
///         1.8780091679184128e-10,
///         2.1982579999135383e-10
///     ],
///     2e-10
/// );
/// ```
///
/// # References
///
/// - Ortner, Michael, and Lucas Gabriel Coliado Bandeira. “Magpylib: A Free Python Package for Magnetic Field Computation.” SoftwareX 11 (January 1, 2020): 100466. <https://doi.org/10.1016/j.softx.2020.100466>.
#[allow(non_snake_case)]
pub fn dipole_B_batch<T: RealField + num_traits::Float + Copy>(
    points: &[Point3<T>],
    position: Point3<T>,
    orientation: UnitQuaternion<T>,
    moment: Vector3<T>,
    out: &mut [Vector3<T>],
) {
    impl_parallel!(
        out,
        dipole_B,
        60,
        points,
        position,
        orientation,
        moment
    )
}

/// Compute B-field at each given points in global frame for multiple magnetic dipole moments.
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
pub fn sum_multiple_dipole_B<T: RealField + num_traits::Float + Copy>(
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
