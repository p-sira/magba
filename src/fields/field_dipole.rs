/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use nalgebra::{Point3, RealField, UnitQuaternion, Vector3};
use numeric_literals::replace_float_literals;

use crate::{
    crate_util::{assert_eq_lens, impl_parallel, impl_parallel_sum},
    geometry::compute_in_local,
};

#[allow(non_snake_case)]
#[replace_float_literals(T::from_f64(literal).unwrap())]
pub fn local_dipole_B<T: RealField + num_traits::Float + Copy>(
    point: &Point3<T>,
    moment: &Vector3<T>,
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

#[allow(non_snake_case)]
pub fn global_dipole_B<T: RealField + num_traits::Float + Copy>(
    point: &Point3<T>,
    position: &Point3<T>,
    orientation: &UnitQuaternion<T>,
    moment: &Vector3<T>,
) -> Vector3<T> {
    compute_in_local!(local_dipole_B, point, position, orientation, (moment),)
}

#[allow(non_snake_case)]
pub fn dipole_B<T: RealField + num_traits::Float + Copy>(
    points: &[Point3<T>],
    position: &Point3<T>,
    orientation: &UnitQuaternion<T>,
    moment: &Vector3<T>,
) -> Vec<Vector3<T>> {
    impl_parallel!(global_dipole_B, 60, points, position, orientation, moment)
}

#[allow(non_snake_case)]
pub fn sum_multiple_dipole_B<T: RealField + num_traits::Float + Copy>(
    points: &[Point3<T>],
    positions: &[Point3<T>],
    orientations: &[UnitQuaternion<T>],
    moments: &[Vector3<T>],
) -> Vec<Vector3<T>> {
    impl_parallel_sum!(
        points,
        [positions, orientations, moments],
        |pos, orien, m| dipole_B(points, pos, orien, m)
    )
}
