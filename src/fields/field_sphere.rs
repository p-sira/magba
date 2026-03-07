/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Analytical B-field computation for homogeneously magnetized sphere.

use nalgebra::{Point3, UnitQuaternion, Vector3};
use numeric_literals::replace_float_literals;

use crate::{
    base::{Float, coordinate::compute_in_local},
    crate_utils::{impl_parallel, impl_parallel_sum},
};

/// Computes B-field of a homogeneously magnetized sphere at point (x, y, z) in local frame.
///
/// Outside the sphere, the field corresponds to a dipole field. Inside, it is
/// 2/3 of the polarization.
///
/// # Arguments
///
/// - `point`: Observer position (m)
/// - `polarization`: Polarization vector (T)
/// - `diameter`: Sphere diameter (m)
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
pub fn local_sphere_B<T: Float>(
    point: Point3<T>,
    polarization: Vector3<T>,
    diameter: T,
) -> Vector3<T> {
    let p = Vector3::from(point.coords);
    let r = p.norm();
    let r_sphere = num_traits::Float::abs(diameter) / 2.0;

    if r > r_sphere {
        // Outside: Dipole field
        let r2 = r * r;
        let r5 = r2 * r2 * r;
        let r_sphere3 = r_sphere * r_sphere * r_sphere;

        (p * (3.0 * polarization.dot(&p)) - polarization * r2) * (r_sphere3 / (3.0 * r5))
    } else {
        // Inside: Constant field
        polarization * (2.0 / 3.0)
    }
}

/// Computes B-field of a homogeneously magnetized sphere at point (x, y, z).
///
/// # Arguments
///
/// - `point`: Observer position (m)
/// - `position`: Sphere center (m)
/// - `orientation`: Sphere orientation in unit quaternion
/// - `polarization`: Polarization vector (T)
/// - `diameter`: Sphere diameter (m)
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
pub fn sphere_B<T: Float>(
    point: Point3<T>,
    position: Point3<T>,
    orientation: UnitQuaternion<T>,
    polarization: Vector3<T>,
    diameter: T,
) -> Vector3<T> {
    compute_in_local!(
        local_sphere_B,
        point,
        position,
        orientation,
        (polarization, diameter),
    )
}

/// Computes B-field at points in global frame for a homogeneously magnetized sphere.
///
/// # Arguments
///
/// - `points`: Observer positions (m)
/// - `position`: Sphere center (m)
/// - `orientation`: Sphere orientation in unit quaternion
/// - `polarization`: Polarization vector (T)
/// - `diameter`: Sphere diameter (m)
/// - `out`: Mutable slice to store the B-field vectors at each observer (T)
///
/// # References
///
/// - Ortner, Michael, and Lucas Gabriel Coliado Bandeira. “Magpylib: A Free Python Package for Magnetic Field Computation.” SoftwareX 11 (January 1, 2020): 100466. <https://doi.org/10.1016/j.softx.2020.100466>.
#[allow(non_snake_case)]
pub fn sphere_B_batch<T: Float>(
    points: &[Point3<T>],
    position: Point3<T>,
    orientation: UnitQuaternion<T>,
    polarization: Vector3<T>,
    diameter: T,
    out: &mut [Vector3<T>],
) {
    impl_parallel!(
        sphere_B,
        rayon_threshold: 60,
        input: points,
        output: out,
        args: [position, orientation, polarization, diameter]
    )
}

/// Computes B-field at each given points in global frame for multiple homogeneously magnetized spheres.
///
/// # Arguments
///
/// - `points`: Observer positions (m)
/// - `positions`: Sphere centers (m)
/// - `orientations`: Sphere orientations in unit quaternion
/// - `polarizations`: Polarization vectors (T)
/// - `diameters`: Sphere diameters (m)
/// - `out`: Mutable slice to store the net B-field vectors at each observer (T)
///
/// # References
///
/// - Ortner, Michael, and Lucas Gabriel Coliado Bandeira. “Magpylib: A Free Python Package for Magnetic Field Computation.” SoftwareX 11 (January 1, 2020): 100466. <https://doi.org/10.1016/j.softx.2020.100466>.
#[allow(non_snake_case)]
pub fn sum_multiple_sphere_B<T: Float>(
    points: &[Point3<T>],
    positions: &[Point3<T>],
    orientations: &[UnitQuaternion<T>],
    polarizations: &[Vector3<T>],
    diameters: &[T],
    out: &mut [Vector3<T>],
) {
    impl_parallel_sum!(
        out,
        points,
        60,
        [positions, orientations, polarizations, diameters],
        |pos, p, o, pol, d| sphere_B(*pos, *p, *o, *pol, *d)
    )
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;
    use nalgebra::{point, vector};

    use super::*;

    #[test]
    fn test_local_sphere_b_outside() {
        let b = local_sphere_B(point![5.0, 6.0, 7.0], vector![0.45, 0.3, 0.15], 1.0);
        let expected = vector![
            8.864838123151168e-06,
            1.9305647468195875e-05,
            2.974645681324058e-05
        ];
        assert_relative_eq!(b, expected, epsilon = 1e-15);
    }

    #[test]
    fn test_local_sphere_b_inside() {
        let b = local_sphere_B(point![0.0, 0.0, 0.0], vector![1.0, 2.0, 3.0], 1.0);
        let expected = vector![1.0, 2.0, 3.0] * (2.0 / 3.0);
        assert_relative_eq!(b, expected, epsilon = 1e-15);
    }
}
