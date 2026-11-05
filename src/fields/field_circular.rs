/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Analytical B-field computation for a circular current loop.

use nalgebra::{Point3, UnitQuaternion, Vector3};
use num_traits::Float as NumFloat;
use numeric_literals::replace_float_literals;

use crate::{
    base::{Float, coordinate::compute_in_local},
    crate_utils::{impl_parallel, impl_parallel_sum},
};

const MAX_ITER: usize = 10;

/// Computes both Bulirsch cel terms for a circular current in one iteration.
///
/// The radial and axial terms have identical modulus, denominator, and
/// convergence sequence. Only their numerators differ.
///
/// # References
///
/// - Ortner, Michael, and Lucas Gabriel Coliado Bandeira. “Magpylib: A Free Python Package for Magnetic Field Computation.” SoftwareX 11 (January 1, 2020): 100466. <https://doi.org/10.1016/j.softx.2020.100466>.
/// - Ortner, Michael, Peter Leitner, and Franz Slanovc. “Numerically Stable and Computationally Efficient Expression for the Magnetic Field of a Current Loop.” Magnetism 3, no. 1 (2023): 11-31. <https://doi.org/10.3390/magnetism3010002>.
#[replace_float_literals(T::from_f64(literal).unwrap())]
fn cel_iter_pair<T: Float>(
    mut qc: T,
    mut p: T,
    mut cc_r: T,
    mut ss_r: T,
    mut cc_z: T,
    mut ss_z: T,
) -> (T, T) {
    let errtol = 1e-8;
    let half_pi = T::pi() / 2.0;
    let mut g = 1.0;
    let mut em = p;
    let mut kk = qc;

    for _ in 0..MAX_ITER {
        if NumFloat::abs(g - qc) < qc * errtol {
            break;
        }

        qc = NumFloat::sqrt(kk) * 2.0;
        kk = qc * em;
        let f_r = cc_r;
        let f_z = cc_z;
        cc_r += ss_r / p;
        cc_z += ss_z / p;
        g = kk / p;
        ss_r = 2.0 * (ss_r + f_r * g);
        ss_z = 2.0 * (ss_z + f_z * g);
        p += g;
        g = em;
        em += qc;
    }
    let denominator = em * (em + p);
    (
        half_pi * (ss_r + cc_r * em) / denominator,
        half_pi * (ss_z + cc_z * em) / denominator,
    )
}

/// Computes local magnetic field (B) of a circular current loop at point (x, y, z).
///
/// # Arguments
///
/// - `point`: Observer position in local frame (m)
/// - `diameter`: Loop diameter (m)
/// - `current`: Current flowing in the loop (A)
///
/// # Returns
///
/// - B-field vector at the observer (T)
///
/// # References
///
/// - Ortner, Michael, and Lucas Gabriel Coliado Bandeira. “Magpylib: A Free Python Package for Magnetic Field Computation.” SoftwareX 11 (January 1, 2020): 100466. <https://doi.org/10.1016/j.softx.2020.100466>.
/// - Ortner, Michael, Peter Leitner, and Franz Slanovc. “Numerically Stable and Computationally Efficient Expression for the Magnetic Field of a Current Loop.” Magnetism 3, no. 1 (2023): 11-31. <https://doi.org/10.3390/magnetism3010002>.
#[allow(non_snake_case)]
#[replace_float_literals(T::from_f64(literal).unwrap())]
pub fn local_circular_B<T: Float>(point: Point3<T>, diameter: T, current: T) -> Vector3<T> {
    let r0 = NumFloat::abs(diameter) / 2.0;
    if r0 == 0.0 {
        return Vector3::zeros();
    }

    let orig_r = NumFloat::sqrt(point.x * point.x + point.y * point.y);
    let r = orig_r / r0;
    let z = point.z / r0;

    // Special case: at singularity (on the loop)
    if NumFloat::abs(r - 1.0) < 1e-15 && NumFloat::abs(z) < 1e-15 {
        return Vector3::zeros();
    }

    let z2 = z * z;
    let r_plus_1 = r + 1.0;
    let r_minus_1 = r - 1.0;
    let x0 = z2 + r_plus_1 * r_plus_1;
    let k2 = 4.0 * r / x0;
    let q2 = (z2 + r_minus_1 * r_minus_1) / x0;

    let q = NumFloat::sqrt(q2);
    let p = 1.0 + q;
    let pf = current / (4.0 * T::pi() * r0 * NumFloat::sqrt(x0) * q2);

    // Radial cel* and axial cel** use the same Landen sequence.
    let cc_r = k2 * 4.0 * z / x0;
    let ss_r = 2.0 * cc_r * q / p;
    let k4 = k2 * k2;
    let cc_z = k4 - (q2 + 1.0) * (4.0 / x0);
    let ss_z = 2.0 * q * (k4 / p - (4.0 / x0) * p);
    let (cel_r, cel_z) = cel_iter_pair(q, p, cc_r, ss_r, cc_z, ss_z);
    let hr = pf * cel_r;
    let hz = -pf * cel_z;

    let (bx, by) = if orig_r > 0.0 {
        let inv_orig_r = 1.0 / orig_r;
        (hr * point.x * inv_orig_r, hr * point.y * inv_orig_r)
    } else {
        (0.0, 0.0)
    };
    Vector3::new(bx, by, hz) * T::mu0()
}

/// Computes magnetic field (B) of a circular current loop.
///
/// # Arguments
///
/// - `point`: Observer position in global frame (m)
/// - `position`: Loop center (m)
/// - `orientation`: Loop orientation as unit quaternion
/// - `diameter`: Loop diameter (m)
/// - `current`: Current flowing in the loop (A)
///
/// # Returns
///
/// - B-field vector at the observer (T)
///
/// # Examples
///
/// ```
/// # use approx::assert_relative_eq;
/// # use magba::fields::circular_B;
/// # use nalgebra::{point, vector, UnitQuaternion};
/// let b_field = circular_B(
///     point![0.0, 0.0, 0.0],
///     point![0.0, 0.0, 0.0],
///     UnitQuaternion::identity(),
///     2.0,
///     1.0,
/// );
/// let expected = vector![0.0, 0.0, 6.283185307179586e-7];
/// assert_relative_eq!(b_field, expected, epsilon = 1e-16);
/// ```
///
/// # References
///
/// - Ortner, Michael, and Lucas Gabriel Coliado Bandeira. “Magpylib: A Free Python Package for Magnetic Field Computation.” SoftwareX 11 (January 1, 2020): 100466. <https://doi.org/10.1016/j.softx.2020.100466>.
/// - Ortner, Michael, Peter Leitner, and Franz Slanovc. “Numerically Stable and Computationally Efficient Expression for the Magnetic Field of a Current Loop.” Magnetism 3, no. 1 (2023): 11-31. <https://doi.org/10.3390/magnetism3010002>.
#[inline]
#[allow(non_snake_case)]
pub fn circular_B<T: Float>(
    point: Point3<T>,
    position: Point3<T>,
    orientation: UnitQuaternion<T>,
    diameter: T,
    current: T,
) -> Vector3<T> {
    compute_in_local!(
        local_circular_B,
        point,
        position,
        orientation,
        (diameter, current),
    )
}

/// Batch magnetic field (B) of a circular current loop.
///
/// # Arguments
///
/// - `points`: Observer positions in global frame (m)
/// - `position`: Loop center (m)
/// - `orientation`: Loop orientation as unit quaternion
/// - `diameter`: Loop diameter (m)
/// - `current`: Current flowing in the loop (A)
/// - `out`: Mutable slice to store the B-field vectors at each observer (T)
///
/// # References
///
/// - Ortner, Michael, and Lucas Gabriel Coliado Bandeira. “Magpylib: A Free Python Package for Magnetic Field Computation.” SoftwareX 11 (January 1, 2020): 100466. <https://doi.org/10.1016/j.softx.2020.100466>.
/// - Ortner, Michael, Peter Leitner, and Franz Slanovc. “Numerically Stable and Computationally Efficient Expression for the Magnetic Field of a Current Loop.” Magnetism 3, no. 1 (2023): 11-31. <https://doi.org/10.3390/magnetism3010002>.
#[allow(non_snake_case)]
pub fn circular_B_batch<T: Float>(
    points: &[Point3<T>],
    position: Point3<T>,
    orientation: UnitQuaternion<T>,
    diameter: T,
    current: T,
    out: &mut [Vector3<T>],
) {
    let inv_orientation = orientation.inverse();
    impl_parallel!(
        rayon_threshold: 340,
        input: points,
        output: out,
        |p| {
            let local_point = inv_orientation * Point3::from(p.coords - position.coords);
            let local_b = local_circular_B(local_point, diameter, current);
            orientation * local_b
        }
    );
}

/// Computes net B-field at each given point in global frame for multiple circular current loops.
///
/// # Arguments
///
/// - `points`: Observer positions in global frame (m)
/// - `positions`: Loop centers (m)
/// - `orientations`: Loop orientations as unit quaternions
/// - `diameters`: Loop diameters (m)
/// - `currents`: Currents flowing in the loops (A)
/// - `out`: Mutable slice to store the net B-field vectors at each observer (T)
///
/// # References
///
/// - Ortner, Michael, and Lucas Gabriel Coliado Bandeira. “Magpylib: A Free Python Package for Magnetic Field Computation.” SoftwareX 11 (January 1, 2020): 100466. <https://doi.org/10.1016/j.softx.2020.100466>.
/// - Ortner, Michael, Peter Leitner, and Franz Slanovc. “Numerically Stable and Computationally Efficient Expression for the Magnetic Field of a Current Loop.” Magnetism 3, no. 1 (2023): 11-31. <https://doi.org/10.3390/magnetism3010002>.
#[allow(non_snake_case)]
pub fn sum_multiple_circular_B<T: Float>(
    points: &[Point3<T>],
    positions: &[Point3<T>],
    orientations: &[UnitQuaternion<T>],
    diameters: &[T],
    currents: &[T],
    out: &mut [Vector3<T>],
) {
    impl_parallel_sum!(
        out,
        points,
        60,
        [positions, orientations, diameters, currents],
        |pos, p, o, d, c| circular_B(*pos, *p, *o, *d, *c)
    )
}

#[cfg(test)]
mod tests {
    use nalgebra::{point, vector};

    use super::*;

    #[test]
    fn test_circular_field_against_elliptic_integral_formula() {
        // Independent Biot-Savart expression in complete Legendre elliptic
        // integrals. Include points near the wire, where the CEL terms are
        // sensitive to the shared convergence sequence.
        let radius: f64 = 1.0;
        let mu0 = <f64 as Float>::mu0();
        for (r, z) in [(0.5_f64, 1.0_f64), (0.9, 0.002), (1.1, -0.002)] {
            let near = (radius - r).powi(2) + z * z;
            let far = (radius + r).powi(2) + z * z;
            let m = 4.0 * radius * r / far;
            let (k, e) = ellip::ellipke(m).unwrap();
            let common = mu0 / (2.0 * core::f64::consts::PI * far.sqrt());
            let expected = vector![
                common * z / r * ((radius * radius + r * r + z * z) / near * e - k),
                0.0,
                common * (k + (radius * radius - r * r - z * z) / near * e)
            ];
            let actual = local_circular_B(point![r, 0.0, z], 2.0 * radius, 1.0);
            approx::assert_relative_eq!(actual, expected, epsilon = 1e-14, max_relative = 1e-7);
        }

        // Axial field has a closed form and avoids the off-axis 1/r factor.
        let z = 0.8_f64;
        let expected_z = mu0 * radius * radius / (2.0 * (radius * radius + z * z).powf(1.5));
        let actual = local_circular_B(point![0.0, 0.0, z], 2.0 * radius, 1.0);
        approx::assert_relative_eq!(actual.z, expected_z, epsilon = 1e-14);
        assert_eq!(actual.x, 0.0);
        assert_eq!(actual.y, 0.0);
    }

    #[test]
    fn test_sum_multiple_circular_b() {
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
        let diameters = &[1.0, 2.0];
        let currents = &[1.5, 2.5];

        impl_test_sum_multiple!(
            sum_multiple_circular_B,
            1e-15,
            points,
            positions,
            orientations,
            (diameters, currents),
            |p, pos, ori, d, c| circular_B(p, pos, ori, d, c)
        );
    }

    #[test]
    fn test_circular_edge_cases_and_batch() {
        // Zero diameter
        let b_zero = local_circular_B(point![0.0, 0.0, 1.0], 0.0, 1.0);
        assert_eq!(b_zero, Vector3::zeros());

        // On the wire (singularity)
        let b_wire = local_circular_B(point![1.0, 0.0, 0.0], 2.0, 1.0);
        assert_eq!(b_wire, Vector3::zeros());

        // circular_B_batch with <= 350 points to trigger serial threshold (verified against magpylib)
        let small_points = [
            point![0.0, 0.0, 1.0],
            point![0.5, 0.0, 1.0],
            point![0.0, 0.5, 1.0],
        ];
        let mut small_out = vec![Vector3::zeros(); 3];
        circular_B_batch(
            &small_points,
            Point3::origin(),
            UnitQuaternion::identity(),
            2.0,
            1.0,
            &mut small_out,
        );
        let expected = [
            vector![0.0, 0.0, 2.22144146878588e-07],
            vector![7.887867348048176e-08, 0.0, 1.8954556071701232e-07],
            vector![0.0, 7.887867348048176e-08, 1.8954556071701232e-07],
        ];
        for (o, e) in small_out.iter().zip(expected.iter()) {
            approx::assert_relative_eq!(o, e, epsilon = 1e-14, max_relative = 1e-12);
        }

        // circular_B_batch with > 350 points to trigger Rayon threshold
        let points = vec![point![0.0, 0.0, 1.0]; 400];
        let mut out = vec![Vector3::zeros(); 400];
        circular_B_batch(
            &points,
            Point3::origin(),
            UnitQuaternion::identity(),
            2.0,
            1.0,
            &mut out,
        );
        assert_eq!(out.len(), 400);
        assert!(out[0].z != 0.0);
    }
}
