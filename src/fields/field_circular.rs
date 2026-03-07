/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Analytical B-field computation for a circular current loop.

use nalgebra::{Point3, UnitQuaternion, Vector3};
use num_traits::Float as NumFloat;
use numeric_literals::replace_float_literals;

use crate::{
    base::{
        Float,
        coordinate::{cart2cyl, compute_in_local, vec_cyl2cart},
    },
    crate_utils::{impl_parallel, impl_parallel_sum},
};

const MAX_ITER: usize = 10;

/// Computes the Iterative part of Bulirsch cel algorithm.
///
/// Implementation based on "Numerically stable and computationally efficient
/// expression for the magnetic field of a current loop.", M. Ortner et al.,
/// Magnetism 2023, 3(1), 11-31.
///
/// # Arguments
///
/// - `qc`, `p`, `g`, `cc`, `ss`, `em`, `kk`: Parameters of the cel integral
///
/// # Returns
///
/// - Result of the iterative part of the cel integral
///
/// # References
///
/// - Ortner, Michael, and Lucas Gabriel Coliado Bandeira. “Magpylib: A Free Python Package for Magnetic Field Computation.” SoftwareX 11 (January 1, 2020): 100466. <https://doi.org/10.1016/j.softx.2020.100466>.
/// - Ortner, Michael, Peter Leitner, and Franz Slanovc. “Numerically Stable and Computationally Efficient Expression for the Magnetic Field of a Current Loop.” Magnetism 3, no. 1 (2023): 11-31. <https://doi.org/10.3390/magnetism3010002>.
#[replace_float_literals(T::from_f64(literal).unwrap())]
fn cel_iter<T: Float>(
    mut qc: T,
    mut p: T,
    mut g: T,
    mut cc: T,
    mut ss: T,
    mut em: T,
    mut kk: T,
) -> T {
    let errtol = 1e-8;
    let half_pi = T::pi() / 2.0;

    for _ in 0..MAX_ITER {
        if NumFloat::abs(g - qc) < qc * errtol {
            break;
        }

        qc = NumFloat::sqrt(kk) * 2.0;
        kk = qc * em;
        let f = cc;
        cc = cc + ss / p;
        g = kk / p;
        ss = 2.0 * (ss + f * g);
        p = p + g;
        g = em;
        em = em + qc;
    }
    half_pi * (ss + cc * em) / (em * (em + p))
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

    let (mut r, phi) = cart2cyl(point.x, point.y);
    let mut z = point.z;

    // Relative coordinates
    r = r / r0;
    z = z / r0;

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

    // cel* part
    let mut cc = k2 * 4.0 * z / x0;
    let mut ss = 2.0 * cc * q / p;
    let hr = pf * cel_iter(q, p, 1.0, cc, ss, p, q);

    // cel** part
    let k4 = k2 * k2;
    cc = k4 - (q2 + 1.0) * (4.0 / x0);
    ss = 2.0 * q * (k4 / p - (4.0 / x0) * p);
    let hz = -pf * cel_iter(q, p, 1.0, cc, ss, p, q);

    let (bx, by) = vec_cyl2cart(hr, 0.0, phi);
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
/// assert_relative_eq!(b_field, expected, epsilon = 1e-12);
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
#[cfg(feature = "alloc")]
#[allow(non_snake_case)]
pub fn circular_B_batch<T: Float>(
    points: &[Point3<T>],
    position: Point3<T>,
    orientation: UnitQuaternion<T>,
    diameter: T,
    current: T,
    out: &mut [Vector3<T>],
) {
    impl_parallel!(
        circular_B,
        rayon_threshold: 60,
        input: points,
        output: out,
        args: [position, orientation, diameter, current]
    )
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
