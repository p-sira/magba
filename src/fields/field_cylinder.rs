/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Analytical B-field computation for cylindrical magnets.
//!
//! Based on Derby & Olbert (2010), Caciagli et al. (2018), and MagpyLib.

use std::iter::Sum;

use ellip::bulirsch::BulirschConst;
use ellip::{cel, ellipe, ellipk};
use nalgebra::{Point3, RealField, UnitQuaternion, Vector3};
use numeric_literals::replace_float_literals;

use crate::crate_util::{impl_parallel, impl_parallel_sum};
use crate::geometry::{cart2cyl, compute_in_local, vec_cyl2cart};
use crate::Float;
use num_traits::Float as NumFloat;

/// Compute B-field of a cylindrical magnet with unit axial (z-axis) polarization
/// at point (r, z) in cylindrical CS.
///
/// # Arguments
/// - `r`, `z`: Observer positions in cylindrical CS, normalized by radius
/// - `z0`: Half the height over radius
///
/// # Returns
/// - B-field vector (T) at point (r, z)
///
/// # References
/// - Derby, Norman, and Stanislaw Olbert. “Cylindrical Magnets and Ideal Solenoids.” American Journal of Physics 78, no. 3 (March 1, 2010): 229–35. <https://doi.org/10.1119/1.3256157>.
/// - Ortner, Michael, and Lucas Gabriel Coliado Bandeira. “Magpylib: A Free Python Package for Magnetic Field Computation.” SoftwareX 11 (January 1, 2020): 100466. <https://doi.org/10.1016/j.softx.2020.100466>.
#[allow(non_snake_case)]
#[inline]
#[replace_float_literals(T::from_f64(literal).unwrap())]
pub fn unit_axial_cylinder_B_cyl<T: Float + Copy>(r: T, z: T, z0: T) -> Vector3<T> {
    let (zp, zm) = (z + z0, z - z0);
    let (rp, rm) = (1.0 + r, 1.0 - r);

    let (zp2, zm2) = (zp * zp, zm * zm);
    let (rp2, rm2) = (rp * rp, rm * rm);

    let sq0 = NumFloat::sqrt(zm2 + rp2);
    let sq1 = NumFloat::sqrt(zp2 + rp2);

    let kp = NumFloat::sqrt((zp2 + rm2) / (zp2 + rp2));
    let km = NumFloat::sqrt((zm2 + rm2) / (zm2 + rp2));

    let gamma = rm / rp;
    let gamma2 = gamma * gamma;

    let br =
        (cel(kp, 1.0, 1.0, -1.0).unwrap() / sq1 - cel(km, 1.0, 1.0, -1.0).unwrap() / sq0) / T::pi();
    let bz = (zp * cel(kp, gamma2, 1.0, gamma).unwrap() / sq1
        - zm * cel(km, gamma2, 1.0, gamma).unwrap() / sq0)
        / (rp * T::pi());
    // bphi = 0
    Vector3::new(br, 0.0, bz)
}

/// Compute B-field of a cylindrical magnet with unit diametrial (r-axis) polarization
/// at point (r, phi, z) in cylindrical CS.
///
/// # Arguments
/// - `r`, `phi`, `z`: Observer positions in cylindrical CS, normalized by radius
/// - `z0`: Half the height over radius
///
/// # Returns
/// - B-field vector (T) at point (r, phi, z)
///
/// # References
/// - Caciagli, Alessio, Roel J. Baars, Albert P. Philipse, and Bonny W. M. Kuipers. “Exact Expression for the Magnetic Field of a Finite Cylinder with Arbitrary Uniform Magnetization.” Journal of Magnetism and Magnetic Materials 456 (June 15, 2018): 423–32. <https://doi.org/10.1016/j.jmmm.2018.02.003>.
/// - Ortner, Michael, and Lucas Gabriel Coliado Bandeira. “Magpylib: A Free Python Package for Magnetic Field Computation.” SoftwareX 11 (January 1, 2020): 100466. <https://doi.org/10.1016/j.softx.2020.100466>.
#[allow(non_snake_case)]
#[inline]
#[replace_float_literals(T::from_f64(literal).unwrap())]
pub fn unit_diametric_cylinder_B_cyl<T>(r: T, phi: T, z: T, z0: T) -> Vector3<T>
where
    T: RealField + Copy + Float + BulirschConst,
{
    let (zp, zm) = (z + z0, z - z0);
    let (zp2, zm2) = (zp * zp, zm * zm);
    let r2 = r * r;

    if r < 5e-2 {
        // Taylor series approximation for small r
        let (zp4, zm4) = (zp2 * zp2, zm2 * zm2);
        let (zpp, zmm) = (zp2 + 1.0, zm2 + 1.0);
        let (zpp2, zmm2) = (zpp * zpp, zmm * zmm);
        let (zpp3, zmm3) = (zpp2 * zpp, zmm2 * zmm);
        let (zpp4, zmm4) = (zpp3 * zpp, zmm3 * zmm);
        let (zpp5, zmm5) = (zpp4 * zpp, zmm4 * zmm);
        let (sqrt_p, sqrt_m) = (NumFloat::sqrt(zpp), NumFloat::sqrt(zmm));
        let (frac1, frac2) = (zp / sqrt_p, zm / sqrt_m);

        let r3 = r2 * r;
        let r4 = r3 * r;
        let r5 = r4 * r;

        let term1 = frac1 - frac2;
        let term2 = (frac1 / zpp2 - frac2 / zmm2) * r2 / 8.0;
        let term3 =
            ((3.0 - 4.0 * zp2) * frac1 / zpp4 - (3.0 - 4.0 * zm2) * frac2 / zmm4) / 64.0 * r4;

        let br = -NumFloat::cos(phi) / 4.0 * (term1 + 9.0 * term2 + 25.0 * term3);
        let bphi = NumFloat::sin(phi) / 4.0 * (term1 + 3.0 * term2 + 5.0 * term3);
        let bz = -NumFloat::cos(phi) / 4.0
            * (r * (1.0 / zpp / sqrt_p - 1.0 / zmm / sqrt_m)
                + 3.0 / 8.0
                    * r3
                    * ((1.0 - 4.0 * zp2) / zpp3 / sqrt_p - (1.0 - 4.0 * zm2) / zmm3 / sqrt_m)
                + 15.0 / 64.0
                    * r5
                    * ((1.0 - 12.0 * zp2 + 8.0 * zp4) / zpp5 / sqrt_p
                        - (1.0 - 12.0 * zm2 + 8.0 * zm4) / zmm5 / sqrt_m));
        return Vector3::new(br, bphi, bz);
    }

    // General case
    let (rp, rm) = (r + 1.0, r - 1.0);
    let (rp2, rm2) = (rp * rp, rm * rm);

    let (ap2, am2) = (zp2 + rm2, zm2 + rm2);
    let (ap, am) = (NumFloat::sqrt(ap2), NumFloat::sqrt(am2));

    let (argp, argm) = (-4.0 * r / ap2, -4.0 * r / am2);

    // Branch special case r=r0
    let (argc, one_over_rm) = if rm == 0.0 {
        (1e16, 0.0)
    } else {
        (-4.0 * r / rm2, 1.0 / rm)
    };

    // Compute elliptics
    let (ellk_p, ellk_m) = (ellipk(argp).unwrap(), ellipk(argm).unwrap());
    let (elle_p, elle_m) = (ellipe(argp).unwrap(), ellipe(argm).unwrap());
    let (ellpi_p, ellpi_m) = (
        cel(NumFloat::sqrt(1.0 - argp), 1.0 - argc, 1.0, 1.0).unwrap(),
        cel(NumFloat::sqrt(1.0 - argm), 1.0 - argc, 1.0, 1.0).unwrap(),
    );

    // Compute the fields
    let br = -NumFloat::cos(phi) / (4.0 * T::pi() * r2)
        * (-zm * am * elle_m + zp * ap * elle_p + zm / am * (2.0 + zm2) * ellk_m
            - zp / ap * (2.0 + zp2) * ellk_p
            + (zm / am * ellpi_m - zp / ap * ellpi_p) * rp * (r2 + 1.0) * one_over_rm);

    let bphi = NumFloat::sin(phi) / (4.0 * T::pi() * r2)
        * (zm * am * elle_m - zp * ap * elle_p - zm / am * (2.0 + zm2 + 2.0 * r2) * ellk_m
            + zp / ap * (2.0 + zp2 + 2.0 * r2) * ellk_p
            + zm / am * rp2 * ellpi_m
            - zp / ap * rp2 * ellpi_p);

    let bz = -NumFloat::cos(phi) / (2.0 * T::pi() * r)
        * (am * elle_m - ap * elle_p - (1.0 + zm2 + r2) / am * ellk_m
            + (1.0 + zp2 + r2) / ap * ellk_p);

    Vector3::new(br, bphi, bz)
}

/// Compute B-field of a cylindrical magnet at point (r, phi, z) in cylindrical CS.
///
/// # Arguments
/// - `r`, `phi`, `z`: Observer positions in cylindrical CS (m, rad, m)
/// - `radius`: Cylinder radius (m)
/// - `height`: Cylinder height (m)
/// - `pol_r`: Radial polarization (T)
/// - `pol_z`: Axial polarization (T)
///
/// # Returns
/// - B-field vector (T) at point (r, phi, z)
///
/// # Notes
/// - Zero vector is returned if the point is close to the cylindrical magnet's edge (rim).
///
/// # References
/// - Ortner, Michael, and Lucas Gabriel Coliado Bandeira. “Magpylib: A Free Python Package for Magnetic Field Computation.” SoftwareX 11 (January 1, 2020): 100466. <https://doi.org/10.1016/j.softx.2020.100466>.
#[allow(non_snake_case)]
#[inline]
#[replace_float_literals(T::from_f64(literal).unwrap())]
pub fn cylinder_B_cyl<T: RealField + Copy + Float + BulirschConst>(
    r: T,
    phi: T,
    z: T,
    radius: T,
    height: T,
    pol_r: T,
    pol_z: T,
) -> Vector3<T> {
    // Scale invariance
    let r = r / radius;
    let z = z / radius;
    let z0 = (height / 2.0) / radius;

    // Check if point is on Cylinder edge
    let is_close = |a, b, rtol| NumFloat::abs(a - b) < rtol * NumFloat::abs(b);
    if is_close(r, 1.0, 1e-15) && is_close(NumFloat::abs(z), z0, 1e-15) {
        return Vector3::zeros();
    }

    // M = Mz + Mr (Caciagli et al., 2018)
    let mut b = Vector3::zeros();
    if pol_z != 0.0 {
        let b_axial_cyl = unit_axial_cylinder_B_cyl(r, z, z0) * pol_z;
        b += b_axial_cyl;
    }

    if pol_r != 0.0 {
        let b_diametric_cyl = unit_diametric_cylinder_B_cyl(r, phi, z, z0) * pol_r;
        b += b_diametric_cyl;
    }

    b
}

/// Compute B-field at point (x, y, z) of a cylindrical magnet in local frame.
///
/// # Arguments
/// - `point`: Observer position in local frame (m)
/// - `polarization`: Polarization vector (T)
/// - `radius`: Cylinder radius (m)
/// - `height`: Cylinder height (m)
///
/// # Returns
/// - B-field vector at the observer (T)
///
/// # References
/// - Ortner, Michael, and Lucas Gabriel Coliado Bandeira. “Magpylib: A Free Python Package for Magnetic Field Computation.” SoftwareX 11 (January 1, 2020): 100466. <https://doi.org/10.1016/j.softx.2020.100466>.
#[allow(non_snake_case)]
#[inline]
pub fn local_cylinder_B<T: RealField + Copy + Float + BulirschConst>(
    point: &Point3<T>,
    polarization: &Vector3<T>,
    radius: T,
    height: T,
) -> Vector3<T> {
    let (r, phi) = cart2cyl(point.x, point.y);
    let (pol_r, theta) = cart2cyl(polarization.x, polarization.y);

    let b_cyl = cylinder_B_cyl(
        r,
        phi - theta,
        point.z,
        radius,
        height,
        pol_r,
        polarization.z,
    );

    let (bx, by) = vec_cyl2cart(b_cyl.x, b_cyl.y, phi);
    // Check if point is in the magnet
    if r <= radius && NumFloat::abs(point.z) <= height / T::from(2.0).unwrap() {
        return Vector3::new(bx + polarization.x, by + polarization.y, b_cyl.z);
    }

    Vector3::new(bx, by, b_cyl.z)
}

/// Compute B-field at point (x, y, z) of a cylindrical magnet.
///
/// # Arguments
/// - `point`: Observer position in local frame (m)
/// - `position`: Magnet position (m)
/// - `orientation`: Magnet orientation in unit quaternion
/// - `polarization`: Polarization vector (T)
/// - `radius`: Cylinder radius (m)
/// - `height`: Cylinder height (m)
///
/// # Returns
/// - B-field vector at the observer (T)
///
/// # References
/// - Ortner, Michael, and Lucas Gabriel Coliado Bandeira. “Magpylib: A Free Python Package for Magnetic Field Computation.” SoftwareX 11 (January 1, 2020): 100466. <https://doi.org/10.1016/j.softx.2020.100466>.
#[allow(non_snake_case)]
pub fn global_cylinder_B<T: RealField + Copy + Float + BulirschConst>(
    point: &Point3<T>,
    position: &Point3<T>,
    orientation: &UnitQuaternion<T>,
    polarization: &Vector3<T>,
    radius: T,
    height: T,
) -> Vector3<T> {
    compute_in_local!(
        local_cylinder_B,
        &point,
        &position,
        &orientation,
        (&polarization, radius, height),
    )
}

/// Compute B-field at points in global frame for a single cylindrical magnet.
///
/// # Arguments
/// - `points`: Observer positions in global frame (m)
/// - `position`: Magnet position (m)
/// - `orientation`: Magnet orientation as unit quaternion
/// - `polarization`: Polarization vector (T)
/// - `radius`: Cylinder radius (m)
/// - `height`: Cylinder height (m)
///
/// # Returns
/// - B-field vectors at each observer (T)
#[allow(non_snake_case)]
pub fn cylinder_B<T: RealField + Copy + Float + BulirschConst>(
    points: &[Point3<T>],
    position: &Point3<T>,
    orientation: &UnitQuaternion<T>,
    polarization: &Vector3<T>,
    radius: T,
    height: T,
) -> Vec<Vector3<T>> {
    impl_parallel!(
        global_cylinder_B,
        60,
        points,
        position,
        orientation,
        polarization,
        radius,
        height,
    )
}

/// Compute net B-field at each given point in global frame for multiple cylindrical magnets.
///
/// # Arguments
/// - `points`: Observer positions in global frame (m)
/// - `positions`: Magnet positions (m)
/// - `orientations`: Magnet orientations as unit quaternions
/// - `polarizations`: Polarization vectors (T)
/// - `radii`: Cylinder radii (m)
/// - `heights`: Cylinder heights (m)
///
/// # Returns
/// - Net B-field vectors at each observer (T)
#[allow(non_snake_case)]
pub fn sum_multiple_cylinder_B<T: RealField + Copy + Float + BulirschConst + Sum>(
    points: &[Point3<T>],
    positions: &[Point3<T>],
    orientations: &[UnitQuaternion<T>],
    polarizations: &[Vector3<T>],
    radii: &[T],
    heights: &[T],
) -> Vec<Vector3<T>> {
    impl_parallel_sum!(
        points,
        [positions, orientations, polarizations, radii, heights],
        |pos, orien, pol, r, h| cylinder_B(points, pos, orien, pol, *r, *h)
    )
}
