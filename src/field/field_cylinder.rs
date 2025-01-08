/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use std::f64::consts::{FRAC_PI_2, PI};
use std::f64::NAN;
use std::panic;

use nalgebra::{Point3, Vector3};
use russell_lab::math::{elliptic_e, elliptic_f, elliptic_pi};
use russell_lab::StrError;

use crate::geometry::{cart2cyl, cyl2cart};
use crate::special::cel;

/// Compute the magnetic field B at (r, z) in Cyl CS
/// for the axially polarized (z-axis) magnet of dimension (radius, height)
///
/// # Examples
/// ```
/// use magba::field;
///
/// let r = 0.0;
/// let z = 16e-3;
/// let radius = 5e-3;
/// let height = 20e-3;
/// let pol_z = 8e5 * magba::constants::MU0;
///
/// let b = field::axial_cyl_b_cyl(r, z, radius, height, pol_z);
/// // Computed using Bz formula Eq. 3.89 in Permanent Magnet and Electromechanical Devices, chp. 3, p. 129 (Furlani, 2001).
/// assert_eq! (b, Ok([0.0, 0.0, 0.10746014580764005]));
/// ```
pub fn unit_axial_cyl_b_cyl(r: f64, z: f64, z0: f64) -> Result<Vector3<f64>, &'static str> {
    let (zp, zm) = (z + z0, z - z0);
    let (rp, rm) = (1.0 + r, 1.0 - r);

    let (zp2, zm2) = (zp * zp, zm * zm);
    let (rp2, rm2) = (rp * rp, rm * rm);

    let sq0 = (zm2 + rp2).sqrt();
    let sq1 = (zp2 + rp2).sqrt();

    let kp = ((zp2 + rm2) / (zp2 + rp2)).sqrt();
    let km = ((zm2 + rm2) / (zm2 + rp2)).sqrt();

    let gamma = rm / rp;
    let gamma2 = gamma * gamma;

    let br = (cel(kp, 1.0, 1.0, -1.0)? - cel(km, 1.0, 1.0, -1.0)?) / (sq0 * PI);
    let bz = (zp * cel(kp, gamma2, 1.0, gamma)? / sq1 - zm * cel(km, gamma2, 1.0, gamma)? / sq0)
        / (rp * PI);
    // bphi = 0
    Ok(Vector3::new(br, 0.0, bz))
}

/// Calculate complete elliptic integral of the first, second, and third kinds
fn calculate_elliptic(arg_kp: f64, arg_km: f64, gam2: f64) -> Result<[f64; 6], &'static str> {
    let result = panic::catch_unwind(|| {
        Ok::<[f64; 6], StrError>([
            elliptic_f(FRAC_PI_2, arg_kp)?,
            elliptic_f(FRAC_PI_2, arg_km)?,
            elliptic_e(FRAC_PI_2, arg_kp)?,
            elliptic_e(FRAC_PI_2, arg_km)?,
            elliptic_pi(1.0 - gam2, FRAC_PI_2, arg_kp)?,
            elliptic_pi(1.0 - gam2, FRAC_PI_2, arg_km)?,
        ])
    });

    match result {
        Ok(Ok(values)) => Ok(values),
        Ok(Err(_)) | Err(_) => Ok([f64::NAN; 6]),
    }
}

pub fn diametric_cyl_b_cyl(cyl_point: [f64; 3], radius: f64, height: f64, pol_r: f64) -> [f64; 3] {
    if pol_r == 0.0 {
        return [0.0; 3];
    }
    // Extract points in cyl CS
    let [r, phi, z] = cyl_point;

    // Define shorthand notations
    let l = height / 2.0;
    let rp = r + radius;
    let rm = r - radius;
    let rp2 = rp * rp;
    let rm2 = rm * rm;

    let epsp = z + l;
    let epsm = z - l;
    let epsp2 = epsp * epsp;
    let epsm2 = epsm * epsm;
    let sqrtp = (epsp2 + rp2).sqrt();
    let sqrtm = (epsm2 + rp2).sqrt();

    let alpp = 1.0 / sqrtp;
    let alpm = 1.0 / sqrtm;

    let betp = epsp * alpp;
    let betm = epsm * alpm;

    let gam = rm / rp;
    let gam2 = gam * gam;

    let kp2 = (epsp2 + rm2) / (epsm2 + rp2);
    let km2 = (epsm2 + rm2) / (epsm2 + rp2);

    // Define complete elliptic integrals
    let arg_kp = 1.0 - kp2;
    let arg_km = 1.0 - km2;

    let (celk_p, celk_m, cele_p, cele_m, celp_p, celp_m) =
        match calculate_elliptic(arg_kp, arg_km, gam2) {
            Ok([celk_p, celk_m, cele_p, cele_m, celp_p, celp_m]) => {
                (celk_p, celk_m, cele_p, cele_m, celp_p, celp_m)
            }
            Err(_) => return [NAN; 3],
        };

    // Define auxilliary functions
    let inv_arg_kp = 1.0 / arg_kp;
    let inv_arg_km = 1.0 / arg_km;
    // May benefit from using special::cel()
    let p1p = celk_p - 2.0 * inv_arg_kp * (celk_p - cele_p);
    let p1m = celk_m - 2.0 * inv_arg_km * (celk_m - cele_m);

    let gam2_term = gam2 / (1.0 - gam2);
    let p3p = inv_arg_kp * (celk_p - cele_p) - gam2_term * (celp_p - celk_p);
    let p3m = inv_arg_km * (celk_m - cele_m) - gam2_term * (celp_m - celk_m);

    let p4p = gam2_term * (celp_p - celk_p) + gam2_term * (gam2 * celp_p - celk_p) - p1p;
    let p4m = gam2_term * (celp_m - celk_m) + gam2_term * (gam2 * celp_m - celk_m) - p1m;

    // Calculate the B field
    let br = pol_r * radius * phi.cos() / (2.0 * PI * r) * (betp * p4p - betm * p4m);
    let bphi = pol_r * radius * phi.sin() / (PI * r) * (betp * p3p - betm * p3m);
    let bz = pol_r * radius * phi.cos() / PI * (alpp * p1p - alpm * p1m);

    [br, bphi, bz]
}

pub fn diametric_cyl_b(point: Point3<f64>, radius: f64, height: f64, pol_r: f64) -> Vector3<f64> {
    let (x, y, z) = (point.x, point.y, point.z);
    let (r, phi) = cart2cyl(x, y);
    let cyl_point = [r, phi, z];
    let [br, bphi, bz] = diametric_cyl_b_cyl(cyl_point, radius, height, pol_r);
    let (bx, by) = cyl2cart(br, bphi);

    Vector3::new(bx, by, bz)
}

// M = Mz + Mr (Caciagli et al., 2018)
pub fn cyl_b_cyl(
    cyl_point: [f64; 3],
    radius: f64,
    height: f64,
    pol_r: f64,
    pol_z: f64,
) -> Result<[f64; 3], &'static str> {
    let z0 = (height / 2.0) / radius;

    let b_axial_cyl = pol_z * unit_axial_cyl_b_cyl(cyl_point[0], cyl_point[2], z0).expect("fn cyl_b_cyl: cannot compute axial B.");
    let b_diametric_cyl = diametric_cyl_b_cyl(cyl_point, radius, height, pol_r);
    let (br, bphi, bz) = (
        b_axial_cyl[0] + b_diametric_cyl[0],
        b_axial_cyl[1] + b_diametric_cyl[1],
        b_axial_cyl[2] + b_diametric_cyl[2],
    );

    Ok([br, bphi, bz])
}

pub fn cyl_b(
    point: Point3<f64>,
    radius: f64,
    height: f64,
    pol: Vector3<f64>,
) -> Result<Vector3<f64>, &'static str> {
    let (x, y, z) = (point.x, point.y, point.z);
    let (r, phi) = cart2cyl(x, y);
    let cyl_point = [r, phi, z];
    let pol_r = (pol.x * pol.x + pol.y * pol.y).sqrt();
    let pol_z = pol.z;

    let [br, bphi, bz] = cyl_b_cyl(cyl_point, radius, height, pol_r, pol_z)?;

    let (bx, by) = cyl2cart(br, bphi);
    Ok(Vector3::new(bx, by, bz))
}
