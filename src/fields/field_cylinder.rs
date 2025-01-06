/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use std::f64::consts::PI;

use crate::special::cel;
use crate::geometry::coordinate::cyl2cart;
/// Compute the magnetic field B at (r, z) in Cyl CS
/// for the axially polarized (z-axis) magnet of dimension (radius, height)
pub fn axial_cyl_b_cyl(r: f64, z: f64, radius: f64, height: f64, pol_z: f64) -> (f64, f64, f64) {
    let b = height / 2.0;
    let zp = z + b;
    let zm = z - b;
    let zp2 = zp * zp;
    let zm2 = zm * zm;

    let radius_p = radius + r;
    let radius_m = radius - r;
    let radius_p2 = radius_p * radius_p;
    let radius_m2 = radius_m * radius_m;

    let gamma = radius_m / radius_p;
    let gamma2 = gamma * gamma;

    let sqrt_p = (zp2 + radius_p2).sqrt();
    let sqrt_m = (zm2 + radius_p2).sqrt();

    let alpha_p = radius / sqrt_p;
    let alpha_m = radius / sqrt_m;

    let beta_p = zp / sqrt_p;
    let beta_m = zm / sqrt_m;

    let kp = (zp2 + radius_m2).sqrt() / sqrt_p;
    let km = (zm2 + radius_m2).sqrt() / sqrt_m;

    let b0 = pol_z / PI;
    let br = b0 * (alpha_p * cel(kp, 1.0, 1.0, -1.0) - alpha_m * cel(km, 1.0, 1.0, -1.0));
    let bz = b0 * radius / radius_p
        * (beta_p * cel(kp, gamma2, 1.0, gamma) - beta_m * cel(km, gamma2, 1.0, gamma));

    // bphi = 0
    (br, 0.0, bz)
}

/// Compute the magnetic field B at (x, y, z)
pub fn axial_cyl_b(x: f64, y: f64, z: f64, radius: f64, height: f64, pol_z: f64) -> (f64, f64, f64) {
    let r = (x * x + y * y).sqrt();
    let (r_res, phi_res, z_res) = axial_cyl_b_cyl(r, z, radius, height, pol_z);
    let (x_res, y_res) = cyl2cart(r_res, phi_res);
    (x_res, y_res, z_res)
}
