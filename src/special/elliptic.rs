/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use std::f64::consts::FRAC_PI_2;

use russell_lab::math::{elliptic_e, elliptic_f};

/// Compute elliptic integral of the second kind for m≠0
/// Implementation based on SciPy
pub fn ellipe(m: f64) -> Result<f64, &'static str> {
    let mut m = m;
    let mut k = 1.0;
    loop {
        if m >= 0.0 {
            return Ok(k * elliptic_e(FRAC_PI_2, m)?);
        }
        k *= (1.0 - m).sqrt();
        m = m / (m - 1.0);
    }
}

/// Compute elliptic integral of the first kind for m<0
/// Translated from SciPy implementation
/// Original authors:
///  - Copyright 1984, 1987 by Stephen L. Moshier
///  - Copyright 2014, Eric W. Moore
#[inline]
fn ellipk_neg_m(phi: f64, m: f64) -> f64 {
    let mpp = (m * phi) * phi;

    if -mpp < 1e-6 && phi < -m {
        return phi + (-mpp * phi * phi / 30.0 + 3.0 * mpp * mpp / 40.0 + mpp / 6.0) * phi;
    }

    if -mpp > 4e7 {
        let sm = (-m).sqrt();
        let sp = phi.sin();
        let cp = phi.cos();

        let a = (4.0 * sp * sm / (1.0 + cp)).ln();
        let b = -(1.0 + cp / (sp * sp) - a) / (4.0 * m);
        return (a + b) / sm;
    }

    let (scale, x, y, z) = if phi > 1e-153 && m > -1e305 {
        let s = phi.sin();
        let phi_tan = phi.tan();
        let csc2 = 1.0 / (s * s);
        (1.0, 1.0 / (phi_tan * phi_tan), csc2 - m, csc2)
    } else {
        (phi, 1.0, 1.0 - m * phi * phi, 1.0)
    };

    if x == y && x == z {
        return scale / x.sqrt();
    }

    let a0 = (x + y + z) / 3.0;
    let mut a = a0;
    let mut x1 = x;
    let mut y1 = y;
    let mut z1 = z;
    let mut q = 400.0 * f64::max((a0 - x).abs(), f64::max((a0 - y).abs(), (a0 - z).abs()));
    let mut n = 0;

    while q > a.abs() && n <= 100 {
        let sx = x1.sqrt();
        let sy = y1.sqrt();
        let sz = z1.sqrt();
        let lam = sx * sy + sx * sz + sy * sz;
        x1 = (x1 + lam) / 4.0;
        y1 = (y1 + lam) / 4.0;
        z1 = (z1 + lam) / 4.0;
        a = (x1 + y1 + z1) / 3.0;
        n += 1;
        q /= 4.0;
    }

    let two_to_2n = (1 << (2 * n)) as f64;
    let x = (a0 - x) / a / two_to_2n;
    let y = (a0 - y) / a / two_to_2n;
    let z = -(x + y);

    let e2 = x * y - z * z;
    let e3 = x * y * z;

    scale * (1.0 - e2 / 10.0 + e3 / 14.0 + e2 * e2 / 24.0 - 3.0 * e2 * e3 / 44.0) / a.sqrt()
}

pub fn ellipk(m: f64) -> Result<f64, &'static str> {
    if m < 0.0 {
        Ok(ellipk_neg_m(FRAC_PI_2, m))
    } else {
        elliptic_f(FRAC_PI_2, m)
    }
}
