/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use std::f64::consts::PI;

use crate::constants::ERRTOL;

/// Compute special case of complete elliptical integral
/// 
/// Reduced operation algorithm modified from Derby & Olbert, 2010.
pub fn cel(kc: f64, p: f64, c: f64, s: f64) -> Result<f64, &'static str> {
    if kc == 0.0 {
        return Err("fn cel: kc cannot be zero.");
    }

    let mut k = kc.abs();
    let mut cc;
    let mut pp;
    let mut ss;

    if p > 0.0 {
        cc = c;
        pp = p.sqrt();
        ss = s / pp;
    } else {
        let f = kc * kc;
        let q = (1.0 - f) * (s - c * p);
        let g = 1.0 - p;
        let h = f - p;
        pp = (h / g).sqrt();
        cc = (c - s) / g;
        ss = -q / (g * g * pp) + cc * pp;
    }

    let mut em = 1.0;
    let mut f = cc;
    cc = cc + ss / pp;
    let mut g = k / pp;
    ss = 2.0 * (ss + f * g);
    pp = g + pp;
    g = em;
    em = k + em;
    let mut kk = k;

    while (g - k).abs() > (g * ERRTOL) {
        k = 2.0 * kk.sqrt();
        kk = k * em;
        f = cc;
        cc = cc + ss / pp;
        g = kk / pp;
        ss = 2.0 * (ss + f * g);
        pp = g + pp;
        g = em;
        em = k + em;
    }

    Ok((PI / 2.0) * (ss + cc * em) / (em * (em + pp)))
}
