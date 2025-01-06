/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

/// Convert cylindrical coordinates (r, phi) to Cartesian coordinates (x, y)
pub fn cyl2cart(r: f64, phi: f64) -> (f64, f64) {
    let x = r * phi.cos();
    let y = r * phi.sin();
    (x, y)
}

/// Convert Cartesian coordinates (x, y) to cylindrical coordinates (r, phi)
pub fn cart2cyl(x: f64, y: f64) -> (f64, f64) {
    let r = (x * x + y * y).sqrt();
    let phi = y.atan2(x);
    (r, phi)
}
