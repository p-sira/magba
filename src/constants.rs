/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Magnetic constants and Magba computation constants, such as tolerances and thresholds.

use std::f64::consts::PI;

pub const ERRTOL: f64 = 0.000001;
pub const MU0: f64 = 4e-7 * PI;