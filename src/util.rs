/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Utilities for Magba

pub fn is_close(a: f64, b: f64, rtol: f64) -> bool {
    (a - b).abs() <= rtol * b.abs()
}
