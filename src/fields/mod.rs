/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Analytical magnetic field computation for various magnetic sources.

pub mod field_cylinder;
pub use field_cylinder::{cyl_B, sum_multiple_cyl_B};

pub mod conversion;
