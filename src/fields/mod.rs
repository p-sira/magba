/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! # Fields
//!
//! Analytical computation of magnetic fields for various source geometries.
//!
//! - [`field_cylinder`]: Analytical B-field for cylindrical magnets.
//! - [`conversion`]: Utilities for converting between B-field, H-field, magnetization, and polarization.

pub mod field_cylinder;
pub use field_cylinder::{cyl_B, sum_multiple_cyl_B};

pub mod conversion;
