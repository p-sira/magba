/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Geometric utilities for 3D transformations and coordinate conversions.
//!
mod coordinate;
pub use coordinate::*;

#[cfg(feature = "transform")]
pub(crate) mod transform;
#[cfg(feature = "transform")]
pub use transform::*;
