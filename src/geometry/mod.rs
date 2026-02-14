/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Geometric utilities for 3D transformations and coordinate conversions.
//!
mod coordinate;
#[cfg(feature = "unstable")]
pub use coordinate::*;
#[cfg(not(feature = "unstable"))]
pub(crate) use coordinate::*;

pub(crate) mod transform;
pub use transform::*;
