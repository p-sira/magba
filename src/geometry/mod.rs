/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Geometry utilities, such as [Transform] and coordinate conversion.

mod coordinate;
pub use coordinate::*;

#[cfg(feature = "transform")]
mod transform;
#[cfg(feature = "transform")]
pub use transform::*;
