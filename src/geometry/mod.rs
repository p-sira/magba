/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Geometric utilities for 3D transformations and coordinate conversions.
//!
//! - [`Transform`] trait: For objects with position/orientation in 3D, with methods for translation, rotation, and rotation with anchor.
//! - Coordinate conversion utilities: [`cart2cyl`], [`vec_cyl2cart`], [`local_point`], [`global_vector`], etc.

mod coordinate;
pub use coordinate::*;

#[cfg(feature = "transform")]
mod transform;
#[cfg(feature = "transform")]
pub use transform::*;
