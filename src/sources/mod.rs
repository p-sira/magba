/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Magnetic source traits and structs.
//!
//! Permanent magnets:
//! - [CylinderMagnet]
//! - [CuboidMagnet]
//!
//! Magnetic source collection:
//! - [SourceCollection]
//! - [MultiSourceCollection]

mod cuboid;
mod cylinder;
mod source;

pub use cuboid::*;
pub use cylinder::*;
pub use source::*;
