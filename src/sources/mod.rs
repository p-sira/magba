/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Magnetic source traits and structs.
//!
//! ## Permanent magnets:
//! - [CylinderMagnet]
//! - [CuboidMagnet]
//!
//! ## [Dipole]
//!
//! ## Magnetic source collection:
//! - [SourceCollection]
//! - [MultiSourceCollection]


mod magnets;

mod cuboid;
mod cylinder;
mod dipole;
mod source;

pub use cuboid::*;
pub use cylinder::*;
pub use dipole::*;
pub use source::*;
