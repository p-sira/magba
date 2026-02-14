/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Magnetic source traits and structs.

mod magnets;

mod collection;
mod cuboid;
mod cylinder;
mod dcollection;
mod dipole;
mod source;
mod zero_magnet;

pub use collection::Collection;
pub use cuboid::CuboidMagnet;
pub use cylinder::CylinderMagnet;
pub use dcollection::{MultiSourceCollection, SourceCollection};
pub use dipole::Dipole;
pub use source::{Field, Source};
pub use zero_magnet::ZeroMagnet;
