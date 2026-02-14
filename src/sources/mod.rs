/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Magnetic source traits and structs.

mod magnets;

mod scollection;
mod cuboid;
mod cylinder;
mod collection;
mod dipole;
mod source;
mod zero_magnet;

pub use scollection::SCollection;
pub use cuboid::CuboidMagnet;
pub use cylinder::CylinderMagnet;
pub use collection::{BoxedCollection, Collection};
pub use dipole::Dipole;
pub use source::{Field, Source};
pub use zero_magnet::ZeroMagnet;
