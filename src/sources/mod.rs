/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Magnetic source traits and structs.

mod magnets;

mod boxed_collection;
mod collection;
mod cuboid;
mod cylinder;
mod dipole;
mod scollection;
mod source;
mod zero_magnet;

pub use boxed_collection::BoxedCollection;
pub use collection::Collection;
pub use cuboid::CuboidMagnet;
pub use cylinder::CylinderMagnet;
pub use dipole::Dipole;
pub use scollection::SCollection;
pub use source::{Field, Source};
pub use zero_magnet::ZeroMagnet;
