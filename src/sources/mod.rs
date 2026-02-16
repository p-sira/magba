/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Magnetic source traits and structs.

mod define_magnet;

pub mod collections;
mod cuboid;
mod cylinder;
mod dipole;
mod source;
mod zero_magnet;

pub use collections::{BoxedCollection, Collection, SCollection};
pub use cuboid::CuboidMagnet;
pub use cylinder::CylinderMagnet;
pub use dipole::Dipole;
pub use source::{Field, Source};
pub use zero_magnet::ZeroMagnet;
