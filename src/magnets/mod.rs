/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

mod cuboid;
mod cylinder;
mod dipole;
mod zero_magnet;

pub use cuboid::CuboidMagnet;
pub use cylinder::CylinderMagnet;
pub use dipole::Dipole;
pub use zero_magnet::ZeroMagnet;

mod magnet;
pub use magnet::Magnet;

mod define_magnet;
use define_magnet::define_magnet;