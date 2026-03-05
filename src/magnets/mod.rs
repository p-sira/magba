/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Magnets and physical objects that generate magnetic fields.
//!
//! # Examples
//!
//! ```rust
//! use magba::prelude::*;
//! use nalgebra::{UnitQuaternion, point};
//!
//! // Create a cuboid magnet
//! let magnet = CuboidMagnet::new(
//!     [0.0, 0.0, 0.0],              // position (m)
//!     UnitQuaternion::identity(),   // orientation as unit quaternion
//!     [0.0, 0.0, 1.0],              // polarization (T)
//!     [0.01, 0.01, 0.02],           // dimensions (m)
//! );
//!
//! // Compute the B-field at a specific point
//! let b_field = magnet.compute_B(point![0.0, 0.0, 0.02]);
//! ```

mod cuboid;
mod cylinder;
mod dipole;

pub use cuboid::CuboidMagnet;
pub use cylinder::CylinderMagnet;
pub use dipole::Dipole;

mod magnet;
pub use magnet::Magnet;

mod define_magnet;
use define_magnet::define_magnet;
