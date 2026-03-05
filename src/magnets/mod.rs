/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Magnets and physical objects that generate magnetic fields.
//!
//! # Declaring Magnets
//!
//! Using constructor:
//! ```
//! # use magba::prelude::*;
//! # use nalgebra::UnitQuaternion;
//! let magnet = CuboidMagnet::new(
//!     [0.0, 0.0, 0.0],              // position (m)
//!     UnitQuaternion::identity(),   // orientation as unit quaternion
//!     [0.0, 0.0, 1.0],              // polarization (T)
//!     [0.01, 0.01, 0.02],           // dimensions (m)
//! );
//! ```
//!
//! Using builder pattern:
//! ```
//! # use magba::magnets::CuboidMagnet;
//! # use nalgebra::UnitQuaternion;
//! let magnet = CuboidMagnet::default()
//!     .with_position([0.0, 0.0, 0.0])
//!     .with_orientation(UnitQuaternion::identity())
//!     .with_polarization([0.0, 0.0, 1.0])
//!     .with_dimensions([0.01, 0.01, 0.02]);
//! ```
//!
//! # Computing B-field
//!
//! ```
//! # use magba::prelude::*;
//! # use nalgebra::point;
//! # let magnet = CuboidMagnet::default();
//! // Compute the B-field at a specific point
//! let b_field = magnet.compute_B(point![0.0, 0.0, 0.02]);
//!
//! // Compute the B-field at multiple points
//! let points = vec![point![0.0, 0.0, 0.02], point![0.0, 0.0, 0.03]];
//! let b_fields = magnet.compute_B_batch(&points);
//! ```
//!
//! With the `rayon` feature (default), `compute_B_batch` automatically parallelizes the magnetic
//! field computation using [Rayon](https://github.com/rayon-rs/rayon) if the number of input
//! points is greater than the threshold to overcome the parallelization overhead.
//! If you disable the `rayon` feature, it will fallback to sequential computation.

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
