/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Sensors for measuring magnetic fields.
//!
//! # Examples
//!
//! ```
//! use magba::prelude::*;
//! use magba::sensors::hall_effect::LinearHallSensor;
//! use nalgebra::{UnitQuaternion, point};
//! # let magnet = CuboidMagnet::default();
//!
//! // Create a linear Hall effect sensor
//! let sensor = LinearHallSensor::new(
//!     [0.0, 0.0, 0.02],             // position (m)
//!     UnitQuaternion::identity(),   // orientation as unit quaternion
//!     [0.0, 0.0, 1.0],              // sensitive axis (automatically normalized)
//!     0.05,                         // sensitivity (V/T)
//!     5.0,                          // supply voltage (V)
//! );
//!
//! // Read the voltage output of the sensor in the presence of the magnet
//! let voltage = sensor.read(&magnet);
//! ```

pub mod hall_effect;
mod sensor;
pub use sensor::Sensor;
