/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Sensors for measuring magnetic fields.

pub mod hall_effect;
mod sensor;

pub use hall_effect::*;
pub use sensor::Sensor;
