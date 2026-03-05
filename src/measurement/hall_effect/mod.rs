/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

mod hall_latch;
mod hall_switch;
mod linear_hall;

pub use hall_latch::hall_latch_state;
pub use hall_switch::hall_switch_state;
pub use linear_hall::{linear_hall_voltage, linear_hall_voltage_batch};
