/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Implementations of Hall effect sensors

mod hall_latch;
mod hall_switch;
mod linear_hall_sensor;

pub use hall_latch::HallLatch;
pub use hall_switch::HallSwitch;
pub use linear_hall_sensor::LinearHallSensor;

/// Normalize an axis only when its norm is finite and nonzero.
fn normalized_axis<T: crate::base::Float>(axis: nalgebra::Vector3<T>) -> nalgebra::Vector3<T> {
    let norm = axis.norm();
    assert!(
        norm.is_finite() && norm > T::zero(),
        "Sensitive axis must be finite and non-zero."
    );
    axis / norm
}
