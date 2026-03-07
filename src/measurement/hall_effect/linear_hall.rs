/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use nalgebra::{RealField, Vector3};

use crate::crate_utils::impl_parallel;

/// Computes analog voltage output of a linear Hall effect sensor.
///
/// # Arguments
///
/// - `b_field`: The 3D magnetic flux density vector at the sensor (T)
/// - `sensitivity`: A vector representing the sensing direction and sensitivity magnitude (V/T)
/// - `quiescent_voltage`: The zero-field output voltage (V)
/// - `min_voltage`: The lower saturation limit (V)
/// - `max_voltage`: The upper saturation limit (V)
///
/// # Returns
///
/// - Analog output voltage (V)
#[inline]
pub fn linear_hall_voltage<T: RealField + Copy>(
    b_field: Vector3<T>,
    sensitivity: Vector3<T>,
    quiescent_voltage: T,
    min_voltage: T,
    max_voltage: T,
) -> T {
    let raw_voltage = quiescent_voltage + b_field.dot(&sensitivity);
    raw_voltage.clamp(min_voltage, max_voltage)
}

/// Computes analog voltage output of a linear Hall effect sensor for multiple points.
///
/// # Arguments
///
/// - `b_fields`: Array of 3D magnetic flux density vectors (T)
/// - `sensitivity`: A vector representing the sensing direction and sensitivity magnitude (V/T)
/// - `quiescent_voltage`: The zero-field output voltage (V)
/// - `min_voltage`: The lower saturation limit (V)
/// - `max_voltage`: The upper saturation limit (V)
/// - `out`: Mutable slice to store the analog voltages (V)
pub fn linear_hall_voltage_batch<T: RealField + Copy>(
    b_fields: &[Vector3<T>],
    sensitivity: Vector3<T>,
    quiescent_voltage: T,
    min_voltage: T,
    max_voltage: T,
    out: &mut [T],
) {
    impl_parallel!(
        linear_hall_voltage,
        rayon_threshold: 60,
        input: b_fields,
        output: out,
        args: [sensitivity, quiescent_voltage, min_voltage, max_voltage]
    )
}
