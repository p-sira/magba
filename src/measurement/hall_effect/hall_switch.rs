/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use nalgebra::{RealField, Vector3};

/// Computes the digital state of a unipolar Hall effect switch.
///
/// # Arguments
///
/// - `b_field`: The 3D magnetic flux density vector at the sensor (T)
/// - `sensitive_axis`: A normalized vector representing the sensing direction
/// - `b_op`: The magnetic operate point (T)
///
/// # Returns
///
/// - `true` if the switch is ON, `false` otherwise
#[inline]
pub fn hall_switch_state<T: RealField + Copy>(
    b_field: Vector3<T>,
    sensitive_axis: Vector3<T>,
    b_op: T,
) -> bool {
    let b_proj = b_field.dot(&sensitive_axis);

    b_proj >= b_op
}
