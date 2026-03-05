/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use nalgebra::{RealField, Vector3};

/// Computes the digital state of a Hall effect latch.
///
/// # Arguments
///
/// - `b_field`: The 3D magnetic flux density vector at the sensor (T)
/// - `sensitive_axis`: A normalized vector representing the sensing direction
/// - `b_op`: The magnetic operate point (T)
/// - `b_rp`: The magnetic release point (T)
/// - `current_state`: The current state of the switch
///
/// # Returns
///
/// - If the projected field exceeds `b_op`, the state becomes `true` (Active).
/// - If the projected field falls below `b_rp`, the state becomes `false` (Inactive).
/// - If the field is between the two, the `current_state` is maintained
#[inline]
pub fn hall_latch_state<T: RealField + Copy>(
    b_field: Vector3<T>,
    sensitive_axis: Vector3<T>,
    b_op: T,
    b_rp: T,
    current_state: bool,
) -> bool {
    let b_proj = b_field.dot(&sensitive_axis);

    if b_proj >= b_op {
        true
    } else if b_proj <= b_rp {
        false
    } else {
        current_state
    }
}
