/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Utilities for Magba

use nalgebra::{distance, Point3, Vector3};

/// Calculate the symmetric relative error
pub fn relative_error(a: f64, b: f64) -> f64 {
    let difference = a - b;
    (difference / a).abs().max((difference / b).abs())
}

/// Check if two numbers are close
pub fn is_close(a: f64, b: f64, rtol: f64) -> bool {
    relative_error(a, b) <= rtol
}

/// Calculate the relative Euclidean distance
pub fn relative_vec_distance(a: Vector3<f64>, b: Vector3<f64>) -> f64 {
    let dist = distance(&Point3::from(a), &Point3::from(b));
    (dist / a.magnitude()).max(dist / b.magnitude())
}

#[allow(unused)]
/// Check if two vectors are close using relative Euclidean distance
pub fn is_vec_close(a: Vector3<f64>, b: Vector3<f64>, rtol: f64) -> bool {
    relative_vec_distance(a, b) <= rtol
}
