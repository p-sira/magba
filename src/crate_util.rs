/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Internal utilities for Magba

use nalgebra::{distance, Point3, Vector3};

/// Calculate the symmetric relative error
pub fn relative_error(a: f64, b: f64) -> f64 {
    let difference = a - b;
    (difference / a).abs().max((difference / b).abs())
}

/// Check if two numbers are close
#[allow(unused)]
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

/// Panics if two vectors are not close element by element
pub fn assert_close_vector_elem(vec1: &Vector3<f64>, vec2: &Vector3<f64>, rtol: f64) {
    let mut n_fail: usize = 0;
    vec1.iter().zip(vec2).enumerate().for_each(|(n, (&a, &b))| {
        if !is_close(a, b, rtol) {
            eprintln!(
                "Element {} mismatch. actual={}, expected={}, relative={:e}, rtol={:e}",
                n,
                a,
                b,
                relative_error(a, b),
                rtol
            );
            n_fail += 1
        }
    });
    if n_fail > 0 {
        panic!("Failed. Mismatched {n_fail}/3 elements.")
    }
}
