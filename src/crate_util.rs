/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Internal utilities for Magba

use nalgebra::{distance, Point3, Vector3};

/// Calculate the symmetric relative error
pub fn relative_error(a: f64, b: f64) -> f64 {
    if a == 0.0 && b == 0.0 {
        return 0.0;
    }
    let difference = a - b;
    let rel1 = (difference / a).abs();
    let rel2 = (difference / b).abs();

    if rel1.is_nan() {
        return rel2;
    }
    if rel2.is_nan() {
        return rel1;
    }

    rel1.max(rel2)
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

macro_rules! format_vector3 {
    ($v: expr) => {
        format!("[{}, {}, {}]", $v[0], $v[1], $v[2])
    };
}
pub(crate) use format_vector3;

macro_rules! format_point3 {
    ($p: expr) => {
        format!("[{}, {}, {}]", $p[0], $p[1], $p[2])
    };
}
pub(crate) use format_point3;

macro_rules! format_quat {
    ($q: expr) => {
        format!("[{}, {}, {}, {}]", $q[0], $q[1], $q[2], $q[3])
    };
}
pub(crate) use format_quat;
