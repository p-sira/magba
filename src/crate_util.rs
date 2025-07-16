/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Internal utilities for Magba

use nalgebra::{distance, Point3, RealField, Vector3};

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
pub fn relative_vec_distance<T: RealField + Copy>(a: Vector3<T>, b: Vector3<T>) -> T {
    let dist = distance(&Point3::from(a), &Point3::from(b));
    (dist / a.magnitude()).max(dist / b.magnitude())
}

#[allow(unused)]
/// Check if two vectors are close using relative Euclidean distance
pub fn is_vec_close(a: Vector3<f64>, b: Vector3<f64>, rtol: f64) -> bool {
    relative_vec_distance(a, b) <= rtol
}

/// Return the number of failed elements if the elements of two vectors are not close.
pub fn is_elem_close(vec1: &Vector3<f64>, vec2: &Vector3<f64>, rtol: f64) -> Option<usize> {
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
        Some(n_fail)
    } else {
        None
    }
}

macro_rules! format_float {
    ($v: expr) => {
        $v.to_string()
    };
}
pub(crate) use format_float;

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

macro_rules! assert_eq_lens {
    ($str_err:expr, [$ref_vec:expr $(, $vec:expr)+]) => {
        {
            let len = $ref_vec.len();
            $(
                if $vec.len() != len {
                    panic!($str_err)
                }
            )+
        }
    };
}
pub(crate) use assert_eq_lens;

macro_rules! impl_parallel {
    ($func: ident, $threshold: expr, $items: expr, $($func_args: expr),* $(,)?) => {
        {
            #[cfg(feature = "parallel")]
            if $items.len() > $threshold {
                use rayon::iter::{IntoParallelRefIterator, ParallelIterator};
                return $items
                    .par_iter()
                    .map(|p| $func(p, $($func_args),*))
                    .collect();
            }

            $items
                .iter()
                .map(|p| $func(p, $($func_args),*))
                .collect()
        }
    };
}
pub(crate) use impl_parallel;

macro_rules! impl_parallel_sum {
    ($items:expr, [$($vecs:expr),+], |$($args:ident),*| $call:expr) => {{
        use crate::crate_util::assert_eq_lens;
        assert_eq_lens!(
            "Lengths of input vectors must be equal.",
            [$($vecs),+]
        );

        let combinations = itertools::izip!($($vecs),+);

        #[cfg(feature = "parallel")]
        {
            use rayon::iter::{ParallelBridge, ParallelIterator};

            let vectors = combinations
                .par_bridge()
                .map(|($($args),*)| $call)
                .collect::<Vec<Vec<_>>>();

            (0..$items.len())
                .map(|i| vectors.iter().map(|v| v[i]).sum())
                .collect()
        }

        #[cfg(not(feature = "parallel"))]
        {
            let mut net_vectors: Vec<Vector3<_>> = vec![Vector3::zeros(); $items.len()];

            combinations
                .map(|($($args),*)| $call)
                .for_each(|field_vectors| {
                    net_vectors
                        .iter_mut()
                        .zip(field_vectors)
                        .for_each(|(net_vector, field_vector)| *net_vector += field_vector)
                });

            net_vectors
        }
    }};
}
pub(crate) use impl_parallel_sum;
