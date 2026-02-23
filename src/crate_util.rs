/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Internal utilities for Magba

use nalgebra::{Point3, RealField, Vector3, distance};

macro_rules! need_feature {
    ($feature:literal, $($body:item)*) => {
        $(
            #[cfg(feature = $feature)]
            $body
        )*
    };
}
pub(crate) use need_feature;

macro_rules! need_std {
    ($($body:item)*) => {
        $(
            #[cfg(not(feature = "no_std"))]
            $body
        )*
    };
}
pub(crate) use need_std;

macro_rules! eprint_if_std {
    ($($args:expr),* $(,)?) => {
        #[cfg(not(feature = "no_std"))]
        eprintln!(
            $($args,)*
        );
    };
}
pub(crate) use eprint_if_std;

macro_rules! return_vec_or_array {
    (
        $(#[$attr:meta])*
        $vis:vis fn $name:ident <($($generics:tt)*)> (
            $($arg:ident : $arg_ty:ty),* $(,)?
        ) -> [$ret_ty:ty]
        {
            $($body:tt)*
        }
    ) => {
        #[cfg(not(feature = "no_std"))]
        $(#[$attr])*
        $vis fn $name<$($generics)*>(
            $($arg: $arg_ty),*
        ) -> Vec<$ret_ty> {
            $($body)*
        }

        #[cfg(feature = "no_std")]
        $(#[$attr])*
        $vis fn $name<$($generics)*>(
            $($arg: $arg_ty),*
        ) -> [$ret_ty; crate::SIZE] {
            $($body)*
        }
    };
}
pub(crate) use return_vec_or_array;

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
#[allow(unused)]
pub fn relative_vec_distance<T: RealField + Copy>(a: Vector3<T>, b: Vector3<T>) -> T {
    let dist = distance(&Point3::from(a), &Point3::from(b));
    (dist / a.magnitude()).max(dist / b.magnitude())
}

/// Return the number of failed elements if the elements of two vectors are not close.
#[allow(unused)]
pub fn is_elem_close(vec1: &Vector3<f64>, vec2: &Vector3<f64>, rtol: f64) -> Option<usize> {
    let mut n_fail: usize = 0;
    vec1.iter().zip(vec2).enumerate().for_each(|(n, (&a, &b))| {
        if !is_close(a, b, rtol) {
            eprint_if_std!(
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
    if n_fail > 0 { Some(n_fail) } else { None }
}

pub(crate) fn write_tree<'a, T: Float, S: Source<T> + 'a>(
    f: &mut std::fmt::Formatter<'_>,
    leafs: impl IntoIterator<Item = &'a S>,
    indent: &str,
) -> std::fmt::Result {
    let mut iter = leafs.into_iter().enumerate().peekable();

    while let Some((i, leaf)) = iter.next() {
        let is_last = iter.peek().is_none();
        let branch = if is_last { "└── " } else { "├── " };

        write!(f, " {}{}{}: ", indent, branch, i)?;

        let extension = if is_last { "    " } else { "│   " };
        let next_indent = format!("{}{}", indent, extension);
        leaf.format(f, &next_indent)?;

        if !is_last {
            writeln!(f)?;
        }
    }
    Ok(())
}

// need_std! {
//     macro_rules! format_point3 {
//         ($p: expr) => {
//             format!("[{}, {}, {}]", $p[0], $p[1], $p[2])
//         };
//     }
//     pub(crate) use format_point3;

//     macro_rules! format_quat {
//         ($q: expr) => {
//             format!("[{}, {}, {}, {}]", $q[0], $q[1], $q[2], $q[3])
//         };
//     }
//     pub(crate) use format_quat;
// }

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
            #[cfg(feature = "rayon")]
            if $items.len() > $threshold {
                use rayon::iter::{IntoParallelRefIterator, ParallelIterator};
                return $items
                    .par_iter()
                    .map(|p| $func(p, $($func_args),*))
                    .collect();
            }

            #[cfg(not(feature = "no_std"))]
            {
                $items
                    .iter()
                    .map(|p| $func(p, $($func_args),*))
                    .collect()
            }

            #[cfg(feature = "no_std")]
            {
                let mut results = [Vector3::zeros(); crate::SIZE];
                $items
                    .iter()
                    .enumerate()
                    .for_each(|(i, p)| results[i] = $func(p, $($func_args),*));

                results
            }
        }
    };
}
pub(crate) use impl_parallel;

// TODO: compare par_bridge with zip -> par_iter
macro_rules! impl_parallel_sum {
    ($items:expr, [$($vecs:expr),+], |$($args:ident),*| $call:expr) => {{
        use crate::crate_util::assert_eq_lens;
        assert_eq_lens!(
            "Lengths of input vectors must be equal.",
            [$($vecs),+]
        );

        let combinations = itertools::izip!($($vecs),+);

        #[cfg(feature = "rayon")]
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

        #[cfg(not(feature = "rayon"))]
        {
            #[cfg(not(feature = "no_std"))]
            let mut net_vectors: Vec<Vector3<_>> = vec![Vector3::zeros(); $items.len()];

            #[cfg(feature = "no_std")]
            let mut net_vectors = [Vector3::zeros(); crate::SIZE];

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

macro_rules! pub_on_feature {
    {$feature:literal, $($kw:ident {$($item:ident),+ $(,)?})+ $(,)?} => {
        $($(
            #[cfg(feature=$feature)]
            pub $kw $item;
            #[cfg(not(feature=$feature))]
            pub(crate) $kw $item;
        )+)+
    };
}
pub(crate) use pub_on_feature;

use crate::{Float, Source};
