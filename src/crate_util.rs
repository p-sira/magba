/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Internal utilities for Magba

macro_rules! need_std {
    ($($body:item)*) => {
        $(
            #[cfg(feature = "std")]
            $body
        )*
    };
}
pub(crate) use need_std;


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
    ($out:expr, $func:ident, $threshold:expr, $items:expr, $($func_args:expr),* $(,)?) => {
        {
            assert_eq!($out.len(), $items.len(), "Output slice length must match points length.");

            #[cfg(feature = "rayon")]
            if $items.len() > $threshold {
                use rayon::iter::{IndexedParallelIterator, IntoParallelRefIterator, IntoParallelRefMutIterator, ParallelIterator};
                $out.par_iter_mut()
                    .zip($items.par_iter())
                    .for_each(|(o, p)| *o = $func(p, $($func_args),*));
                return;
            }

            $out.iter_mut()
                .zip($items.iter())
                .for_each(|(o, p)| *o = $func(p, $($func_args),*));
        }
    };
}
pub(crate) use impl_parallel;

// TODO: compare par_bridge with zip -> par_iter
macro_rules! impl_parallel_sum {
    ($out:expr, $points:expr, $threshold:expr, [$($vecs:expr),+], |$p:ident, $($args:ident),*| $calc:expr) => {{
        crate::crate_util::assert_eq_lens!(
            "Lengths of inputs must be equal.",
            [$out, $points, $($vecs),+]
        );

        #[cfg(feature = "rayon")]
        if $points.len() > $threshold {
            use rayon::iter::{IndexedParallelIterator, IntoParallelRefIterator, IntoParallelRefMutIterator, ParallelIterator};
            $out.par_iter_mut().zip($points.par_iter()).for_each(|(o, p_ref)| {
                let mut sum = nalgebra::Vector3::zeros();
                for ($($args),*) in itertools::izip!($($vecs),+) {
                    let $p = p_ref;
                    sum += $calc;
                }
                *o = sum;
            });
            return;
        }

        $out.iter_mut().zip($points.iter()).for_each(|(o, p_ref)| {
            let mut sum = nalgebra::Vector3::zeros();
            for ($($args),*) in itertools::izip!($($vecs),+) {
                let $p = p_ref;
                sum += $calc;
            }
            *o = sum;
        });
    }};
}
pub(crate) use impl_parallel_sum;
