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

need_std!(
    use crate::base::Float;
    use nalgebra::Vector3;
    use std::fmt::Formatter;

    pub(crate) fn format_float<T: Float>(f: &mut Formatter, v: T) -> String {
        if let Some(p) = f.precision() {
            format!("{:.p$}", v, p = p)
        } else {
            format!("{:?}", v)
        }
    }

    pub(crate) fn format_vector3<T: Float>(f: &mut Formatter, v: Vector3<T>) -> String {
        if let Some(p) = f.precision() {
            format!("[{:.p$}, {:.p$}, {:.p$}]", v.x, v.y, v.z, p = p)
        } else {
            format!("[{:?}, {:?}, {:?}]", v.x, v.y, v.z)
        }
    }
);

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
    ($func:ident, rayon_threshold: $threshold:expr, input: $inputs:expr, output: $out:expr, args: [$($func_args:expr),* $(,)?]) => {
        {
            assert_eq!($out.len(), $inputs.len(), "Output slice length must match input vectors length.");

            #[cfg(feature = "rayon")]
            if $inputs.len() > $threshold {
                use rayon::iter::*;
                $out.par_iter_mut()
                    .zip($inputs.par_iter())
                    .for_each(|(o, p)| *o = $func(*p, $($func_args),*));
                return;
            }

            $out.iter_mut()
                .zip($inputs.iter())
                .for_each(|(o, p)| *o = $func(*p, $($func_args),*));
        }
    };
}
pub(crate) use impl_parallel;

// TODO: compare par_bridge with zip -> par_iter
macro_rules! impl_parallel_sum {
    ($out:expr, $points:expr, $threshold:expr, [$($vecs:expr),+], |$p:ident, $($args:ident),*| $calc:expr) => {{
        assert_eq!($out.len(), $points.len(), "Output slice length must match points length.");
        crate::crate_util::assert_eq_lens!(
            "Lengths of input vectors must be equal.",
            [$($vecs),+]
        );

        #[cfg(feature = "rayon")]
        if $points.len() > $threshold {
            use rayon::iter::*;
            $out.par_iter_mut()
                .zip($points.par_iter())
                .for_each(|(o, p_ref)| {
                    let $p = p_ref;
                    *o = itertools::izip!($($vecs),+)
                        .fold(nalgebra::Vector3::zeros(), |acc, ($($args),*)| {
                            acc + $calc
                        });
                });
            return;
        }

        $out.iter_mut()
            .zip($points.iter())
            .for_each(|(o, p_ref)| {
                let $p = p_ref;
                *o = itertools::izip!($($vecs),+)
                    .fold(nalgebra::Vector3::zeros(), |acc, ($($args),*)| {
                        acc + $calc
                    });
            });
    }};
}
pub(crate) use impl_parallel_sum;
