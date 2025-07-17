/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Utilities for Magba.

pub use crate::crate_util::is_elem_close;
pub use crate::crate_util::relative_error;

/// Panics if the elements of two vectors are not close.
#[macro_export]
macro_rules! assert_close_vector_elem {
    ($vec1:expr, $vec2:expr, $rtol:expr) => {
        if let Some(n_fail) = magba::util::is_elem_close($vec1, $vec2, $rtol) {
            panic!("Failed. Mismatched {n_fail}/3 elements.")
        };
    };
}
