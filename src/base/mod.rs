/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

mod source;
pub(crate) mod transform;

pub use source::*;
pub use transform::Transform;

use crate::constants::MagneticConstants;

/// Generic trait for floating point numbers compatible with all [Magba](crate) implementations.
///
/// Supports [f32] and [f64].
pub trait Float: nalgebra::RealField + num_traits::Float + MagneticConstants + Copy {}
impl Float for f32 {}
impl Float for f64 {}
