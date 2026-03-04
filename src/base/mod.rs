/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Base traits and behaviors.

pub(crate) mod coordinate;

pub(crate) mod pose;
pub use pose::Pose;

mod observer;
mod source;
pub(crate) mod transform;

pub use observer::{Observer, SensorOutput};
pub use source::Source;
pub use transform::Transform;

#[cfg(feature = "std")]
pub trait DynClone: dyn_clone::DynClone {}
#[cfg(feature = "std")]
impl<T: dyn_clone::DynClone> DynClone for T {}

#[cfg(not(feature = "std"))]
pub trait DynClone {}
#[cfg(not(feature = "std"))]
impl<T> DynClone for T {}

const MU0: f64 = 1.2566370614359173e-6;

/// Generic trait for floating point numbers compatible with all [Magba](crate) implementations.
///
/// Supports [f32] and [f64].
pub trait Float: nalgebra::RealField + num_traits::Float + Copy {
    /// Permeability of free space (μ₀) = 4π × 10⁻⁷ H/m.
    fn mu0() -> Self;
}

impl Float for f32 {
    #[inline]
    fn mu0() -> Self {
        MU0 as f32
    }
}

impl Float for f64 {
    #[inline]
    fn mu0() -> Self {
        MU0
    }
}
