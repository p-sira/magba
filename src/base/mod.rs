/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Base traits and behaviors.

pub(crate) mod coordinate;

pub(crate) mod math;
pub use math::Float;

pub(crate) mod pose;
pub use pose::Pose;

mod observer;
mod source;
pub(crate) mod transform;

pub use observer::{Observer, SensorOutput};
pub use source::Source;
pub use transform::Transform;

/// Wrapper trait for optional [dyn_clone::DynClone](https://docs.rs/dyn-clone/latest/dyn_clone/trait.DynClone.html).
///
/// No-op if the `std` feature is disabled.
#[cfg(feature = "std")]
pub trait DynClone: dyn_clone::DynClone {}
#[cfg(not(feature = "std"))]
pub trait DynClone {}

#[cfg(feature = "std")]
impl<T: dyn_clone::DynClone> DynClone for T {}
#[cfg(not(feature = "std"))]
impl<T> DynClone for T {}
