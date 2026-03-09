/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

#[cfg(feature = "std")]
use derive_more::Display;

use enum_dispatch::enum_dispatch;
use nalgebra::{Point3, Vector3};

use super::CircularCurrent;
use crate::base::{Float, Pose, Source, Transform};

/// Current source variants.
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "std", derive(Display))]
#[enum_dispatch(Source<T>, Transform<T>,)]
pub enum Current<T: Float = f64> {
    Circular(CircularCurrent<T>),
}
