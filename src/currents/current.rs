/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use crate::crate_utils::display_enum;

use enum_dispatch::enum_dispatch;
use nalgebra::{Point3, Vector3};

use super::{CircularCurrent, TriangleCurrent};

#[cfg(feature = "alloc")]
use super::PathCurrent;

#[cfg(feature = "mesh")]
use super::SheetCurrent;

use crate::base::{Float, Pose, Source, Transform};

display_enum! {
    /// Current source variants.
    #[derive(Clone, Debug, PartialEq, Eq)]
    #[enum_dispatch(Source<T>, Transform<T>,)]
    pub enum Current<T: Float = f64> {
        Circular(CircularCurrent<T>),
        #[cfg(feature = "alloc")]
        Path(PathCurrent<T>),
        #[cfg(feature = "mesh")]
        Sheet(SheetCurrent<T>),
        Triangle(TriangleCurrent<T>),
    }
}
