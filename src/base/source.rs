/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use std::fmt::{Debug, Display};

use crate::{
    Pose,
    base::{Float, Transform},
    collections::Component,
    magnets::Magnet,
};
use delegate::delegate;
use dyn_clone::{DynClone, clone_trait_object};
use enum_dispatch::enum_dispatch;
use nalgebra::{Point3, RealField, Vector3};

// MARK: Field

/// Trait shared by objects that generate magnetic field.
#[enum_dispatch]
#[allow(non_snake_case)]
pub trait Field<T: RealField = f64> {
    /// Compute the magnetic field (B) at the given points.
    ///
    /// # Arguments
    /// - `points`: Slice of observer positions (m)
    ///
    /// # Returns
    /// - B-field vectors at each observer.
    fn get_B(&self, points: &[Point3<T>]) -> Vec<Vector3<T>>;
}

// MARK: Source

#[enum_dispatch]
/// Physical representation of magnetic sources.
pub trait Source<T: RealField>: Transform<T> + Field<T> + Send + Sync + DynClone {
    /// A default formatter that behaves like Display.
    /// Last argument is the indentation, which is for Collection support.
    /// Override this for custom printouts.
    fn format(&self, f: &mut std::fmt::Formatter<'_>, _: &str) -> std::fmt::Result {
        write!(f, "Source at {}", self.pose())
    }
}

#[cfg(feature = "std")]
impl<T: RealField> std::fmt::Display for dyn Source<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // Delegate to the trait method
        self.format(f, "")
    }
}

// MARK: Box<dyn Source>

impl<T: Float> Field<T> for Box<dyn Source<T>> {
    fn get_B(&self, points: &[Point3<T>]) -> Vec<Vector3<T>> {
        (**self).get_B(points)
    }
}

impl<T: Float> Transform<T> for Box<dyn Source<T>> {
    delegate!(
        to (**self) {
            fn pose(&self) -> &Pose<T>;
            fn pose_mut(&mut self) -> &mut Pose<T>;
            fn set_pose(&mut self, pose: Pose<T>);
        }
    );
}

clone_trait_object!(<T> Source<T> where T: Float);

impl<T: Float> Debug for Box<dyn Source<T>> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        (**self).fmt(f)
    }
}

impl<T: Float> Source<T> for Box<dyn Source<T>> {}
