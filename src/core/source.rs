/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use crate::{
    collections::component::Component,
    core::{Float, Transform},
    magnets::Magnet,
};
use enum_dispatch::enum_dispatch;
use nalgebra::{Point3, RealField, Vector3};

/// Trait shared by objects that generate magnetic field.
#[enum_dispatch]
#[allow(non_snake_case)]
pub trait Field<T: RealField> {
    /// Compute the magnetic field (B) at the given points.
    ///
    /// # Arguments
    /// - `points`: Slice of observer positions (m)
    ///
    /// # Returns
    /// - B-field vectors at each observer.
    fn get_B(&self, points: &[Point3<T>]) -> Vec<Vector3<T>>;
}

#[enum_dispatch]
/// Magnetic sources that can apply 3D transformations and calculate magnetic fields.
pub trait Source<T: RealField>: Transform<T> + Field<T> + Send + Sync {
    /// A default formatter that behaves like Display.
    /// Override this for custom printouts.
    fn format(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Source at pos={}, q={}",
            self.pose().position(),
            self.pose().orientation()
        )
    }
}

impl<T: RealField> std::fmt::Display for dyn Source<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // Delegate to the trait method
        self.format(f)
    }
}
