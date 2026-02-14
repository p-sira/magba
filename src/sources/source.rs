/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Core traits and types for magnetic sources and collections of sources.

use crate::geometry::Pose;
use delegate::delegate;
use nalgebra::{Point3, RealField, Vector3};

/// Trait shared by objects that generate magnetic field.
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

/// Magnetic sources that can apply 3D transformations and calculate magnetic fields.
pub trait Source<T: RealField>: Field<T> + Send + Sync {
    /// Get the pose object.
    fn pose(&self) -> &Pose<T>;

    /// Get the mutable pose object.
    fn pose_mut(&mut self) -> &mut Pose<T>;

    /// Set the pose.
    fn set_pose(&mut self, pose: Pose<T>) {
        *self.pose_mut() = pose;
    }

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

/* #region Box<Source> */

impl<S: Field<T> + ?Sized, T: RealField + Copy> Field<T> for Box<S> {
    fn get_B(&self, points: &[Point3<T>]) -> Vec<Vector3<T>> {
        (**self).get_B(points)
    }
}

impl<S: Source<T> + ?Sized, T: RealField + Copy> Source<T> for Box<S> {
    delegate! {
        to (**self) {
            fn pose(&self) -> &Pose<T>;
            fn pose_mut(&mut self) -> &mut Pose<T>;
            fn set_pose(&mut self, pose: Pose<T>);
            fn format(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result;
        }
    }
}
