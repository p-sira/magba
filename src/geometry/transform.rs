/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Provides the [`Transform`] trait and macros for implementing it.

use nalgebra::{Point3, RealField, Translation3, UnitQuaternion};

/// Trait for transforming objects in 3D Cartesian CS.
pub trait Transform<T: RealField + Copy> {
    /// Get the object position.
    fn position(&self) -> Point3<T>;
    /// Get the object orientation.
    fn orientation(&self) -> UnitQuaternion<T>;
    /// Set the object position.
    fn set_position(&mut self, position: Point3<T>);
    /// Set the object orientation.
    fn set_orientation(&mut self, orientation: UnitQuaternion<T>);
    /// Translate the object.
    fn translate(&mut self, translation: &Translation3<T>);
    /// Rotate the object.
    fn rotate(&mut self, rotation: &UnitQuaternion<T>);
    /// Rotate the object using the anchor point as the center of rotation.
    fn rotate_anchor(&mut self, rotation: &UnitQuaternion<T>, anchor: &Point3<T>);
}

/// Macro to implement shallow [`Transform`] for types with no children.
#[macro_export]
macro_rules! impl_transform {
    () => {
        #[inline]
        fn position(&self) -> Point3<T> {
            self.position
        }

        #[inline]
        fn orientation(&self) -> UnitQuaternion<T> {
            self.orientation
        }

        #[inline]
        fn set_position(&mut self, position: Point3<T>) {
            self.position = position;
        }

        #[inline]
        fn set_orientation(&mut self, orientation: UnitQuaternion<T>) {
            self.orientation = orientation;
        }

        #[inline]
        fn translate(&mut self, translation: &Translation3<T>) {
            self.position = translation.transform_point(&self.position);
        }

        #[inline]
        fn rotate(&mut self, rotation: &UnitQuaternion<T>) {
            self.orientation = rotation * &self.orientation;
        }

        #[inline]
        fn rotate_anchor(&mut self, rotation: &UnitQuaternion<T>, anchor: &Point3<T>) {
            let local_position = self.position - anchor;
            self.position = Point3::from(rotation * local_position + Vector3::from(anchor.coords));
            self.orientation = rotation * &self.orientation;
        }
    };
}
