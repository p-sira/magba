/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Provides the [`Transform`] trait and macros for implementing it.

use nalgebra::{Point3, RealField, Translation3, UnitQuaternion};

/// Trait for transforming objects in 3D Cartesian CS with methods
/// for translation, rotation, and rotation with anchor.
pub trait Transform<T: RealField + Copy> {
    /// Get the object position.
    fn position(&self) -> Point3<T>;
    /// Get the object orientation.
    fn orientation(&self) -> UnitQuaternion<T>;
    /// Set the object position.
    fn set_position(&mut self, position: Point3<T>);
    /// Set the object orientation.
    fn set_orientation(&mut self, orientation: UnitQuaternion<T>);
    /// Set the object orientation from scaled axis (rotation vector)
    fn set_orientation_from_scaled_axis(&mut self, scaled_axis: nalgebra::Vector3<T>);
    /// Translate the object.
    fn translate(&mut self, translation: &Translation3<T>);
    /// Rotate the object.
    fn rotate(&mut self, rotation: &UnitQuaternion<T>);
    /// Rotate the object using the anchor point as the center of rotation.
    fn rotate_anchor(&mut self, rotation: &UnitQuaternion<T>, anchor: &Point3<T>);
}

/// Macro to implement [`Transform`].
macro_rules! impl_transform {
    (@getters) => {
        #[inline]
        fn position(&self) -> nalgebra::Point3<T> {
            self.position
        }

        #[inline]
        fn orientation(&self) -> nalgebra::UnitQuaternion<T> {
            self.orientation
        }
    };
    (@setters) => {
        #[inline]
        fn set_position(&mut self, position: nalgebra::Point3<T>) {
            self.position = position;
        }

        #[inline]
        fn set_orientation(&mut self, orientation: nalgebra::UnitQuaternion<T>) {
            self.orientation = orientation;
        }

        #[inline]
        fn set_orientation_from_scaled_axis(&mut self, scaled_axis: nalgebra::Vector3<T>) {
            self.orientation = nalgebra::UnitQuaternion::from_scaled_axis(scaled_axis);
        }
    };
    (@shallow_transform) => {
        #[inline]
        fn translate(&mut self, translation: &nalgebra::Translation3<T>) {
            self.position = translation.transform_point(&self.position);
        }

        #[inline]
        fn rotate(&mut self, rotation: &nalgebra::UnitQuaternion<T>) {
            self.orientation = rotation * self.orientation;
        }

        #[inline]
        fn rotate_anchor(
            &mut self,
            rotation: &nalgebra::UnitQuaternion<T>,
            anchor: &nalgebra::Point3<T>,
        ) {
            let local_position = self.position - anchor;
            self.position = nalgebra::Point3::from(
                rotation * local_position + nalgebra::Vector3::from(anchor.coords),
            );
            self.orientation = rotation * self.orientation;
        }
    };
    () => {
        crate::geometry::impl_transform!(@getters);
        crate::geometry::impl_transform!(@setters);
        crate::geometry::impl_transform!(@shallow_transform);
    };
}
pub(crate) use impl_transform;
