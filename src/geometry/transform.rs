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
    fn translate(&mut self, translation: &Translation3<T>) {
        let new_pos = translation.transform_point(&self.position());
        self.set_position(new_pos);
    }
    /// Rotate the object.
    fn rotate(&mut self, rotation: &UnitQuaternion<T>) {
        let new_rot = rotation * self.orientation();
        self.set_orientation(new_rot);
    }
    /// Rotate the object using the anchor point as the center of rotation.
    fn rotate_anchor(&mut self, rotation: &UnitQuaternion<T>, anchor: &Point3<T>) {
        let local_position = self.position() - anchor;
        
        let new_pos = rotation * local_position + anchor.coords;
        let new_rot = rotation * self.orientation();

        self.set_position(new_pos.into());
        self.set_orientation(new_rot);
    }
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
    (@builders) => {
        #[inline]
        pub fn with_position(mut self, position: impl Into<nalgebra::Point3<T>>) -> Self {
            self.position = position.into();
            self
        }

        #[inline]
        pub fn with_orientation(mut self, orientation: impl Into<nalgebra::UnitQuaternion<T>>) -> Self {
            self.orientation = orientation.into();
            self
        }

        #[inline]
        pub fn with_orientation_from_scaled_axis(mut self, scaled_axis: impl Into<nalgebra::Vector3<T>>) -> Self {
            self.orientation = nalgebra::UnitQuaternion::from_scaled_axis(scaled_axis.into());
            self
        }
    };
    () => {
        crate::geometry::impl_transform!(@getters);
        crate::geometry::impl_transform!(@setters);
    };
}
pub(crate) use impl_transform;
