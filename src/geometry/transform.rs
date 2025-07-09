/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! # Transform
//!
//! Provides the [`Transform`] trait for 3D position/orientation and macros for implementing it.

use nalgebra::{Point3, Translation3, UnitQuaternion};

/// Trait for transforming objects in 3D Cartesian CS.
pub trait Transform {
    /// Get the object position.
    fn position(&self) -> Point3<f64>;
    /// Get the object orientation.
    fn orientation(&self) -> UnitQuaternion<f64>;
    /// Set the object position.
    fn set_position(&mut self, position: Point3<f64>);
    /// Set the object orientation.
    fn set_orientation(&mut self, orientation: UnitQuaternion<f64>);
    /// Translate the object.
    fn translate(&mut self, translation: &Translation3<f64>);
    /// Rotate the object.
    fn rotate(&mut self, rotation: &UnitQuaternion<f64>);
    /// Rotate the object using the anchor point as the center of rotation.
    fn rotate_anchor(&mut self, rotation: &UnitQuaternion<f64>, anchor: &Point3<f64>);
}

/// Macro to implement shallow [`Transform`] for types with no children.
#[macro_export]
macro_rules! impl_transform {
    ($type:ty) => {
        impl Transform for $type {
            #[inline]
            fn position(&self) -> Point3<f64> {
                self.position
            }

            #[inline]
            fn orientation(&self) -> UnitQuaternion<f64> {
                self.orientation
            }

            #[inline]
            fn set_position(&mut self, position: Point3<f64>) {
                self.position = position;
            }

            #[inline]
            fn set_orientation(&mut self, orientation: UnitQuaternion<f64>) {
                self.orientation = orientation;
            }

            #[inline]
            fn translate(&mut self, translation: &Translation3<f64>) {
                self.position = translation.transform_point(&self.position);
            }

            #[inline]
            fn rotate(&mut self, rotation: &UnitQuaternion<f64>) {
                self.orientation = rotation * &self.orientation;
            }

            #[inline]
            fn rotate_anchor(&mut self, rotation: &UnitQuaternion<f64>, anchor: &Point3<f64>) {
                let local_position = self.position - anchor;
                self.position =
                    Point3::from(rotation * local_position + Vector3::from(anchor.coords));
                self.orientation = rotation * &self.orientation;
            }
        }
    };
}
