/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Transform object position and orientation. Provide [Transform] trait.

use nalgebra::{Point3, Translation3, UnitQuaternion};

/// Transform object in 3D Cartesian CS.
pub trait Transform {
    fn position(&self) -> Point3<f64>;
    fn orientation(&self) -> UnitQuaternion<f64>;
    fn set_position(&mut self, position: Point3<f64>);
    fn set_orientation(&mut self, orientation: UnitQuaternion<f64>);
    fn translate(&mut self, translation: &Translation3<f64>);
    fn rotate(&mut self, rotation: &UnitQuaternion<f64>);
    fn rotate_anchor(&mut self, rotation: &UnitQuaternion<f64>, anchor: &Point3<f64>);
}

/// Implement shallow [Transform] for objects with no children.
#[macro_export]
macro_rules! impl_transform {
    ($type:ty) => {
        impl Transform for $type {
            /// Get the object position.
            fn position(&self) -> Point3<f64> {
                self.position
            }

            /// Get the object orientation.
            fn orientation(&self) -> UnitQuaternion<f64> {
                self.orientation
            }

            /// Set the object position.
            fn set_position(&mut self, position: Point3<f64>) {
                self.position = position;
            }

            /// Set the object orientation.
            fn set_orientation(&mut self, orientation: UnitQuaternion<f64>) {
                self.orientation = orientation;
            }

            /// Translate the object.
            fn translate(&mut self, translation: &Translation3<f64>) {
                self.position = translation.transform_point(&self.position);
            }

            /// Rotate the object.
            fn rotate(&mut self, rotation: &UnitQuaternion<f64>) {
                self.orientation = rotation * &self.orientation;
            }

            /// Rotate the object using the anchor point as the center of rotation.
            fn rotate_anchor(&mut self, rotation: &UnitQuaternion<f64>, anchor: &Point3<f64>) {
                let local_position = self.position - anchor;
                self.position =
                    Point3::from(rotation * local_position + Vector3::from(anchor.coords));
                self.orientation = rotation * &self.orientation;
            }
        }
    };
}
