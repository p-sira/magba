/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use nalgebra::{Point3, UnitQuaternion, Translation3};

/// Transform the object in 3D CS
pub trait Transform {
    fn position(&self) -> Point3<f64>;
    fn orientation(&self) -> UnitQuaternion<f64>;
    fn set_position(&mut self, position: Point3<f64>);
    fn set_orientation(&mut self, orientation: UnitQuaternion<f64>);
    fn translate(&mut self, translation: Translation3<f64>);
    fn rotate(&mut self, rotation: UnitQuaternion<f64>);
}

/// Implement shallow Transform for objects with no children 
#[macro_export]
macro_rules! impl_transform {
    ($type:ty) => {
        impl Transform for $type {
            fn position(&self) -> Point3<f64> {
                self.position
            }

            fn orientation(&self) -> UnitQuaternion<f64> {
                self.orientation
            }

            fn set_position(&mut self, position: Point3<f64>){
                self.position = position;
            }

            fn set_orientation(&mut self, orientation: UnitQuaternion<f64>) {
                self.orientation = orientation;
            }

            fn translate(&mut self, translation: Translation3<f64>) {
                self.position = translation.transform_point(&self.position);
            }

            fn rotate(&mut self, rotation: UnitQuaternion<f64>) {
                self.orientation = rotation * self.orientation ;
            }
        }
    };
}
