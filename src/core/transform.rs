/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Provides the [`Transform`] trait and macros for implementing it.

use crate::{collections::Component, core::Float, magnets::Magnet, magnets::*};
use enum_dispatch::enum_dispatch;
use nalgebra::RealField;

#[enum_dispatch]
pub trait Transform<T: RealField> {
    /// Get the pose object.
    fn pose(&self) -> &Pose<T>;

    /// Get the mutable pose object.
    fn pose_mut(&mut self) -> &mut Pose<T>;

    /// Set the pose.
    fn set_pose(&mut self, pose: Pose<T>) {
        *self.pose_mut() = pose;
    }
}

macro_rules! impl_transform {
    ($name:ident < $( $args:ident ),* > where $( $bounds:tt )* ) => {
        impl< $( $bounds )* > crate::core::transform::Transform<T> for $name< $( $args ),* > {
            fn pose(&self) -> &crate::geometry::Pose<T> {
                &self.pose
            }

            fn pose_mut(&mut self) -> &mut crate::geometry::Pose<T> {
                &mut self.pose
            }
        }
    };
}
pub(crate) use impl_transform;

macro_rules! delegate_to_pose {
    () => {
        delegate::delegate! {
            to self.pose {
                pub fn position(&self) -> nalgebra::Point3<T>;
                pub fn orientation(&self) -> nalgebra::UnitQuaternion<T>;
                pub fn set_position(&mut self, position: impl Into<nalgebra::Translation3<T>>);
                pub fn set_orientation(&mut self, orientation: nalgebra::UnitQuaternion<T>);
                pub fn translate(&mut self, translation: impl Into<nalgebra::Translation3<T>>);
                pub fn rotate(&mut self, rotation: nalgebra::UnitQuaternion<T>);
                pub fn rotate_anchor(&mut self, rotation: nalgebra::UnitQuaternion<T>, anchor: impl Into<nalgebra::Point3<T>>);
            }
        }
    };
}
pub(crate) use delegate_to_pose;

use crate::geometry::Pose;
