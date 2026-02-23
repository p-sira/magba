/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Provides the [`Transform`] trait and macros for implementing it.

use crate::{Collection, Component, Float, Source, geometry::Pose, magnets::*};

use enum_dispatch::enum_dispatch;
use nalgebra::RealField;

/// Trait shared by objects that can return [Pose].
#[enum_dispatch]
pub trait Transform<T: RealField> {
    fn pose(&self) -> &Pose<T>;

    fn pose_mut(&mut self) -> &mut Pose<T>;

    fn set_pose(&mut self, pose: Pose<T>) {
        *self.pose_mut() = pose;
    }
}

macro_rules! impl_transform {
    ($name:ident < $( $args:ty ),* > where $( $bounds:tt )* ) => {
        impl< $( $bounds )* > crate::base::transform::Transform<T> for $name< $( $args ),* > {
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

macro_rules! impl_group_transform {
    ($name:ident < $( $args:ty ),* > where $( $bounds:tt )* ) => {
        impl< $( $bounds )* > $name< $( $args ),*> {
            pub fn set_pose(&mut self, new_pose: impl Into<Pose<T>>) {
                self.pose = new_pose.into();
                for (child, offset) in self.children.iter_mut().zip(&self.offsets) {
                    let global_isometry = self.pose.as_isometry() * offset.as_isometry();
                    child.set_pose(global_isometry.into());
                }
            }

            delegate::delegate! {
                to self.pose {
                    pub fn position(&self) -> Point3<T>;
                    pub fn orientation(&self) -> UnitQuaternion<T>;
                }
            }

            pub fn set_position(&mut self, position: impl Into<Translation3<T>>) {
                let mut new_pose = *self.pose();
                new_pose.set_position(position);
                self.set_pose(new_pose);
            }

            pub fn set_orientation(&mut self, orientation: UnitQuaternion<T>) {
                let mut new_pose = *self.pose();
                new_pose.set_orientation(orientation);
                self.set_pose(new_pose);
            }

            pub fn translate(&mut self, translation: impl Into<Translation3<T>>) {
                let mut new_pose = *self.pose();
                new_pose.translate(translation);
                self.set_pose(new_pose);
            }

            pub fn rotate(&mut self, rotation: UnitQuaternion<T>) {
                let mut new_pose = *self.pose();
                new_pose.rotate(rotation);
                self.set_pose(new_pose);
            }

            pub fn rotate_anchor(&mut self, rotation: UnitQuaternion<T>, anchor: impl Into<Point3<T>>) {
                let mut new_pose = *self.pose();
                new_pose.rotate_anchor(rotation, anchor);
                self.set_pose(new_pose);
            }
        }
    };
}
pub(crate) use impl_group_transform;
