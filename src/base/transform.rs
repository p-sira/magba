/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Provides the [`Transform`] trait and macros for implementing it.

use crate::{base::Float, collections::Component, geometry::Pose, magnets::*};
use enum_dispatch::enum_dispatch;
use nalgebra::RealField;

/// Trait shared by objects that can return [Pose].
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
