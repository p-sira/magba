/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Provides the [`Transform`] trait and macros for implementing it.

use std::fmt::Display;

use nalgebra::{Isometry3, Point3, RealField, Translation3, UnitQuaternion};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose<T: RealField> {
    isometry: Isometry3<T>,
}

impl<T: RealField> Default for Pose<T> {
    fn default() -> Self {
        Self {
            isometry: Isometry3::identity(),
        }
    }
}

impl<T: RealField> Display for Pose<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Source at pos={}, q={}",
            self.isometry.translation, self.isometry.rotation,
        )
    }
}

impl<T: RealField> Pose<T> {
    #[inline]
    pub fn new(position: impl Into<Translation3<T>>, orientation: UnitQuaternion<T>) -> Self {
        Self {
            isometry: Isometry3::from_parts(position.into(), orientation),
        }
    }

    #[inline]
    pub fn position(&self) -> Point3<T> {
        self.isometry.translation.vector.clone().into()
    }

    #[inline]
    pub fn orientation(&self) -> UnitQuaternion<T> {
        self.isometry.rotation.clone()
    }

    #[inline]
    pub fn set_position(&mut self, position: impl Into<Translation3<T>>) {
        self.isometry.translation = position.into();
    }

    #[inline]
    pub fn set_orientation(&mut self, orientation: UnitQuaternion<T>) {
        self.isometry.rotation = orientation;
    }

    /// Translate the object.
    #[inline]
    pub fn translate(&mut self, translation: impl Into<Translation3<T>>) {
        self.isometry.append_translation_mut(&translation.into());
    }

    /// Rotate the object.
    #[inline]
    pub fn rotate(&mut self, rotation: UnitQuaternion<T>) {
        self.isometry.append_rotation_wrt_center_mut(&rotation);
    }

    /// Rotate the object using the anchor point as the center of rotation.
    #[inline]
    pub fn rotate_anchor(&mut self, rotation: UnitQuaternion<T>, anchor: impl Into<Point3<T>>) {
        self.isometry
            .append_rotation_wrt_point_mut(&rotation, &anchor.into());
    }

    #[inline]
    pub fn as_isometry(&self) -> &Isometry3<T> {
        &self.isometry
    }
}

impl<T: RealField> From<Isometry3<T>> for Pose<T> {
    fn from(isometry: Isometry3<T>) -> Self {
        Self { isometry }
    }
}

macro_rules! impl_pose_method {
    () => {
        fn pose(&self) -> &crate::geometry::Pose<T> {
            &self.pose
        }

        fn pose_mut(&mut self) -> &mut crate::geometry::Pose<T> {
            &mut self.pose
        }
    };
}
pub(crate) use impl_pose_method;

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
