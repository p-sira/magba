/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Provides the [`Transform`] trait and macros for implementing it.

use enum_dispatch::enum_dispatch;
use nalgebra::RealField;

use crate::{
    base::{Float, Observer, Pose, Source},
    collections::{SensorComponent, SourceAssembly, SourceComponent},
    magnets::{CuboidMagnet, CylinderMagnet, Dipole, Magnet},
    sensors::{LinearHallSensor, Sensor},
};

/// Trait shared by objects that can return [Pose].
///
/// In addition to implementing this trait, all objects in Magba
/// implement methods for position and orientation manipulation
/// in 3D cartesian space. Please refer to any magnet structs (such as the [CuboidMagnet](CuboidMagnet#impl-CuboidMagnet<T>-2))
/// for the full list of transformation methods.
///
/// # Examples
///
/// ```
/// # use magba::*;
/// # use approx::assert_relative_eq;
/// # use nalgebra::*;
/// # use std::f64::consts::PI;
/// let mut magnet = CylinderMagnet::default()
///     .with_polarization([0.0, 0.0, 0.9])
///     .with_diameter(0.01)
///     .with_height(0.02);
/// # let observer = point![0.0, 0.0, 0.05];
/// # assert_relative_eq!(
/// #     magnet.compute_B(observer),
/// #     vector![0.0, 0.0, 0.0019205466890453442]
/// # );
///
/// // Translating the magnet and getting its position
///
/// magnet.translate([0.0, 0.0, 0.01]);
/// # assert_relative_eq!(
/// #     magnet.compute_B(observer),
/// #     vector![0.0, 0.0, 0.0038894698700304275]
/// # );
/// assert_eq!(magnet.position(), point![0.0, 0.0, 0.01]);
///
/// magnet.set_position([0.0, 0.0, 0.02]);
/// # assert_relative_eq!(
/// #     magnet.compute_B(observer),
/// #     vector![0.0, 0.0, 0.00996091945575112]
/// # );
/// assert_eq!(magnet.position(), point![0.0, 0.0, 0.02]);
///
/// // Rotating the magnet and getting its orientation
///
/// let rotation = UnitQuaternion::from_scaled_axis([PI / 4.0, 0.0, 0.0].into());
/// magnet.rotate(rotation);
/// # assert_relative_eq!(
/// #     magnet.compute_B(observer),
/// #     vector![
/// #         3.9407500527173422e-19,
/// #         0.0035238379945531874,
/// #         0.005577663229073966
/// #     ]
/// # );
/// assert_eq!(magnet.orientation(), rotation);
///
/// let rotation = UnitQuaternion::from_scaled_axis([PI / 2.0, 0.0, 0.0].into());
/// magnet.set_orientation(rotation);
/// # assert_relative_eq!(
/// #     magnet.compute_B(observer),
/// #     vector![6.086025172136602e-35, 0.003642460886175623, 0.0]
/// # );
/// assert_eq!(magnet.orientation(), rotation);
/// ```
///
/// Please refer to any magnet structs (such as the [CuboidMagnet](CuboidMagnet#impl-CuboidMagnet<T>-2))
/// for the full list of transformation methods.
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
            fn pose(&self) -> &crate::base::Pose<T> {
                &self.pose
            }

            fn pose_mut(&mut self) -> &mut crate::base::Pose<T> {
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
                for node in self.nodes.iter_mut() {
                    let global_isometry = self.pose.as_isometry() * node.local_offset.as_isometry();
                    node.component.set_pose(global_isometry.into());
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
