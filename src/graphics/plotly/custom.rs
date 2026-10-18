/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2026 Sira Pornsiriprasert <code@psira.me>
 */

//! Trait objects that preserve physical behavior and rendering in collections.

use super::Graphics;
use crate::base::{Float, Observer, Pose, SensorOutput, Source, Transform};
use nalgebra::{Point3, Vector3};

/// A source whose custom geometry is retained inside collections.
/// Construct components with [`crate::collections::SourceComponent::custom_graphics`].
pub trait GraphicsSource<T: Float = f64>: Source<T> + Graphics<T> {}
/// Retain custom rendering for any source implementing [`Graphics`].
impl<T: Float, S: Source<T> + Graphics<T>> GraphicsSource<T> for S {}

/// An observer whose custom geometry is retained inside collections.
/// Construct components with [`crate::collections::ObserverComponent::custom_graphics`].
pub trait GraphicsObserver<T: Float = f64>: Observer<T> + Graphics<T> {}
/// Retain custom rendering for any observer implementing [`Graphics`].
impl<T: Float, S: Observer<T> + Graphics<T>> GraphicsObserver<T> for S {}

dyn_clone::clone_trait_object!(<T> GraphicsSource<T> where T: Float);
dyn_clone::clone_trait_object!(<T> GraphicsObserver<T> where T: Float);

macro_rules! boxed_graphics {
    ($trait:ident) => {
        /// Delegate pose access and transformations to the boxed object.
        impl<T: Float> Transform<T> for Box<dyn $trait<T>> {
            fn pose(&self) -> &Pose<T> {
                (**self).pose()
            }
            fn pose_mut(&mut self) -> &mut Pose<T> {
                (**self).pose_mut()
            }
            fn set_pose(&mut self, pose: Pose<T>) {
                (**self).set_pose(pose);
            }
        }
        /// Format the boxed object using its physical representation.
        impl<T: Float> core::fmt::Debug for Box<dyn $trait<T>> {
            fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                (**self).format(f, "")
            }
        }
    };
}
boxed_graphics!(GraphicsSource);
boxed_graphics!(GraphicsObserver);

/// Delegate field evaluation and complexity estimates to the boxed source.
impl<T: Float> Source<T> for Box<dyn GraphicsSource<T>> {
    fn compute_B(&self, point: Point3<T>) -> Vector3<T> {
        (**self).compute_B(point)
    }
    fn compute_B_batch(&self, points: &[Point3<T>]) -> Vec<Vector3<T>> {
        (**self).compute_B_batch(points)
    }
    fn relative_complexity(&self) -> usize {
        (**self).relative_complexity()
    }
    fn format(&self, f: &mut core::fmt::Formatter<'_>, indent: &str) -> core::fmt::Result {
        (**self).format(f, indent)
    }
}
/// Delegate sensor readings to the boxed observer.
impl<T: Float> Observer<T> for Box<dyn GraphicsObserver<T>> {
    fn read(&self, source: &dyn Source<T>) -> SensorOutput<T> {
        (**self).read(source)
    }
    fn format(&self, f: &mut core::fmt::Formatter<'_>, indent: &str) -> core::fmt::Result {
        (**self).format(f, indent)
    }
}
