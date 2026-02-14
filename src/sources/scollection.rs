/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use nalgebra::{Point3, Translation3, UnitQuaternion, Vector3};

use crate::{
    Field, Float, Source,
    geometry::{Pose, Transform, impl_transform},
};

#[derive(Debug)]
pub struct SCollection<S: Source<T>, T: Float, const N: usize> {
    pose: Pose<T>,
    children: [S; N],
    offsets: [Pose<T>; N],
}

impl<S: Source<T>, T: Float, const N: usize> SCollection<S, T, N> {
    /// Initialize [SourceCollection].
    pub fn new(position: Point3<T>, orientation: UnitQuaternion<T>, sources: [S; N]) -> Self {
        let pose = Pose::new(position, orientation);
        let pose_inv = pose.as_isometry().inverse();

        let offsets = core::array::from_fn(|i| (pose_inv * sources[i].pose().as_isometry()).into());

        Self {
            pose,
            children: sources,
            offsets,
        }
    }

    /// Initialize [SourceCollection] from a vec of homogeneous [Source].
    pub fn from_sources(sources: [S; N]) -> Self {
        let offsets = core::array::from_fn(|i| sources[i].pose().clone());

        Self {
            pose: Pose::default(),
            children: sources,
            offsets,
        }
    }

    pub fn set_pose(&mut self, new_pose: impl Into<Pose<T>>) {
        self.pose = new_pose.into();

        // Re-calculate absolute positions for all children
        // This wipes out any drift that might have occurred
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

impl<S: Source<T>, T: Float, const N: usize> Source<T> for SCollection<S, T, N> {}

impl_transform!(SCollection<S, T, N> where S: Source<T>, T: Float, const N: usize);

impl<S: Source<T> + Default, T: Float, const N: usize> Default for SCollection<S, T, N> {
    fn default() -> Self {
        Self {
            pose: Pose::default(),
            children: core::array::from_fn(|_| S::default()),
            offsets: [Pose::default(); N],
        }
    }
}

impl<S: Source<T>, T: Float, const N: usize> Field<T> for SCollection<S, T, N> {
    #[inline]
    fn get_B(&self, points: &[Point3<T>]) -> Vec<Vector3<T>> {
        let mut net_field = vec![Vector3::zeros(); points.len()];
        #[cfg(feature = "parallel")]
        {
            use rayon::prelude::*;
            let b_fields: Vec<_> = self
                .children
                .par_iter()
                .map(|source| source.get_B(points))
                .collect();
            b_fields.iter().for_each(|child_b_field| {
                net_field
                    .iter_mut()
                    .zip(child_b_field)
                    .for_each(|(sum, b)| *sum += b)
            });
        }
        #[cfg(not(feature = "parallel"))]
        {
            for source in &self.children {
                let b_fields = source.get_B(points);
                net_field
                    .iter_mut()
                    .zip(b_fields)
                    .for_each(|(sum, b)| *sum += b);
            }
        }
        net_field
    }
}
