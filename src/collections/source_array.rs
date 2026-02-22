/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use std::usize;

use nalgebra::{Point3, Translation3, UnitQuaternion, Vector3};

use crate::{
    base::{Field, Float, Source, Transform, transform::impl_transform},
    geometry::Pose,
    transform::impl_group_transform,
};

#[derive(Debug, Clone)]
pub struct SourceArray<S: Source<T>, T: Float, const N: usize> {
    pose: Pose<T>,
    children: [S; N],
    offsets: [Pose<T>; N],
}

impl<S: Source<T>, T: Float, const N: usize> SourceArray<S, T, N> {
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
}

impl_group_transform!(SourceArray<S, T, N> where S: Source<T>, T: Float, const N: usize);

impl<S: Source<T> + Clone, T: Float, const N: usize> Source<T> for SourceArray<S, T, N> {}

impl_transform!(SourceArray<S, T, N> where S: Source<T>, T: Float, const N: usize);

impl<S: Source<T> + Default, T: Float, const N: usize> Default for SourceArray<S, T, N> {
    fn default() -> Self {
        Self {
            pose: Pose::default(),
            children: core::array::from_fn(|_| S::default()),
            offsets: [Pose::default(); N],
        }
    }
}

impl<S: Source<T>, T: Float, const N: usize> Field<T> for SourceArray<S, T, N> {
    #[inline]
    fn get_B(&self, points: &[Point3<T>]) -> Vec<Vector3<T>> {
        let mut net_field = vec![Vector3::zeros(); points.len()];
        #[cfg(feature = "rayon")]
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
        #[cfg(not(feature = "rayon"))]
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
