/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use core::ops::{Index, IndexMut};
use std::fmt::Display;

use nalgebra::{Point3, Translation3, UnitQuaternion, Vector3};

use crate::{
    base::{Float, Source, Transform, transform::impl_transform},
    geometry::Pose,
    transform::impl_group_transform,
};

#[macro_export]
macro_rules! sources {
    ($($items:expr),*) => {
        SourceArray::new([0.0; 3], nalgebra::UnitQuaternion::identity(), [$($items),*])
    };
}

/// Stack-allocated data structure for grouping homogeneous [Source].
///
/// ### Examples
///
/// ```
/// # use magba::*;
/// let cuboid1 = CuboidMagnet::default();
/// let cuboid2 = cuboid1.clone().with_position([0.0, 1.0, 0.0]);
///
/// let source_array = sources!(cuboid1, cuboid2);
/// ```
#[derive(Debug, Clone)]
pub struct SourceArray<S: Source<T>, T: Float, const N: usize> {
    pose: Pose<T>,
    children: [S; N],
    offsets: [Pose<T>; N],
}

impl<S: Source<T>, T: Float, const N: usize> SourceArray<S, T, N> {
    pub fn new(
        position: impl Into<Point3<T>>,
        orientation: UnitQuaternion<T>,
        sources: [S; N],
    ) -> Self {
        let pose = Pose::new(position.into(), orientation);
        let pose_inv = pose.as_isometry().inverse();

        let offsets = core::array::from_fn(|i| (pose_inv * sources[i].pose().as_isometry()).into());

        Self {
            pose,
            children: sources,
            offsets,
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, S> {
        self.children.iter()
    }
}

impl<S: Source<T> + Default, T: Float, const N: usize> Default for SourceArray<S, T, N> {
    fn default() -> Self {
        Self {
            pose: Pose::default(),
            children: core::array::from_fn(|_| S::default()),
            offsets: [Pose::default(); N],
        }
    }
}

// MARK: Transform

impl_transform!(SourceArray<S, T, N> where S: Source<T>, T: Float, const N: usize);
impl_group_transform!(SourceArray<S, T, N> where S: Source<T>, T: Float, const N: usize);

// MARK: Source

impl<S: Source<T> + Clone, T: Float, const N: usize> Source<T> for SourceArray<S, T, N> {
#[inline]
    fn compute_B(&self, point: Point3<T>) -> Vector3<T> {
        let mut net_field = Vector3::zeros();
        for source in &self.children {
            net_field += source.compute_B(point);
        }
        net_field
    }

    #[inline]
    fn compute_B_batch(&self, points: &[Point3<T>]) -> Vec<Vector3<T>> {
        let mut net_field = vec![Vector3::zeros(); points.len()];

        #[cfg(feature = "rayon")]
        {
            use rayon::prelude::*;

            let b_fields: Vec<_> = self
                .children
                .par_iter()
                .map(|source| source.compute_B_batch(points))
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
                let b_fields = source.compute_B_batch(points);
                net_field
                    .iter_mut()
                    .zip(b_fields)
                    .for_each(|(sum, b)| *sum += b);
            }
        }
        net_field
    }
}

// MARK: Index

impl<S: Source<T>, T: Float, const N: usize> Index<usize> for SourceArray<S, T, N> {
    type Output = S;

    fn index(&self, index: usize) -> &Self::Output {
        &self.children[index]
    }
}

impl<S: Source<T>, T: Float, const N: usize> IndexMut<usize> for SourceArray<S, T, N> {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.children[index]
    }
}

// MARK: From, Into

impl<S: Source<T>, T: Float, const N: usize> From<[S; N]> for SourceArray<S, T, N> {
    fn from(sources: [S; N]) -> Self {
        let offsets = core::array::from_fn(|i| *sources[i].pose());

        Self {
            pose: Pose::default(),
            children: sources,
            offsets,
        }
    }
}

impl<'a, S: Source<T>, T: Float, const N: usize> IntoIterator for &'a SourceArray<S, T, N> {
    type Item = &'a S;
    type IntoIter = std::slice::Iter<'a, S>;

    fn into_iter(self) -> Self::IntoIter {
        self.children.iter()
    }
}

impl<S: Source<T>, T: Float, const N: usize> IntoIterator for SourceArray<S, T, N> {
    type Item = S;
    type IntoIter = std::array::IntoIter<S, N>;

    fn into_iter(self) -> Self::IntoIter {
        self.children.into_iter()
    }
}

// MARK: PartialEq

impl<S: Source<T> + PartialEq, T: Float, const N: usize> PartialEq for SourceArray<S, T, N> {
    fn eq(&self, other: &Self) -> bool {
        if self.position() != other.position() || self.orientation() != other.orientation() {
            return false;
        }

        // Order-independent comparison
        let mut matched = [false; N];
        for source in &self.children {
            let found = other
                .children
                .iter()
                .enumerate()
                .find(|(idx, other_source)| !matched[*idx] && *source == **other_source);

            match found {
                Some((idx, _)) => matched[idx] = true,
                None => return false,
            }
        }
        true
    }
}

// MARK: Display

impl<S: Source<T>, T: Float, const N: usize> Display for SourceArray<S, T, N> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(
            f,
            "Source Array ({} children) at {}",
            self.children.len(),
            self.pose()
        )?;

        crate::collections::utils::write_tree(f, &self.children, "")
    }
}

// MARK: Test Display

#[cfg(test)]
mod display_tests {
    use super::*;
    use crate::{magnets::*, testing_util::*};

    #[test]
    fn test_source_array_display() {
        let m1 = CylinderMagnet::default().with_polarization([1.0, 2.0, 3.0]);
        let m2 = CylinderMagnet::default()
            .with_diameter(0.1)
            .with_height(0.3);
        let m3 = CylinderMagnet::default()
            .with_position([4.0, 5.0, 6.0])
            .with_orientation(UnitQuaternion::from_scaled_axis(
                [std::f64::consts::FRAC_PI_2, 0.0, 0.0].into(),
            ));

        let collection = sources!(m1, m2, m3);

        assert_eq!(
            "Source Array (3 children) at pos=[0.0, 0.0, 0.0], rot=[0.0, 0.0, 0.0]
 ├── 0: CylinderMagnet (pol=[1, 2, 3], d=1, h=1) at pos=[0.0, 0.0, 0.0], rot=[0.0, 0.0, 0.0]
 ├── 1: CylinderMagnet (pol=[0, 0, 1], d=0.1, h=0.3) at pos=[0.0, 0.0, 0.0], rot=[0.0, 0.0, 0.0]
 └── 2: CylinderMagnet (pol=[0, 0, 1], d=1, h=1) at pos=[4.0, 5.0, 6.0], rot=[<float>, 0.0, 0.0]",
            mask_long_floats(&format!("{}", collection))
        )
    }
}

// MARK: Test Field

#[cfg(test)]
mod fielde_tests {
    use std::f64::consts::PI;

    use super::*;
    use crate::{magnets::*, testing_util::*};
    use nalgebra::{Translation3, point};

    fn array() -> SourceArray<CylinderMagnet, f64, 3> {
        let m1 = CylinderMagnet::new(
            [0.0094, 0.0, -0.006],
            UnitQuaternion::from_scaled_axis([1.2092, 1.2092, 1.2092].into()),
            [1.0, 2.0, 3.0],
            3e-3,
            4e-3,
        );
        let m2 = CylinderMagnet::new(
            [-0.0047, 0.0081, -0.006],
            UnitQuaternion::from_scaled_axis([1.5316, 0.4104, 0.4104].into()),
            [0.4, 0.5, 0.6],
            4e-3,
            5e-3,
        );
        let m3 = CylinderMagnet::new(
            [-0.0047, -0.0081, -0.006],
            UnitQuaternion::from_scaled_axis([1.5316, -0.4104, -0.4104].into()),
            [0.9, 0.8, 0.6],
            5e-3,
            6e-3,
        );
        SourceArray::from([m1, m2, m3])
    }

    #[test]
    fn test_static() {
        let arr = array();
        test_B_magnet!(@small, &arr, "cylinder-collection.csv", 5e-9);
    }

    #[test]
    fn test_translate() {
        let mut arr = array();
        let translation = Translation3::new(0.01, 0.015, 0.02);
        arr.translate(translation);
        test_B_magnet!(@small, &arr, "cylinder-collection-translate.csv", 1e-8);

        arr.translate(translation.inverse());
        arr.set_position([0.01, 0.015, 0.02]);
        test_B_magnet!(@small, &arr, "cylinder-collection-translate.csv", 1e-8);
    }

    #[test]
    fn test_rotate() {
        let mut arr = array();
        let rotation = UnitQuaternion::from_scaled_axis([PI / 3.0, PI / 4.0, PI / 5.0].into());
        arr.rotate(rotation);
        test_B_magnet!(@small, &arr, "cylinder-collection-rotate.csv", 1e-9);

        arr.rotate(rotation.inverse());
        arr.set_orientation(rotation);
        test_B_magnet!(@small, &arr, "cylinder-collection-rotate.csv", 5e-8);

        arr.set_position(point![0.01, 0.015, 0.02]);
        test_B_magnet!(@small, &arr, "cylinder-collection-translate-rotate.csv", 5e-8);
    }
}
