/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use core::fmt::Display;
use core::ops::{Index, IndexMut};

use nalgebra::{Point3, Translation3, UnitQuaternion, Vector3};

use crate::{
    base::{
        Float, Pose, Source, Transform,
        transform::impl_group_transform,
    },
    collections::{node::Node, utils::impl_group_compute_B},
};

/// Stack-allocated data structure for grouping [Source].
///
/// # Examples
///
/// Grouping the same type of magnets:
///
/// ```
/// # use magba::sources;
/// # use magba::prelude::*;
/// let cuboid1 = CuboidMagnet::default();
/// let cuboid2 = cuboid1.clone().with_position([0.0, 1.0, 0.0]);
///
/// let source_array = sources!([cuboid1, cuboid2]);
/// ```
///
/// Grouping different types of magnets using [Magnet](crate::magnets::Magnet) as a wrapper:
///
/// ```
/// # use magba::sources;
/// # use magba::prelude::*;
/// let cylinder: Magnet = CylinderMagnet::default().into();
/// let dipole: Magnet = Dipole::default().into();
///
/// let source_array = sources!([cylinder, dipole]);
/// ```
#[derive(Debug, Clone)]
pub struct SourceArray<S: Source<T>, const N: usize, T: Float = f64> {
    pose: Pose<T>,
    nodes: [Node<S, T>; N],
}

impl<S: Source<T>, const N: usize, T: Float> SourceArray<S, N, T> {
    pub fn new(
        position: impl Into<Point3<T>>,
        orientation: UnitQuaternion<T>,
        sources: [S; N],
    ) -> Self {
        let pose = Pose::new(position.into(), orientation);
        let pose_inv = pose.as_isometry().inverse();

        let mut into_iter = sources.into_iter();
        let nodes = core::array::from_fn(|_| {
            let component = into_iter.next().unwrap();
            let local_offset = (pose_inv * component.pose().as_isometry()).into();
            Node::new(component, local_offset)
        });

        Self { pose, nodes }
    }

    pub fn components(&self) -> impl Iterator<Item = &S> {
        self.nodes.iter().map(Node::component)
    }

    pub fn iter(&self) -> impl Iterator<Item = &S> {
        self.components()
    }
}

impl<S: Source<T> + Default, T: Float, const N: usize> Default for SourceArray<S, N, T> {
    fn default() -> Self {
        Self {
            pose: Pose::default(),
            nodes: core::array::from_fn(|_| Node::default()),
        }
    }
}

// MARK: Transform

impl_group_transform!(SourceArray<S, N, T> where S: Source<T>, const N: usize, T: Float);

// MARK: Source

impl<S: Source<T> + Clone, T: Float, const N: usize> Source<T> for SourceArray<S, N, T> {
    impl_group_compute_B!();
}

// MARK: Index

impl<S: Source<T>, const N: usize, T: Float> Index<usize> for SourceArray<S, N, T> {
    type Output = S;

    fn index(&self, index: usize) -> &Self::Output {
        self.nodes[index].component()
    }
}

impl<S: Source<T>, const N: usize, T: Float> IndexMut<usize> for SourceArray<S, N, T> {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        self.nodes[index].component_mut()
    }
}

// MARK: From, Into

impl<S: Source<T>, const N: usize, T: Float> From<[S; N]> for SourceArray<S, N, T> {
    fn from(sources: [S; N]) -> Self {
        let mut into_iter = sources.into_iter();
        let nodes = core::array::from_fn(|_| {
            let component = into_iter.next().unwrap();
            let local_offset = *component.pose();
            Node::new(component, local_offset)
        });

        Self {
            pose: Pose::default(),
            nodes,
        }
    }
}

impl<S: Source<T>, const N: usize, T: Float> FromIterator<S> for SourceArray<S, N, T> {
    fn from_iter<I: IntoIterator<Item = S>>(iter: I) -> Self {
        let mut iter = iter.into_iter();
        let sources = core::array::from_fn(|_| {
            iter.next()
                .expect("SourceArray::from_iter: iterator yielded fewer than N items")
        });

        if iter.next().is_some() {
            panic!("SourceArray::from_iter: iterator yielded more than N items");
        }

        Self::from(sources)
    }
}

impl<'a, S: Source<T>, const N: usize, T: Float> IntoIterator for &'a SourceArray<S, N, T> {
    type Item = &'a S;
    type IntoIter = std::iter::Map<std::slice::Iter<'a, Node<S, T>>, fn(&'a Node<S, T>) -> &'a S>;

    fn into_iter(self) -> Self::IntoIter {
        self.nodes.iter().map(Node::component)
    }
}

impl<S: Source<T>, const N: usize, T: Float> IntoIterator for SourceArray<S, N, T> {
    type Item = S;
    type IntoIter = std::iter::Map<std::array::IntoIter<Node<S, T>, N>, fn(Node<S, T>) -> S>;

    fn into_iter(self) -> Self::IntoIter {
        self.nodes.into_iter().map(Node::into_component)
    }
}

// MARK: PartialEq

impl<S: Source<T> + PartialEq, T: Float, const N: usize> PartialEq for SourceArray<S, N, T> {
    fn eq(&self, other: &Self) -> bool {
        if self.position() != other.position() || self.orientation() != other.orientation() {
            return false;
        }

        // Order-independent comparison
        let mut matched = [false; N];
        for node in &self.nodes {
            let found =
                other.nodes.iter().enumerate().find(|(idx, other_node)| {
                    !matched[*idx] && node.component() == other_node.component()
                });

            match found {
                Some((idx, _)) => matched[idx] = true,
                None => return false,
            }
        }
        true
    }
}

// MARK: Display

impl<S: Source<T>, const N: usize, T: Float> Display for SourceArray<S, N, T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(
            f,
            "SourceArray ({} children) at {}",
            self.nodes.len(),
            self.pose()
        )?;

        crate::collections::utils::write_tree(f, self.components(), "", |leaf, f, ind| {
            leaf.format(f, ind)
        })
    }
}

// MARK: Test Display

#[cfg(test)]
mod display_tests {
    use super::*;
    use crate::{collections::sources, magnets::*, testing_util::*};

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

        let sources = sources!([m1, m2, m3]);

        assert_eq!(
            "SourceArray (3 children) at pos=[0.0, 0.0, 0.0], rot=[0.0, 0.0, 0.0]
 ├── 0: CylinderMagnet (pol=[1.0, 2.0, 3.0], d=1.0, h=1.0) at pos=[0.0, 0.0, 0.0], rot=[0.0, 0.0, 0.0]
 ├── 1: CylinderMagnet (pol=[0.0, 0.0, 1.0], d=0.1, h=0.3) at pos=[0.0, 0.0, 0.0], rot=[0.0, 0.0, 0.0]
 └── 2: CylinderMagnet (pol=[0.0, 0.0, 1.0], d=1.0, h=1.0) at pos=[4.0, 5.0, 6.0], rot=[<float>, 0.0, 0.0]",
            mask_long_floats(&format!("{}", sources))
        )
    }
}

// MARK: Test Field

#[cfg(not(feature = "test_force_fail"))]
#[cfg(test)]
mod field_tests {
    use std::f64::consts::PI;

    use super::*;
    use crate::{collections::SourceAssembly, magnets::*, testing_util::*};
    use nalgebra::{Translation3, point};

    fn array() -> SourceArray<CylinderMagnet, 3, f64> {
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
    fn child_mutation_updates_local_offset_before_parent_transform() {
        let mut array = SourceArray::from([
            Dipole::default().with_position([1.0, 0.0, 0.0]),
            Dipole::default().with_position([2.0, 0.0, 0.0]),
        ]);
        let untouched_offset = *array.nodes[1].local_offset();

        array[0].set_position([5.0, 0.0, 0.0]);
        array[1].set_moment([1.0, 0.0, 0.0]);
        assert!(array.nodes[0].is_dirty());
        assert!(array.nodes[1].is_dirty());

        array.translate([10.0, 0.0, 0.0]);

        assert_eq!(array[0].position(), point![15.0, 0.0, 0.0]);
        assert_eq!(array[1].position(), point![12.0, 0.0, 0.0]);
        assert_eq!(*array.nodes[1].local_offset(), untouched_offset);
        assert!(!array.nodes[0].is_dirty());
    }

    #[test]
    fn trait_set_pose_propagates_to_children() {
        let mut array = SourceArray::from([Dipole::default().with_position([1.0, 0.0, 0.0])]);

        Transform::set_pose(
            &mut array,
            Pose::new([10.0, 0.0, 0.0], UnitQuaternion::identity()),
        );

        assert_eq!(array[0].position(), point![11.0, 0.0, 0.0]);
    }

    #[test]
    fn transform_propagates_through_nested_collections() {
        let inner = SourceAssembly::from([Dipole::default().with_position([1.0, 0.0, 0.0])]);
        let mut outer = SourceArray::from([inner]);

        outer.translate([10.0, 0.0, 0.0]);

        assert_eq!(outer[0][0].pose().position(), point![11.0, 0.0, 0.0]);
    }

    #[test]
    fn trait_pose_mut_accessible() {
        let mut array = SourceArray::from([Dipole::default().with_position([1.0, 0.0, 0.0])]);
        let pose = Transform::pose_mut(&mut array);
        assert_eq!(pose.position(), point![0.0, 0.0, 0.0]);
    }

    #[test]
    fn test_into_iterator_ref() {
        let arr = SourceArray::from([
            Dipole::default().with_position([1.0, 0.0, 0.0]),
            Dipole::default().with_position([2.0, 0.0, 0.0]),
        ]);
        let mut count = 0;
        for s in &arr {
            count += 1;
            assert!(s.position().x > 0.0);
        }
        assert_eq!(count, 2);
    }

    #[test]
    fn test_partial_eq() {
        let arr1 = SourceArray::from([
            Dipole::default().with_position([1.0, 0.0, 0.0]),
            Dipole::default().with_position([2.0, 0.0, 0.0]),
        ]);
        let arr2 = SourceArray::from([
            Dipole::default().with_position([2.0, 0.0, 0.0]),
            Dipole::default().with_position([1.0, 0.0, 0.0]),
        ]);
        let arr3 = SourceArray::from([
            Dipole::default().with_position([3.0, 0.0, 0.0]),
            Dipole::default().with_position([1.0, 0.0, 0.0]),
        ]);
        assert_eq!(arr1, arr2);
        assert_ne!(arr1, arr3);
    }

    #[test]
    fn test_iter() {
        let arr = SourceArray::from([Dipole::<f64>::default()]);
        assert_eq!(arr.iter().count(), 1);
    }

    #[test]
    fn test_default_and_from_iter() {
        let def: SourceArray<Dipole<f64>, 2> = SourceArray::default();
        assert_eq!(def.components().count(), 2);

        let iter_arr: SourceArray<Dipole<f64>, 2> = vec![Dipole::default(), Dipole::default()].into_iter().collect();
        assert_eq!(iter_arr.components().count(), 2);

        let mut diff_pos = iter_arr.clone();
        diff_pos.set_position([10.0, 0.0, 0.0]);
        assert_ne!(iter_arr, diff_pos);
    }

    #[test]
    fn test_rotate_anchor() {
        let mut arr = SourceArray::from([Dipole::default().with_position([1.0, 0.0, 0.0])]);
        let rot = UnitQuaternion::from_scaled_axis([0.0, 0.0, core::f64::consts::PI].into());
        arr.rotate_anchor(rot, [1.0, 0.0, 0.0]);
        assert_eq!(arr.position().x, 2.0);
    }

    #[test]
    #[should_panic(expected = "fewer than N items")]
    fn test_from_iter_too_few() {
        let _: SourceArray<Dipole<f64>, 2> = vec![Dipole::default()].into_iter().collect();
    }

    #[test]
    #[should_panic(expected = "more than N items")]
    fn test_from_iter_too_many() {
        let _: SourceArray<Dipole<f64>, 2> = vec![Dipole::default(), Dipole::default(), Dipole::default()].into_iter().collect();
    }

    #[test]
    #[allow(non_snake_case)]
    fn test_compute_B_single() {
        let arr = SourceArray::from([Dipole::<f64>::default()]);
        let b = Source::compute_B(&arr, point![0.0, 0.0, 0.05]);
        assert!(b.norm() > 0.0);
    }

    #[test]
    fn test_static() {
        let arr = array();
        test_B_magnet!(@small, &arr, "cylinder-sources.csv", 5e-9);
    }

    #[test]
    fn test_translate() {
        let mut arr = array();
        let translation = Translation3::new(0.01, 0.015, 0.02);
        arr.translate(translation);
        test_B_magnet!(@small, &arr, "cylinder-sources-translate.csv", 1e-8);

        arr.translate(translation.inverse());
        arr.set_position([0.01, 0.015, 0.02]);
        test_B_magnet!(@small, &arr, "cylinder-sources-translate.csv", 1e-8);
    }

    #[test]
    fn test_rotate() {
        let mut arr = array();
        let rotation = UnitQuaternion::from_scaled_axis([PI / 3.0, PI / 4.0, PI / 5.0].into());
        arr.rotate(rotation);
        test_B_magnet!(@small, &arr, "cylinder-sources-rotate.csv", 1e-9);

        arr.rotate(rotation.inverse());
        arr.set_orientation(rotation);
        test_B_magnet!(@small, &arr, "cylinder-sources-rotate.csv", 5e-8);

        arr.set_position(point![0.01, 0.015, 0.02]);
        test_B_magnet!(@small, &arr, "cylinder-sources-translate-rotate.csv", 5e-8);
    }
}
