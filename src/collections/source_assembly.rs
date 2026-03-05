/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use core::ops::{Index, IndexMut};
use std::fmt::Display;

use nalgebra::{Point3, Translation3, UnitQuaternion, Vector3};

use crate::{
    base::transform::{impl_group_transform, impl_transform},
    base::{Float, Pose, Source, Transform},
    collections::{
        SourceArray, node::Node, source_component::SourceComponent, utils::impl_group_compute_B,
    },
};

// MARK: Base

/// Heap-allocated data structure for grouping [SourceComponent].
///
/// ### Examples
///
/// ```
/// # use magba::sources;
/// # use magba::prelude::*;
/// let cylinder = CylinderMagnet::default();
/// let cuboid = CuboidMagnet::default();
/// let dipole = Dipole::default();
///
/// let sources: SourceAssembly = sources!(cylinder, cuboid, dipole);
/// ```
#[derive(Debug, Clone)]
pub struct SourceAssembly<T: Float = f64> {
    pose: Pose<T>,
    nodes: Vec<Node<SourceComponent<T>, T>>,
}

impl<T: Float> SourceAssembly<T> {
    /// Constructs a [SourceAssembly], keeping the components' coordinates as GLOBAL.
    pub fn new(
        position: Point3<T>,
        orientation: UnitQuaternion<T>,
        components: impl IntoIterator<Item = impl Into<SourceComponent<T>>>,
    ) -> Self {
        let pose = Pose::new(position, orientation);
        let pose_inv = pose.as_isometry().inverse();

        let nodes = components
            .into_iter()
            .map(|c| {
                let component: SourceComponent<T> = c.into();
                let local_offset = (pose_inv * component.pose().as_isometry()).into();
                Node::new(component, local_offset)
            })
            .collect();

        Self { pose, nodes }
    }

    pub fn components(&self) -> impl Iterator<Item = &SourceComponent<T>> {
        self.nodes.iter().map(|n| &n.component)
    }

    /// ```
    /// # use magba::prelude::*;
    /// let cylinder = CylinderMagnet::default();
    /// let cuboid = CuboidMagnet::default();
    /// let dipole = Dipole::default();
    ///
    /// let components: [SourceComponent; _] = [cylinder.into(), cuboid.into(), dipole.into()];
    /// let sources = SourceAssembly::from(components.clone());
    /// sources.iter().enumerate().for_each(|(i, component)| assert_eq!(*component, components[i]));
    /// ```
    pub fn iter(&self) -> impl Iterator<Item = &SourceComponent<T>> {
        self.components()
    }
}

impl<T: Float> Default for SourceAssembly<T> {
    fn default() -> Self {
        Self {
            pose: Pose::default(),
            nodes: Vec::new(),
        }
    }
}

// MARK: With builders

impl<T: Float> SourceAssembly<T> {
    pub fn with(mut self, component: impl Into<SourceComponent<T>>) -> Self {
        self.push(component);
        self
    }

    pub fn with_position(mut self, position: impl Into<Translation3<T>>) -> Self {
        self.set_position(position);
        self
    }

    pub fn with_orientation(mut self, orientation: UnitQuaternion<T>) -> Self {
        self.set_orientation(orientation);
        self
    }

    pub fn with_pose(mut self, pose: impl Into<Pose<T>>) -> Self {
        self.set_pose(pose);
        self
    }
}

// MARK: From, Into

impl<T: Float> FromIterator<SourceComponent<T>> for SourceAssembly<T> {
    fn from_iter<I: IntoIterator<Item = SourceComponent<T>>>(iter: I) -> Self {
        let nodes = iter
            .into_iter()
            .map(|c| {
                let local_offset = *c.pose();
                Node::new(c, local_offset)
            })
            .collect();

        Self {
            pose: Pose::default(),
            nodes,
        }
    }
}

impl<T: Float, I: Into<SourceComponent<T>>, const N: usize> From<[I; N]> for SourceAssembly<T> {
    fn from(components: [I; N]) -> Self {
        components.into_iter().map(Into::into).collect()
    }
}

impl<T: Float, I: Into<SourceComponent<T>>> From<Vec<I>> for SourceAssembly<T> {
    fn from(components: Vec<I>) -> Self {
        components.into_iter().map(Into::into).collect()
    }
}

impl<T: Float> From<&[SourceComponent<T>]> for SourceAssembly<T> {
    fn from(components: &[SourceComponent<T>]) -> Self {
        components.iter().cloned().collect()
    }
}

impl<'a, T: Float> IntoIterator for &'a SourceAssembly<T> {
    type Item = &'a SourceComponent<T>;
    type IntoIter = std::iter::Map<
        std::slice::Iter<'a, Node<SourceComponent<T>, T>>,
        fn(&'a Node<SourceComponent<T>, T>) -> &'a SourceComponent<T>,
    >;

    fn into_iter(self) -> Self::IntoIter {
        self.nodes.iter().map(|n| &n.component)
    }
}

impl<S, T: Float, const N: usize> From<SourceArray<S, N, T>> for SourceAssembly<T>
where
    S: Source<T> + Into<SourceComponent<T>>,
{
    fn from(array: SourceArray<S, N, T>) -> Self {
        SourceAssembly::new(array.position(), array.orientation(), array)
    }
}

// MARK: Extend, Push

impl<T: Float> SourceAssembly<T> {
    pub fn push(&mut self, component: impl Into<SourceComponent<T>>) {
        let component: SourceComponent<T> = component.into();
        let local_offset =
            (self.pose.as_isometry().inverse() * component.pose().as_isometry()).into();
        self.nodes.push(Node::new(component, local_offset));
    }
}

impl<T: Float> Extend<SourceComponent<T>> for SourceAssembly<T> {
    fn extend<I: IntoIterator<Item = SourceComponent<T>>>(&mut self, iter: I) {
        for component in iter {
            self.push(component);
        }
    }
}

// MARK: Index

impl<T: Float> Index<usize> for SourceAssembly<T> {
    type Output = SourceComponent<T>;

    fn index(&self, index: usize) -> &Self::Output {
        &self.nodes[index].component
    }
}

impl<T: Float> IndexMut<usize> for SourceAssembly<T> {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.nodes[index].component
    }
}

// MARK: Transform

impl_transform!(SourceAssembly<T> where T: Float);
impl_group_transform!(SourceAssembly<T> where T: Float);

// MARK: Source

impl<T: Float> Source<T> for SourceAssembly<T> {
    impl_group_compute_B!();

    // MARK: Display

    fn format(&self, f: &mut std::fmt::Formatter<'_>, indent: &str) -> std::fmt::Result {
        writeln!(
            f,
            "SourceAssembly ({} children) at {}",
            self.nodes.len(),
            self.pose()
        )?;

        crate::collections::utils::write_tree(f, self.components(), indent, |leaf, f, ind| {
            leaf.format(f, ind)
        })
    }
}

impl<T: Float> Display for SourceAssembly<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.format(f, "")
    }
}

// MARK: PartialEq

impl<T: Float> PartialEq for SourceAssembly<T> {
    fn eq(&self, other: &Self) -> bool {
        if self.position() != other.position()
            || self.orientation() != other.orientation()
            || self.nodes.len() != other.nodes.len()
        {
            return false;
        }

        let mut matched = vec![false; other.nodes.len()];
        for node in &self.nodes {
            let found =
                other.nodes.iter().enumerate().find(|(idx, other_node)| {
                    !matched[*idx] && node.component == other_node.component
                });

            match found {
                Some((idx, _)) => matched[idx] = true,
                None => return false,
            }
        }
        true
    }
}

// MARK: Test Display

#[cfg(test)]
mod display_tests {
    use std::f64::consts::FRAC_PI_2;

    use super::*;
    use crate::{collections::sources, magnets::*, testing_util::*};

    #[test]
    fn test_collection_display() {
        let cylinder = CylinderMagnet::default()
            .with_polarization([1.0, 2.0, 3.0])
            .with_diameter(0.1)
            .with_height(0.3);
        let cuboid = CuboidMagnet::default()
            .with_position([4.0, 5.0, 6.0])
            .with_orientation(UnitQuaternion::from_scaled_axis(
                [FRAC_PI_2, 0.0, 0.0].into(),
            ));

        let dipole: Dipole = Dipole::default();
        let mut nested_collection = sources!(dipole.clone(), dipole.clone());
        let deep_collection = sources!([CuboidMagnet::default()]);
        nested_collection.push(deep_collection);

        let sources = sources!(cylinder, nested_collection, cuboid);

        assert_eq!(
            "SourceAssembly (3 children) at pos=[0.0, 0.0, 0.0], rot=[0.0, 0.0, 0.0]
 ├── 0: CylinderMagnet (pol=[1.0, 2.0, 3.0], d=0.1, h=0.3) at pos=[0.0, 0.0, 0.0], rot=[0.0, 0.0, 0.0]
 ├── 1: SourceAssembly (3 children) at pos=[0.0, 0.0, 0.0], rot=[0.0, 0.0, 0.0]
 │   ├── 0: Dipole (m=[0.0, 0.0, 1.0]) at pos=[0.0, 0.0, 0.0], rot=[0.0, 0.0, 0.0]
 │   ├── 1: Dipole (m=[0.0, 0.0, 1.0]) at pos=[0.0, 0.0, 0.0], rot=[0.0, 0.0, 0.0]
 │   └── 2: SourceAssembly (1 children) at pos=[0.0, 0.0, 0.0], rot=[0.0, 0.0, 0.0]
 │       └── 0: CuboidMagnet (pol=[0.0, 0.0, 1.0], dim=[1.0, 1.0, 1.0]) at pos=[0.0, 0.0, 0.0], rot=[0.0, 0.0, 0.0]
 └── 2: CuboidMagnet (pol=[0.0, 0.0, 1.0], dim=[1.0, 1.0, 1.0]) at pos=[4.0, 5.0, 6.0], rot=[<float>, 0.0, 0.0]",
            mask_long_floats(&format!("{}", sources))
        )
    }
}

// MARK: Test PartialEq

#[cfg(test)]
mod partial_eq_tests {
    use std::f64::consts::FRAC_PI_2;

    use super::*;
    use crate::{collections::sources, magnets::*};

    fn magnet1() -> CylinderMagnet<f64> {
        CylinderMagnet::default()
            .with_position([1.0, 2.0, 3.0])
            .with_polarization([1.0, 2.0, 3.0])
    }

    fn magnet2() -> CylinderMagnet<f64> {
        CylinderMagnet::default()
            .with_orientation(UnitQuaternion::from_scaled_axis(
                [FRAC_PI_2, 0.0, 0.0].into(),
            ))
            .with_diameter(0.3)
            .with_height(0.4)
    }

    #[test]
    fn test_equal() {
        let c = sources!(magnet1(), magnet2());
        assert_eq!(c, c.clone());
    }

    #[test]
    fn test_different_transformation() {
        let c1 = sources!().with(magnet1());
        let c2 = sources!(magnet1()).with_position([1.0, 0.0, 0.0]);
        let c3 = sources!(magnet1())
            .with_orientation(UnitQuaternion::from_quaternion([1.0, 0.0, 0.0, 0.0].into()));
        assert_ne!(c1, c2);
        assert_ne!(c1, c3);
    }

    #[test]
    fn test_different_children() {
        let c1 = sources!(magnet1());
        let c2 = sources!(magnet2());
        let c3 = sources!(magnet1(), magnet2());
        assert_ne!(c1, c2);
        assert_ne!(c1, c3);
    }

    #[test]
    fn test_different_types() {
        let c1 = sources!(magnet1(), magnet2());
        let c2 = sources!(magnet1(), CuboidMagnet::default());
        assert_ne!(c1, c2);
    }

    #[test]
    fn test_order_independent() {
        let c1 = sources!(magnet1(), magnet2());
        let c2 = sources!(magnet2(), magnet1());
        assert_eq!(c1, c2);
    }

    #[test]
    fn test_duplicates() {
        let c1 = sources!(magnet1(), magnet1());
        let c2 = sources!(magnet1(), magnet2());
        let mut c3 = sources!(magnet1(), magnet1());
        assert_ne!(c1, c2);
        assert_eq!(c1, c3);
        c3.push(magnet2());
        assert_ne!(c1, c3);
        assert_ne!(c2, c3);
    }

    #[test]
    fn test_empty() {
        let c1: SourceAssembly = sources!();
        let c2: SourceAssembly = sources!();
        let c3 = sources!(magnet1());
        assert_eq!(c1, c2);
        assert_ne!(c1, c3);
    }
}

// MARK: Test Field

#[cfg(test)]
mod field_tests {
    use std::f64::consts::{FRAC_PI_3, PI};

    use super::*;
    use crate::{magnets::*, testing_util::*};
    use nalgebra::Translation3;

    fn assembly() -> SourceAssembly {
        let base_magnet = CuboidMagnet::default()
            .with_polarization([0.1, 0.2, 0.3])
            .with_dimensions([0.02, 0.02, 0.03]);
        let m1 = base_magnet.clone().with_position([0.005, 0.01, 0.015]);
        let m2 = base_magnet
            .clone()
            .with_position([0.015, 0.005, 0.01])
            .with_orientation(UnitQuaternion::from_scaled_axis(
                [0.0, FRAC_PI_3, 0.0].into(),
            ));
        let m3 = base_magnet
            .with_position([0.01, 0.015, 0.005])
            .with_orientation(UnitQuaternion::from_scaled_axis(
                [0.0, 0.0, FRAC_PI_3].into(),
            ));
        SourceAssembly::from([m1, m2, m3])
    }

    #[test]
    fn test_static() {
        let sources = assembly();
        test_B_magnet!(@small, &sources, "cuboid-sources.csv", 2e-13);
    }

    #[test]
    fn test_translate() {
        let mut sources = assembly();
        let translation = Translation3::new(0.01, 0.015, 0.02);
        sources.translate(translation);
        test_B_magnet!(@small, &sources, "cuboid-sources-translate.csv", 2e-13);

        sources.translate(translation.inverse());
        sources.set_position([0.01, 0.015, 0.02]);
        test_B_magnet!(@small, &sources, "cuboid-sources-translate.csv", 2e-13);
    }

    #[test]
    fn test_collection_rotate() {
        let mut sources = assembly();
        let rotation = UnitQuaternion::from_scaled_axis([PI / 3.0, PI / 4.0, PI / 5.0].into());
        sources.rotate(rotation);
        test_B_magnet!(@small, &sources, "cuboid-sources-rotate.csv", 2e-13);

        sources.rotate(rotation.inverse());
        sources.set_orientation(rotation);
        test_B_magnet!(@small, &sources, "cuboid-sources-rotate.csv", 2e-13);

        sources.set_position([0.01, 0.015, 0.02]);
        test_B_magnet!(@small, &sources, "cuboid-sources-translate-rotate.csv", 2e-13);
    }
}

#[cfg(test)]
mod heterogeneous_collection_tests {
    use std::f64::consts::{FRAC_PI_2, FRAC_PI_3, PI};

    use super::*;
    use crate::{collections::sources, magnets::*, testing_util::*};

    fn sources() -> SourceAssembly {
        let m1 = CylinderMagnet::new(
            [0.005, 0.01, 0.015],
            UnitQuaternion::identity(),
            [0.1, 0.2, 0.3],
            0.04,
            0.05,
        );
        let m2 = CuboidMagnet::new(
            [0.015, 0.005, 0.01],
            UnitQuaternion::from_scaled_axis([0.0, FRAC_PI_3, 0.0].into()),
            [0.1, 0.2, 0.3],
            [0.02, 0.02, 0.03],
        );
        let m3 = Dipole::default()
            .with_orientation(UnitQuaternion::from_scaled_axis(
                [0.0, FRAC_PI_2, FRAC_PI_2].into(),
            ))
            .with_moment([0.4, 0.5, 0.6]);
        sources!(m1, m2, m3)
    }

    #[test]
    fn test_static() {
        let sources = sources();
        test_B_magnet!(@small, &sources, "multi-sources.csv", 1e-10);
    }

    #[test]
    fn test_translate() {
        let mut sources = sources();
        let translation = Translation3::new(0.01, 0.015, 0.02);
        sources.translate(translation);
        test_B_magnet!(@small, &sources, "multi-sources-translate.csv", 5e-10);

        sources.translate(translation.inverse());
        sources.set_position([0.01, 0.015, 0.02]);
        test_B_magnet!(@small, &sources, "multi-sources-translate.csv", 5e-10);
    }

    #[test]
    fn test_rotate() {
        let mut sources = sources();
        let rotation = UnitQuaternion::from_scaled_axis([PI / 3.0, PI / 4.0, PI / 5.0].into());
        sources.rotate(rotation);
        test_B_magnet!(@small, &sources, "multi-sources-rotate.csv", 2e-10);

        sources.rotate(rotation.inverse());
        sources.set_orientation(rotation);
        test_B_magnet!(@small, &sources, "multi-sources-rotate.csv", 2e-10);

        sources.set_position([0.01, 0.015, 0.02]);
        test_B_magnet!(@small, &sources, "multi-sources-translate-rotate.csv", 2e-10);
    }
}
