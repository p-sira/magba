/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use core::ops::{Index, IndexMut};
use std::fmt::Display;

use nalgebra::{Point3, Translation3, UnitQuaternion, Vector3};

use crate::{
    SourceArray,
    base::*,
    collections::{component::Component, utils::impl_group_compute_B},
    geometry::Pose,
    transform::{impl_group_transform, impl_transform},
};

// MARK: Base

/// Heap-allocated data structure for grouping [Component].
///
/// ### Examples
///
/// ```
/// # use magba::*;
/// let cylinder = CylinderMagnet::default();
/// let cuboid = CuboidMagnet::default();
/// let dipole = Dipole::default();
///
/// let collection: Collection = collection!(cylinder, cuboid, dipole);
/// ```
#[derive(Debug, Clone)]
pub struct Collection<T: Float = f64> {
    pose: Pose<T>,
    children: Vec<Component<T>>,
    offsets: Vec<Pose<T>>,
}

impl<T: Float> Collection<T> {
    /// Initialize a new collection, keeping the components' coordinates as GLOBAL.
    pub fn new(
        position: Point3<T>,
        orientation: UnitQuaternion<T>,
        components: impl IntoIterator<Item = impl Into<Component<T>>>,
    ) -> Self {
        let pose = Pose::new(position, orientation);
        let pose_inv = pose.as_isometry().inverse();

        let children: Vec<Component<T>> = components.into_iter().map(|c| c.into()).collect();

        let offsets = children
            .iter()
            .map(|c| (pose_inv * c.pose().as_isometry()).into())
            .collect();

        Self {
            pose,
            children,
            offsets,
        }
    }

    /// ```
    /// # use magba::*;
    /// let cylinder = CylinderMagnet::default();
    /// let cuboid = CuboidMagnet::default();
    /// let dipole = Dipole::default();
    ///
    /// let components: [Component; _] = [cylinder.into(), cuboid.into(), dipole.into()];
    /// let collection = Collection::from(components.clone());
    /// collection.iter().enumerate().for_each(|(i, component)| assert_eq!(*component, components[i]));
    /// ```
    pub fn iter(&self) -> std::slice::Iter<'_, Component<T>> {
        self.children.iter()
    }
}

impl<T: Float> Default for Collection<T> {
    fn default() -> Self {
        Self {
            pose: Pose::default(),
            children: Vec::new(),
            offsets: Vec::new(),
        }
    }
}

// MARK: With builders

impl<T: Float> Collection<T> {
    pub fn with(mut self, component: impl Into<Component<T>>) -> Self {
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

// MARK: Macro

#[macro_export]
macro_rules! collection {
    // collection!(magnet1, magnet2, ...)
    ($($items:expr),* $(,)?) => {{
        let c: [Component<_>; _] = [$($items.into()),*];
        Collection::from(c)
    }};
    () => { Collection::default() };
}

// MARK: From, Into

impl<T: Float> FromIterator<Component<T>> for Collection<T> {
    fn from_iter<I: IntoIterator<Item = Component<T>>>(iter: I) -> Self {
        let children: Vec<Component<T>> = iter.into_iter().collect();
        let offsets: Vec<Pose<T>> = children.iter().map(|c| *c.pose()).collect();

        Self {
            pose: Pose::default(),
            children,
            offsets,
        }
    }
}

impl<T: Float, I: Into<Component<T>>, const N: usize> From<[I; N]> for Collection<T> {
    fn from(components: [I; N]) -> Self {
        components.into_iter().map(Into::into).collect()
    }
}

impl<T: Float, I: Into<Component<T>>> From<Vec<I>> for Collection<T> {
    fn from(components: Vec<I>) -> Self {
        components.into_iter().map(Into::into).collect()
    }
}

impl<T: Float> From<&[Component<T>]> for Collection<T> {
    fn from(components: &[Component<T>]) -> Self {
        components.iter().cloned().collect()
    }
}

impl<'a, T: Float> IntoIterator for &'a Collection<T> {
    type Item = &'a Component<T>;
    type IntoIter = std::slice::Iter<'a, Component<T>>;

    fn into_iter(self) -> Self::IntoIter {
        self.children.iter()
    }
}

impl<S, T: Float, const N: usize> From<SourceArray<S, T, N>> for Collection<T>
where
    S: Source<T> + Into<Component<T>>,
{
    fn from(array: SourceArray<S, T, N>) -> Self {
        Collection::new(array.position(), array.orientation(), array)
    }
}

// MARK: Extend, Push

impl<T: Float> Collection<T> {
    pub fn push(&mut self, component: impl Into<Component<T>>) {
        let component: Component<T> = component.into();
        let relative_pose = self.pose.as_isometry().inverse() * component.pose().as_isometry();

        self.offsets.push(relative_pose.into());
        self.children.push(component);
    }
}

impl<T: Float> Extend<Component<T>> for Collection<T> {
    fn extend<I: IntoIterator<Item = Component<T>>>(&mut self, iter: I) {
        for component in iter {
            self.push(component);
        }
    }
}

// MARK: Index

impl<T: Float> Index<usize> for Collection<T> {
    type Output = Component<T>;

    fn index(&self, index: usize) -> &Self::Output {
        &self.children[index]
    }
}

impl<T: Float> IndexMut<usize> for Collection<T> {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.children[index]
    }
}

// MARK: Transform

impl_transform!(Collection<T> where T: Float);
impl_group_transform!(Collection<T> where T: Float);

// MARK: Source

impl<T: Float> Source<T> for Collection<T> {
    impl_group_compute_B!();

    // MARK: Display

    fn format(&self, f: &mut std::fmt::Formatter<'_>, indent: &str) -> std::fmt::Result {
        writeln!(
            f,
            "Collection ({} children) at {}",
            self.children.len(),
            self.pose()
        )?;

        crate::collections::utils::write_tree(f, &self.children, indent)
    }
}

impl<T: Float> Display for Collection<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.format(f, "")
    }
}

// MARK: PartialEq

impl<T: Float> PartialEq for Collection<T> {
    fn eq(&self, other: &Self) -> bool {
        if self.position() != other.position()
            || self.orientation() != other.orientation()
            || self.children.len() != other.children.len()
        {
            return false;
        }

        let mut matched = vec![false; other.children.len()];
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

// MARK: Test Display

#[cfg(test)]
mod display_tests {
    use std::f64::consts::FRAC_PI_2;

    use super::*;
    use crate::{magnets::*, testing_util::*};

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
        let mut nested_collection = collection!(dipole.clone(), dipole);
        let deep_collection = collection!(CuboidMagnet::default());
        nested_collection.push(deep_collection);

        let collection = collection!(cylinder, nested_collection, cuboid);

        assert_eq!(
            "Collection (3 children) at pos=[0.0, 0.0, 0.0], rot=[0.0, 0.0, 0.0]
 ├── 0: CylinderMagnet (pol=[1.0, 2.0, 3.0], d=0.1, h=0.3) at pos=[0.0, 0.0, 0.0], rot=[0.0, 0.0, 0.0]
 ├── 1: Collection (3 children) at pos=[0.0, 0.0, 0.0], rot=[0.0, 0.0, 0.0]
 │   ├── 0: Dipole (m=[0.0, 0.0, 1.0]) at pos=[0.0, 0.0, 0.0], rot=[0.0, 0.0, 0.0]
 │   ├── 1: Dipole (m=[0.0, 0.0, 1.0]) at pos=[0.0, 0.0, 0.0], rot=[0.0, 0.0, 0.0]
 │   └── 2: Collection (1 children) at pos=[0.0, 0.0, 0.0], rot=[0.0, 0.0, 0.0]
 │       └── 0: CuboidMagnet (pol=[0.0, 0.0, 1.0], dim=[1.0, 1.0, 1.0]) at pos=[0.0, 0.0, 0.0], rot=[0.0, 0.0, 0.0]
 └── 2: CuboidMagnet (pol=[0.0, 0.0, 1.0], dim=[1.0, 1.0, 1.0]) at pos=[4.0, 5.0, 6.0], rot=[<float>, 0.0, 0.0]",
            mask_long_floats(&format!("{}", collection))
        )
    }
}

// MARK: Test PartialEq

#[cfg(test)]
mod partial_eq_tests {
    use std::f64::consts::FRAC_PI_2;

    use super::*;
    use crate::magnets::*;

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
        let c = collection!(magnet1(), magnet2());
        assert_eq!(c, c.clone());
    }

    #[test]
    fn test_different_transformation() {
        let c1 = collection!().with(magnet1());
        let c2 = collection!(magnet1()).with_position([1.0, 0.0, 0.0]);
        let c3 = collection!(magnet1())
            .with_orientation(UnitQuaternion::from_quaternion([1.0, 0.0, 0.0, 0.0].into()));
        assert_ne!(c1, c2);
        assert_ne!(c1, c3);
    }

    #[test]
    fn test_different_children() {
        let c1 = collection!(magnet1());
        let c2 = collection!(magnet2());
        let c3 = collection!(magnet1(), magnet2());
        assert_ne!(c1, c2);
        assert_ne!(c1, c3);
    }

    #[test]
    fn test_different_types() {
        let c1 = collection!(magnet1(), magnet2());
        let c2 = collection!(magnet1(), CuboidMagnet::default());
        assert_ne!(c1, c2);
    }

    #[test]
    fn test_order_independent() {
        let c1 = collection!(magnet1(), magnet2());
        let c2 = collection!(magnet2(), magnet1());
        assert_eq!(c1, c2);
    }

    #[test]
    fn test_duplicates() {
        let c1 = collection!(magnet1(), magnet1());
        let c2 = collection!(magnet1(), magnet2());
        let mut c3 = collection!(magnet1(), magnet1());
        assert_ne!(c1, c2);
        assert_eq!(c1, c3);
        c3.push(magnet2());
        assert_ne!(c1, c3);
        assert_ne!(c2, c3);
    }

    #[test]
    fn test_empty() {
        let c1: Collection = collection!();
        let c2: Collection = collection!();
        let c3 = collection!(magnet1());
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

    fn collection() -> Collection {
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
        Collection::from([m1, m2, m3])
    }

    #[test]
    fn test_static() {
        let collection = collection();
        test_B_magnet!(@small, &collection, "cuboid-collection.csv", 2e-13);
    }

    #[test]
    fn test_translate() {
        let mut collection = collection();
        let translation = Translation3::new(0.01, 0.015, 0.02);
        collection.translate(translation);
        test_B_magnet!(@small, &collection, "cuboid-collection-translate.csv", 2e-13);

        collection.translate(translation.inverse());
        collection.set_position([0.01, 0.015, 0.02]);
        test_B_magnet!(@small, &collection, "cuboid-collection-translate.csv", 2e-13);
    }

    #[test]
    fn test_collection_rotate() {
        let mut collection = collection();
        let rotation = UnitQuaternion::from_scaled_axis([PI / 3.0, PI / 4.0, PI / 5.0].into());
        collection.rotate(rotation);
        test_B_magnet!(@small, &collection, "cuboid-collection-rotate.csv", 2e-13);

        collection.rotate(rotation.inverse());
        collection.set_orientation(rotation);
        test_B_magnet!(@small, &collection, "cuboid-collection-rotate.csv", 2e-13);

        collection.set_position([0.01, 0.015, 0.02]);
        test_B_magnet!(@small, &collection, "cuboid-collection-translate-rotate.csv", 2e-13);
    }
}

#[cfg(test)]
mod heterogeneous_collection_tests {
    use std::f64::consts::{FRAC_PI_2, FRAC_PI_3, PI};

    use super::*;
    use crate::{magnets::*, testing_util::*};

    fn collection() -> Collection {
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
        collection!(m1, m2, m3)
    }

    #[test]
    fn test_static() {
        let collection = collection();
        test_B_magnet!(@small, &collection, "multi-collection.csv", 1e-10);
    }

    #[test]
    fn test_translate() {
        let mut collection = collection();
        let translation = Translation3::new(0.01, 0.015, 0.02);
        collection.translate(translation);
        test_B_magnet!(@small, &collection, "multi-collection-translate.csv", 5e-10);

        collection.translate(translation.inverse());
        collection.set_position([0.01, 0.015, 0.02]);
        test_B_magnet!(@small, &collection, "multi-collection-translate.csv", 5e-10);
    }

    #[test]
    fn test_rotate() {
        let mut collection = collection();
        let rotation = UnitQuaternion::from_scaled_axis([PI / 3.0, PI / 4.0, PI / 5.0].into());
        collection.rotate(rotation);
        test_B_magnet!(@small, &collection, "multi-collection-rotate.csv", 2e-10);

        collection.rotate(rotation.inverse());
        collection.set_orientation(rotation);
        test_B_magnet!(@small, &collection, "multi-collection-rotate.csv", 2e-10);

        collection.set_position([0.01, 0.015, 0.02]);
        test_B_magnet!(@small, &collection, "multi-collection-translate-rotate.csv", 2e-10);
    }
}
