/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use std::fmt::Display;

use nalgebra::{Point3, Translation3, UnitQuaternion, Vector3};

use crate::{
    base::{Field, Float, Source, Transform, transform::impl_transform},
    collections::component::Component,
    crate_util::write_tree,
    geometry::Pose,
    transform::impl_group_transform,
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
    /// let collection = Collection::from(components);
    /// collection.iter().enumerate().for_each(|(i, &component)| assert_eq!(component, components[i]));
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
        let children: Vec<Component<T>> = iter.into_iter().map(|c| c.into()).collect();
        let offsets: Vec<Pose<T>> = children.iter().map(|c| c.pose().clone()).collect();

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

impl<T: Float> From<Component<T>> for Collection<T> {
    fn from(component: Component<T>) -> Self {
        [component].into()
    }
}

impl<'a, T: Float> IntoIterator for &'a Collection<T> {
    type Item = &'a Component<T>;
    type IntoIter = std::slice::Iter<'a, Component<T>>;

    fn into_iter(self) -> Self::IntoIter {
        self.children.iter()
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

// MARK: Transform

impl_transform!(Collection<T> where T: Float);
impl_group_transform!(Collection<T> where T: Float);

// MARK: Field, Source

impl<T: Float> Field<T> for Collection<T> {
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

impl<T: Float> Source<T> for Collection<T> {}

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

// MARK: Display

impl<T: Float> Display for Collection<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(
            f,
            "Collection ({} children) at {}",
            self.children.len(),
            self.pose()
        )?;

        write_tree(f, &self.children)
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
        let m1 = CylinderMagnet::default()
            .with_polarization([1.0, 2.0, 3.0])
            .with_diameter(0.1)
            .with_height(0.3);
        let m2: Dipole = Dipole::default();
        let m3 = CuboidMagnet::default()
            .with_position([4.0, 5.0, 6.0])
            .with_orientation(UnitQuaternion::from_scaled_axis(
                [FRAC_PI_2, 0.0, 0.0].into(),
            ));

        let collection = collection!(m1, m2, m3);

        assert_eq!(
            "Collection (3 children) at pos=[0.0, 0.0, 0.0], r=[0.0, 0.0, 0.0]
├── 0: CylinderMagnet (pol=[1, 2, 3], d=0.1, h=0.3) at pos=[0.0, 0.0, 0.0], r=[0.0, 0.0, 0.0]
├── 1: Dipole (m=[0, 0, 1]) at pos=[0.0, 0.0, 0.0], r=[0.0, 0.0, 0.0]
└── 2: CuboidMagnet (pol=[0, 0, 1], dim=[1, 1, 1]) at pos=[4.0, 5.0, 6.0], r=[<float>, 0.0, 0.0]",
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
mod cylinder_collection_tests {
    use std::f64::consts::PI;

    use super::*;
    use crate::{magnets::*, testing_util::*};
    use nalgebra::{Translation3, point};

    fn get_collection() -> Collection<f64> {
        let mut collection = Collection::default();
        collection.push(CylinderMagnet::new(
            [0.0094, 0.0, -0.006],
            quat_from_rotvec(1.2092, 1.2092, 1.2092),
            [1.0, 2.0, 3.0],
            3e-3,
            4e-3,
        ));
        collection.push(CylinderMagnet::new(
            [-0.0047, 0.0081, -0.006],
            quat_from_rotvec(1.5316, 0.4104, 0.4104),
            [0.4, 0.5, 0.6],
            4e-3,
            5e-3,
        ));
        collection.push(CylinderMagnet::new(
            [-0.0047, -0.0081, -0.006],
            quat_from_rotvec(1.5316, -0.4104, -0.4104),
            [0.9, 0.8, 0.6],
            5e-3,
            6e-3,
        ));
        collection
    }

    #[test]
    fn test_collection() {
        let collection = get_collection();
        test_B_magnet!(@small, &collection, "cylinder-collection.csv", 5e-9);
    }

    #[test]
    fn test_collection_translate() {
        let mut collection = get_collection();
        let translation = Translation3::new(0.01, 0.015, 0.02);
        collection.translate(translation);
        test_B_magnet!(@small, &collection, "cylinder-collection-translate.csv", 1e-8);

        collection.translate(translation.inverse());
        collection.set_position(point![0.01, 0.015, 0.02]);
        test_B_magnet!(@small, &collection, "cylinder-collection-translate.csv", 1e-8);
    }

    #[test]
    fn test_collection_rotate() {
        let mut collection = get_collection();
        let rotation = quat_from_rotvec(PI / 3.0, PI / 4.0, PI / 5.0);
        collection.rotate(rotation);
        test_B_magnet!(@small, &collection, "cylinder-collection-rotate.csv", 1e-9);

        collection.rotate(rotation.inverse());
        collection.set_orientation(rotation);
        test_B_magnet!(@small, &collection, "cylinder-collection-rotate.csv", 5e-8);

        collection.set_position(point![0.01, 0.015, 0.02]);
        test_B_magnet!(@small, &collection, "cylinder-collection-translate-rotate.csv", 5e-8);
    }
}

#[cfg(test)]
mod cuboid_collection_tests {
    use std::f64::consts::{FRAC_PI_3, PI};

    use super::*;
    use crate::{magnets::*, testing_util::*};
    use nalgebra::{Translation3, point, vector};

    fn get_collection() -> Collection<f64> {
        let mut collection = Collection::default();
        collection.push(CuboidMagnet::new(
            point![0.005, 0.01, 0.015],
            UnitQuaternion::identity(),
            vector![0.1, 0.2, 0.3],
            vector![0.02, 0.02, 0.03],
        ));
        collection.push(CuboidMagnet::new(
            point![0.015, 0.005, 0.01],
            quat_from_rotvec(0.0, FRAC_PI_3, 0.0),
            vector![0.1, 0.2, 0.3],
            vector![0.02, 0.02, 0.03],
        ));
        collection.push(CuboidMagnet::new(
            point![0.01, 0.015, 0.005],
            quat_from_rotvec(0.0, 0.0, FRAC_PI_3),
            vector![0.1, 0.2, 0.3],
            vector![0.02, 0.02, 0.03],
        ));
        collection
    }

    #[test]
    fn test_collection() {
        let collection = get_collection();
        test_B_magnet!(@small, &collection, "cuboid-collection.csv", 2e-13);
    }

    #[test]
    fn test_collection_translate() {
        let mut collection = get_collection();
        let translation = Translation3::new(0.01, 0.015, 0.02);
        collection.translate(translation);
        test_B_magnet!(@small, &collection, "cuboid-collection-translate.csv", 2e-13);

        collection.translate(translation.inverse());
        collection.set_position(point![0.01, 0.015, 0.02]);
        test_B_magnet!(@small, &collection, "cuboid-collection-translate.csv", 2e-13);
    }

    #[test]
    fn test_collection_rotate() {
        let mut collection = get_collection();
        let rotation = quat_from_rotvec(PI / 3.0, PI / 4.0, PI / 5.0);
        collection.rotate(rotation);
        test_B_magnet!(@small, &collection, "cuboid-collection-rotate.csv", 2e-13);

        collection.rotate(rotation.inverse());
        collection.set_orientation(rotation);
        test_B_magnet!(@small, &collection, "cuboid-collection-rotate.csv", 2e-13);

        collection.set_position(point![0.01, 0.015, 0.02]);
        test_B_magnet!(@small, &collection, "cuboid-collection-translate-rotate.csv", 2e-13);
    }
}
