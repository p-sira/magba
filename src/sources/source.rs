/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Core traits and types for magnetic sources and collections of sources.

use std::fmt::{Debug, Display};

use crate::{
    Float, crate_util,
    geometry::{Pose, impl_pose_method},
};
use delegate::delegate;
use nalgebra::{Point3, RealField, Translation3, UnitQuaternion, Vector3};

/// Trait shared by objects that generate magnetic field.
#[allow(non_snake_case)]
pub trait Field<T: RealField> {
    /// Compute the magnetic field (B) at the given points.
    ///
    /// # Arguments
    /// - `points`: Slice of observer positions (m)
    ///
    /// # Returns
    /// - B-field vectors at each observer.
    fn get_B(&self, points: &[Point3<T>]) -> Vec<Vector3<T>>;
}

/// Trait shared by magnetic sources.
///
/// Requires [`Transform`] and [`Field`].
pub trait Source<T: RealField>: Field<T> + Send + Sync {
    /// Get the pose object.
    fn pose(&self) -> &Pose<T>;

    /// Get the mutable pose object.
    fn pose_mut(&mut self) -> &mut Pose<T>;

    /// Set the pose.
    fn set_pose(&mut self, pose: Pose<T>) {
        *self.pose_mut() = pose;
    }

    /// A default formatter that behaves like Display.
    /// Override this for custom printouts.
    fn format(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Source at pos={}, q={}",
            self.pose().position(),
            self.pose().orientation()
        )
    }
}

impl<T: RealField> std::fmt::Display for dyn Source<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // Delegate to the trait method
        self.format(f)
    }
}

/* #region Box<Source> */

impl<S: Field<T> + ?Sized, T: RealField + Copy> Field<T> for Box<S> {
    fn get_B(&self, points: &[Point3<T>]) -> Vec<Vector3<T>> {
        (**self).get_B(points)
    }
}

impl<S: Source<T> + ?Sized, T: RealField + Copy> Source<T> for Box<S> {
    delegate! {
        to (**self) {
            fn pose(&self) -> &Pose<T>;
            fn pose_mut(&mut self) -> &mut Pose<T>;
            fn set_pose(&mut self, pose: Pose<T>);
            fn format(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result;
        }
    }
}

// #region: SourceCollection

/// Stack-allocated collection of a single source type.
///
/// # Fields
/// - `position`: Center of the collection (m), where the children reference.
/// - `orientation`: Orientation of the collection, where the children reference.
/// - `children`: An ordered-vec of homogeneous magnetic sources.
///
/// # Examples
/// ```
/// use magba::sources::*;
/// use nalgebra::*;
///
/// let magnet = CylinderMagnet::<f64>::default();
/// let mut collection = SourceCollection::default();
/// collection.add(magnet);
/// ```
#[derive(Debug)]
pub struct SourceCollection<S: Source<T>, T: Float> {
    /// Local frame of the collection, where the children reference
    pose: Pose<T>,

    /// An ordered-vec of homogeneous magnetic sources
    children: Vec<S>,

    /// The relative pose of each child in LOCAL frame
    offsets: Vec<Pose<T>>,
}

impl<S: Source<T>, T: Float> SourceCollection<S, T> {
    /// Initialize [SourceCollection].
    pub fn new(position: Point3<T>, orientation: UnitQuaternion<T>, sources: Vec<S>) -> Self {
        let pose = Pose::new(position, orientation);
        let pose_inv = pose.as_isometry().inverse();
        let offsets = sources
            .iter()
            .map(|s| (pose_inv * s.pose().as_isometry()).into())
            .collect();

        Self {
            pose,
            children: sources,
            offsets,
        }
    }

    /// Initialize [SourceCollection] from a vec of homogeneous [Source].
    pub fn from_sources(sources: Vec<S>) -> Self {
        let offsets = sources.iter().map(|s| s.pose().clone()).collect();

        Self {
            pose: Pose::default(),
            children: sources,
            offsets,
        }
    }

    /// Add [Source] to the collection.
    pub fn add(&mut self, source: S) {
        let relative_pose = self.pose.as_isometry().inverse() * source.pose().as_isometry();

        self.offsets.push(relative_pose.into());
        self.children.push(source);
    }

    /// Add multiple [Source] to the collection.
    pub fn add_sources(&mut self, source: &mut Vec<S>) {
        self.children.append(source);
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

impl<S: Source<T>, T: Float> Source<T> for SourceCollection<S, T> {
    impl_pose_method!();
}

/// Alias of [SourceCollection] for heap-allocated collection of multiple source types.
pub type MultiSourceCollection<T> = SourceCollection<Box<dyn Source<T>>, T>;

// #region Default Implementation

impl<S: Source<T>, T: Float> Default for SourceCollection<S, T> {
    fn default() -> Self {
        Self {
            pose: Pose::default(),
            children: Vec::new(),
            offsets: Vec::new(),
        }
    }
}

// #region Field Implementation

impl<S: Source<T>, T: Float> Field<T> for SourceCollection<S, T> {
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

// #region Partial Equal Implementation

impl<S: Source<T> + PartialEq, T: Float> PartialEq for SourceCollection<S, T> {
    fn eq(&self, other: &Self) -> bool {
        if self.position() != other.position()
            || self.orientation() != other.orientation()
            || self.children.len() != other.children.len()
        {
            return false;
        }

        // Track which elements in other.children have been matched
        // This ensures proper 1:1 matching and handles duplicates correctly
        let mut matched = vec![false; other.children.len()];

        for source in &self.children {
            // Find the first unmatched element in other that equals source
            let found = other
                .children
                .iter()
                .enumerate()
                .find(|(idx, other_source)| !matched[*idx] && source.eq(other_source));

            match found {
                Some((idx, _)) => matched[idx] = true,
                None => return false,
            }
        }

        true
    }
}

// #region Display Implementation

impl<S: Source<T>, T: Float> Display for SourceCollection<S, T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let len = self.children.len();
        writeln!(
            f,
            "SourceCollection ({} children) at pos={}, q={}",
            len,
            crate_util::format_point3!(self.position()),
            crate_util::format_quat!(self.orientation())
        )?;

        for (i, source) in self.children.iter().enumerate() {
            if i != 0 {
                writeln!(f)?;
            }

            let prefix = if i + 1 != len {
                "├──"
            } else {
                "└──"
            };

            write!(f, "{} {}: ", prefix, i)?;
            source.format(f)?;
        }

        Ok(())
    }
}

#[cfg(test)]
mod mytest {
    use crate::{CuboidMagnet, CylinderMagnet, Source, SourceCollection};

    #[test]
    fn my() {
        let cylinder = CylinderMagnet::default();
        let cuboid = CuboidMagnet::default();

        let sources: Vec<Box<dyn Source<f64>>> = vec![Box::new(cylinder), Box::new(cuboid)];

        SourceCollection::from_sources(sources);
    }
}

#[cfg(test)]
mod base_source_collection_tests {
    use std::f64::consts::FRAC_PI_2;

    use super::*;
    use crate::{sources::*, testing_util::*};
    use nalgebra::{point, vector};

    #[test]
    fn test_collection_display() {
        let magnet1 = CylinderMagnet::new(
            point![4.0, 5.0, 6.0],
            UnitQuaternion::identity(),
            vector![1.0, 2.0, 3.0],
            0.1,
            0.3,
        );
        let magnet2 = CylinderMagnet::new(
            point![10.0, 11.0, 12.0],
            quat_from_rotvec(FRAC_PI_2, 0.0, 0.0),
            vector![7.0, 8.0, 9.0],
            0.1,
            0.3,
        );

        let collection = SourceCollection::new(
            Point3::origin(),
            UnitQuaternion::identity(),
            vec![magnet1, magnet2],
        );

        assert_eq!("SourceCollection (2 children) at pos=[0, 0, 0], q=[0, 0, 0, 1]
├── 0: CylinderMagnet (pol=[1, 2, 3], d=0.1, h=0.3) at pos=[4, 5, 6], q=[0, 0, 0, 1]
└── 1: CylinderMagnet (pol=[7, 8, 9], d=0.1, h=0.3) at pos=[10, 11, 12], q=[<float>, 0, 0, <float>]", mask_long_floats(&format!("{}", collection)))
    }
}

#[cfg(test)]
mod partial_eq_tests {
    use std::f64::consts::FRAC_PI_2;

    use super::*;
    use crate::{sources::*, testing_util::quat_from_rotvec};
    use nalgebra::{point, vector};

    fn magnet1() -> CylinderMagnet<f64> {
        CylinderMagnet::new(
            point![1.0, 2.0, 3.0],
            UnitQuaternion::identity(),
            vector![1.0, 2.0, 3.0],
            0.1,
            0.2,
        )
    }

    fn magnet2() -> CylinderMagnet<f64> {
        CylinderMagnet::new(
            point![4.0, 5.0, 6.0],
            quat_from_rotvec(FRAC_PI_2, 0.0, 0.0),
            vector![7.0, 8.0, 9.0],
            0.3,
            0.4,
        )
    }

    fn magnet3() -> CylinderMagnet<f64> {
        CylinderMagnet::new(
            point![10.0, 11.0, 12.0],
            UnitQuaternion::identity(),
            vector![0.1, 0.2, 0.3],
            0.5,
            0.6,
        )
    }

    fn collection(
        pos: Point3<f64>,
        orient: UnitQuaternion<f64>,
        magnets: Vec<CylinderMagnet<f64>>,
    ) -> SourceCollection<CylinderMagnet<f64>, f64> {
        SourceCollection::new(pos, orient, magnets)
    }

    #[test]
    fn test_equal() {
        let c1 = collection(
            Point3::origin(),
            UnitQuaternion::identity(),
            vec![magnet1(), magnet2()],
        );
        let c2 = collection(
            Point3::origin(),
            UnitQuaternion::identity(),
            vec![magnet1(), magnet2()],
        );
        assert_eq!(c1, c2);
    }

    #[test]
    fn test_different_position() {
        let c1 = collection(
            Point3::origin(),
            UnitQuaternion::identity(),
            vec![magnet1()],
        );
        let c2 = collection(
            point![1.0, 0.0, 0.0],
            UnitQuaternion::identity(),
            vec![magnet1()],
        );
        assert_ne!(c1, c2);
    }

    #[test]
    fn test_different_orientation() {
        let c1 = collection(
            Point3::origin(),
            UnitQuaternion::identity(),
            vec![magnet1()],
        );
        let c2 = collection(
            Point3::origin(),
            quat_from_rotvec(FRAC_PI_2, 0.0, 0.0),
            vec![magnet1()],
        );
        assert_ne!(c1, c2);
    }

    #[test]
    fn test_different_length() {
        let c1 = collection(
            Point3::origin(),
            UnitQuaternion::identity(),
            vec![magnet1(), magnet2()],
        );
        let c2 = collection(
            Point3::origin(),
            UnitQuaternion::identity(),
            vec![magnet1()],
        );
        assert_ne!(c1, c2);
    }

    #[test]
    fn test_different_children() {
        let c1 = collection(
            Point3::origin(),
            UnitQuaternion::identity(),
            vec![magnet1(), magnet2()],
        );
        let c2 = collection(
            Point3::origin(),
            UnitQuaternion::identity(),
            vec![magnet1(), magnet3()],
        );
        assert_ne!(c1, c2);
    }

    #[test]
    fn test_order_independent() {
        let c1 = collection(
            Point3::origin(),
            UnitQuaternion::identity(),
            vec![magnet1(), magnet2()],
        );
        let c2 = collection(
            Point3::origin(),
            UnitQuaternion::identity(),
            vec![magnet2(), magnet1()],
        );
        assert_eq!(c1, c2);
    }

    #[test]
    fn test_duplicates() {
        let c1 = collection(
            Point3::origin(),
            UnitQuaternion::identity(),
            vec![magnet1(), magnet1()],
        );
        let c2 = collection(
            Point3::origin(),
            UnitQuaternion::identity(),
            vec![magnet1(), magnet2()],
        );
        let c3 = collection(
            Point3::origin(),
            UnitQuaternion::identity(),
            vec![magnet1(), magnet1()],
        );

        assert_ne!(c1, c2);
        assert_eq!(c1, c3);
    }

    #[test]
    fn test_empty() {
        let c1: SourceCollection<CylinderMagnet<f64>, f64> =
            collection(Point3::origin(), UnitQuaternion::identity(), vec![]);
        let c2: SourceCollection<CylinderMagnet<f64>, f64> =
            collection(Point3::origin(), UnitQuaternion::identity(), vec![]);
        assert_eq!(c1, c2);
    }

    #[test]
    fn test_empty_vs_non_empty() {
        let c1 = collection(Point3::origin(), UnitQuaternion::identity(), vec![]);
        let c2 = collection(
            Point3::origin(),
            UnitQuaternion::identity(),
            vec![magnet1()],
        );
        assert_ne!(c1, c2);
    }

    #[test]
    fn test_multiple_duplicates() {
        let c1 = collection(
            Point3::origin(),
            UnitQuaternion::identity(),
            vec![magnet1(), magnet1(), magnet2()],
        );
        let c2 = collection(
            Point3::origin(),
            UnitQuaternion::identity(),
            vec![magnet1(), magnet2(), magnet2()],
        );
        let c3 = collection(
            Point3::origin(),
            UnitQuaternion::identity(),
            vec![magnet2(), magnet1(), magnet1()],
        );

        assert_ne!(c1, c2);
        assert_eq!(c1, c3);
    }
}

#[cfg(test)]
mod cylinder_collection_tests {
    use std::f64::consts::PI;

    use super::*;
    use crate::{sources::*, testing_util::*};
    use nalgebra::{Translation3, point, vector};

    fn get_collection() -> SourceCollection<CylinderMagnet<f64>, f64> {
        let mut collection = SourceCollection::default();
        collection.add(CylinderMagnet::new(
            point![0.009389999999999999, 0.0, -0.006],
            quat_from_rotvec(1.2091995761561452, 1.209199576156145, 1.2091995761561452),
            vector![1.0, 2.0, 3.0],
            3e-3,
            4e-3,
        ));
        collection.add(CylinderMagnet::new(
            point![-0.004694999999999998, 0.008131978541535878, -0.006],
            quat_from_rotvec(1.5315599088338596, 0.41038024073191587, 0.4103802407319159),
            vector![0.4, 0.5, 0.6],
            4e-3,
            5e-3,
        ));
        collection.add(CylinderMagnet::new(
            point![-0.004695000000000004, -0.008131978541535875, -0.006],
            quat_from_rotvec(1.5315599088338594, -0.410380240731917, -0.41038024073191703),
            vector![0.9, 0.8, 0.6],
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
    use crate::{sources::*, testing_util::*};
    use nalgebra::{Translation3, point, vector};

    fn get_collection() -> SourceCollection<CuboidMagnet<f64>, f64> {
        let mut collection = SourceCollection::default();
        collection.add(CuboidMagnet::new(
            point![0.005, 0.01, 0.015],
            UnitQuaternion::identity(),
            vector![0.1, 0.2, 0.3],
            vector![0.02, 0.02, 0.03],
        ));
        collection.add(CuboidMagnet::new(
            point![0.015, 0.005, 0.01],
            quat_from_rotvec(0.0, FRAC_PI_3, 0.0),
            vector![0.1, 0.2, 0.3],
            vector![0.02, 0.02, 0.03],
        ));
        collection.add(CuboidMagnet::new(
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

#[cfg(test)]
mod multi_source_collection_tests {
    use std::f64::consts::{FRAC_PI_3, PI};

    use super::*;
    use crate::{sources::*, testing_util::*};
    use nalgebra::{Translation3, point, vector};

    fn get_collection() -> MultiSourceCollection<f64> {
        let mut collection = MultiSourceCollection::default();
        collection.add(Box::new(CylinderMagnet::new(
            point![0.005, 0.01, 0.015],
            UnitQuaternion::identity(),
            vector![0.1, 0.2, 0.3],
            0.04,
            0.05,
        )));
        collection.add(Box::new(CuboidMagnet::new(
            point![0.015, 0.005, 0.01],
            quat_from_rotvec(0.0, FRAC_PI_3, 0.0),
            vector![0.1, 0.2, 0.3],
            vector![0.02, 0.02, 0.03],
        )));
        collection
    }

    #[test]
    fn test_collection() {
        let collection = get_collection();
        test_B_magnet!(@small, &collection, "multi-collection.csv", 5e-11);
    }

    #[test]
    fn test_collection_translate() {
        let mut collection = get_collection();
        let translation = Translation3::new(0.01, 0.015, 0.02);
        collection.translate(translation.clone());
        test_B_magnet!(@small, &collection, "multi-collection-translate.csv", 5e-11);

        collection.translate(translation.inverse());
        collection.set_position(point![0.01, 0.015, 0.02]);
        test_B_magnet!(@small, &collection, "multi-collection-translate.csv", 5e-11);
    }

    #[test]
    fn test_collection_rotate() {
        let mut collection = get_collection();
        let rotation = quat_from_rotvec(PI / 3.0, PI / 4.0, PI / 5.0);
        collection.rotate(rotation);
        test_B_magnet!(@small, &collection, "multi-collection-rotate.csv", 5e-11);

        collection.rotate(rotation.inverse());
        collection.set_orientation(rotation);
        test_B_magnet!(@small, &collection, "multi-collection-rotate.csv", 5e-11);

        collection.set_position(point![0.01, 0.015, 0.02]);
        test_B_magnet!(@small, &collection, "multi-collection-translate-rotate.csv", 5e-11);
    }

    #[test]
    fn test_collection_display() {
        let collection = get_collection();

        println!("{}", collection);
        assert_eq!("SourceCollection (2 children) at pos=[0, 0, 0], q=[0, 0, 0, 1]
├── 0: CylinderMagnet (pol=[0.1, 0.2, 0.3], d=0.04, h=0.05) at pos=[0.005, 0.01, 0.015], q=[0, 0, 0, 1]
└── 1: CuboidMagnet (pol=[0.1, 0.2, 0.3], dim=[0.02, 0.02, 0.03]) at pos=[0.015, 0.005, 0.01], q=[0, 0.5, 0, <float>]",
         mask_long_floats(&format!("{}", collection)))
    }
}
