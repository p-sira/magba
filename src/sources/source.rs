/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Core traits and types for magnetic sources and collections of sources.

use std::fmt::{Debug, Display};

use nalgebra::{Point3, RealField, Translation3, UnitQuaternion, Vector3};

use crate::{
    crate_util::{self, implement},
    geometry::Transform,
    Float,
};

/// Trait shared by objects that generate magnetic field.
#[allow(non_snake_case)]
pub trait Field<T: RealField + Copy> {
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
pub trait Source<T: RealField + Copy>:
    Transform<T> + Field<T> + Debug + Send + Sync + Display
{
}

macro_rules! impl_default {
    ($struct:ident <$(($bound_var:ident : $($bound:tt)+)),+>) => {
        implement!(
            Default for $struct
            bounds: [$(($bound_var : $($bound)+)),+]

            fn default() -> Self {
                Self {
                    position: Point3::origin(),
                    orientation: UnitQuaternion::identity(),
                    children: Vec::new(),
                }
            }
        );
    };
}

macro_rules! impl_transform_collection {
    ($struct:ident <$(($bound_var:ident : $($bound:tt)+)),+>) => {
        implement!(
            Transform<T> for $struct
            bounds: [$(($bound_var : $($bound)+)),+]

            #[inline]
            fn position(&self) -> Point3<T> {
                self.position
            }

            #[inline]
            fn orientation(&self) -> UnitQuaternion<T> {
                self.orientation
            }

            #[inline]
            fn set_position(&mut self, position: Point3<T>) {
                let translation = Translation3::from(position - &self.position);
                self.children
                    .iter_mut()
                    .for_each(|source| source.translate(&translation));

                self.position = position
            }

            #[inline]
            fn set_orientation(&mut self, orientation: UnitQuaternion<T>) {
                let rotation = orientation * &self.orientation.inverse();
                self.children
                    .iter_mut()
                    .for_each(|source| source.rotate_anchor(&rotation, &self.position));

                self.orientation = orientation;
            }

            #[inline]
            fn translate(&mut self, translation: &Translation3<T>) {
                self.children
                    .iter_mut()
                    .for_each(|source| source.translate(translation));

                self.position = translation.transform_point(&self.position);
            }

            #[inline]
            fn rotate(&mut self, rotation: &UnitQuaternion<T>) {
                #[cfg(feature = "parallel")]
                if self.children.len() > 5000 {
                    use rayon::iter::{IntoParallelRefMutIterator, ParallelIterator};

                    self.children
                        .par_iter_mut()
                        .for_each(|source| source.rotate_anchor(rotation, &self.position));

                    self.orientation = rotation * self.orientation;
                    return;
                }

                self.children
                    .iter_mut()
                    .for_each(|source| source.rotate_anchor(rotation, &self.position));
                self.orientation = rotation * self.orientation;
            }

            #[inline]
            fn rotate_anchor(&mut self, rotation: &UnitQuaternion<T>, anchor: &Point3<T>) {
                #[cfg(feature = "parallel")]
                if self.children.len() > 5000 {
                    use rayon::iter::{IntoParallelRefMutIterator, ParallelIterator};
                    self.children
                        .par_iter_mut()
                        .for_each(|source| source.rotate_anchor(rotation, anchor));
                } else {
                    self.children
                        .iter_mut()
                        .for_each(|source| source.rotate_anchor(rotation, anchor));
                }

                #[cfg(not(feature = "parallel"))]
                {
                    self.children
                        .iter_mut()
                        .for_each(|source| source.rotate_anchor(rotation, anchor));
                }

                let local_position = self.position - anchor;
                self.position = Point3::from(rotation * local_position + Vector3::from(anchor.coords));
                self.orientation = rotation * &self.orientation;
            }

        );
    };
    () => {

    };
}

macro_rules! impl_field_collection {
    ($struct:ident <$(($bound_var:ident : $($bound:tt)+)),+>) => {
        implement!(
            Field<T> for $struct
            bounds: [$(($bound_var : $($bound)+)),+]

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
        );
    };
}

macro_rules! impl_partial_eq {
    ($struct:ident <$(($bound_var:ident : $($bound:tt)+)),+>) => {
        implement!(
            PartialEq for $struct
            bounds: [$(($bound_var : $($bound)+)),+]

            fn eq(&self, other: &Self) -> bool {
                if self.position != other.position
                    || self.orientation != other.orientation
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

        );
    };
}

macro_rules! impl_display {
    ($struct:ident <$(($bound_var:ident : $($bound:tt)+)),+>) => {
        implement!(
            Display for $struct
            bounds: [$(($bound_var : $($bound)+)),+]

            fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                let len = self.children.len();
                writeln!(
                    f,
                    concat![stringify!($struct), " ({} children) at pos={}, q={}"],
                    len,
                    crate_util::format_point3!(self.position),
                    crate_util::format_quat!(self.orientation)
                )?;
                for (i, source) in self.children.iter().enumerate() {
                    if i + 1 != len {
                        writeln!(f, "├── {}: {}", i, source)?;
                    } else {
                        write!(f, "└── {}: {}", i, source)?;
                    }
                }
                Ok(())
            }
        );
    }
}

/// Stack-allocated collection of a single source type.
///
/// # Fields
/// - `position`: Center of the collection (m), where the children reference.
/// - `orientation`: Orientation of the collection, where the children reference.
/// - `children`: An ordered-vec of homogeneous magnetic sources.
///
/// # Example
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
    /// Center of the collection (m), where the children reference
    position: Point3<T>,
    /// Orientation of the collection, where the children reference
    orientation: UnitQuaternion<T>,

    /// An ordered-vec of homogeneous magnetic sources
    children: Vec<S>,
}

impl<S: Source<T>, T: Float> Source<T> for SourceCollection<S, T> {}

impl<S: Source<T>, T: Float> SourceCollection<S, T> {
    /// Initialize [SourceCollection].
    pub fn new(position: Point3<T>, orientation: UnitQuaternion<T>, sources: Vec<S>) -> Self {
        Self {
            position,
            orientation,
            children: sources,
        }
    }

    /// Initialize [SourceCollection] from a vec of homogeneous [Source].
    pub fn from_sources(sources: Vec<S>) -> Self {
        Self {
            position: Point3::origin(),
            orientation: UnitQuaternion::identity(),
            children: sources,
        }
    }

    /// Add [Source] to the collection.
    pub fn add(&mut self, source: S) {
        self.children.push(source);
    }

    /// Add multiple [Source] to the collection.
    pub fn add_sources(&mut self, source: &mut Vec<S>) {
        self.children.append(source);
    }
}

impl_default!(SourceCollection <(S: Source<T>), (T: Float)>);
impl_transform_collection!(SourceCollection <(S: Source<T>), (T: Float)>);
impl_field_collection!(SourceCollection <(S: Source<T>), (T: Float)>);
impl_partial_eq!(SourceCollection<(S: Source<T> + PartialEq), (T: Float)>);
impl_display!(SourceCollection <(S: Source<T>), (T: Float)>);

/// Heap-allocated collection of multiple source types.
///
/// # Fields
/// - `position`: Center of the collection (m), where the children reference.
/// - `orientation`: Orientation of the collection, where the children reference.
/// - `children`: An ordered-vec of heterogeneous magnetic sources.
///
/// # Example
/// ```
/// use magba::sources::*;
/// use nalgebra::*;
///
/// let mut collection = MultiSourceCollection::<f64>::default();
/// collection.add(Box::new(CylinderMagnet::default()));
/// collection.add(Box::new(CuboidMagnet::default()));
/// ```
#[derive(Debug)]
pub struct MultiSourceCollection<T: Float> {
    /// Center of the collection (m), where the children reference
    position: Point3<T>,
    /// Orientation of the collection, where the children reference
    orientation: UnitQuaternion<T>,

    /// An ordered-vec of heterogeneous magnetic sources
    children: Vec<Box<dyn Source<T>>>,
}

impl<T: Float> Source<T> for MultiSourceCollection<T> {}

impl<T: Float> MultiSourceCollection<T> {
    /// Initialize [`MultiSourceCollection`].
    pub fn new(
        position: Point3<T>,
        orientation: UnitQuaternion<T>,
        sources: Vec<Box<dyn Source<T>>>,
    ) -> Self {
        Self {
            position,
            orientation,
            children: sources,
        }
    }

    /// Initialize [MultiSourceCollection] from a vec of [Source].
    pub fn from_sources(sources: Vec<Box<dyn Source<T>>>) -> Self {
        Self {
            position: Point3::origin(),
            orientation: UnitQuaternion::identity(),
            children: sources,
        }
    }

    /// Add [Source] to the collection.
    pub fn add(&mut self, source: Box<dyn Source<T>>) {
        self.children.push(source);
    }

    /// Add multiple [Source] to the collection.
    pub fn add_sources(&mut self, source: &mut Vec<Box<dyn Source<T>>>) {
        self.children.append(source);
    }
}

impl_default!(MultiSourceCollection <(T: Float)>);
impl_transform_collection!(MultiSourceCollection <(T: Float)>);
impl_field_collection!(MultiSourceCollection <(T: Float)>);
impl_display!(MultiSourceCollection <(T: Float)>);

#[cfg(test)]
mod base_source_collection_tests {
    use std::f64::consts::FRAC_PI_2;

    use super::*;
    use crate::{sources::*, testing_util::quat_from_rotvec};

    #[test]
    fn test_collection_display() {
        let magnet1 = CylinderMagnet::new(
            Point3::new(4.0, 5.0, 6.0),
            UnitQuaternion::identity(),
            Vector3::new(1.0, 2.0, 3.0),
            0.1,
            0.3,
        );
        let magnet2 = CylinderMagnet::new(
            Point3::new(10.0, 11.0, 12.0),
            quat_from_rotvec(FRAC_PI_2, 0.0, 0.0),
            Vector3::new(7.0, 8.0, 9.0),
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
└── 1: CylinderMagnet (pol=[7, 8, 9], d=0.1, h=0.3) at pos=[10, 11, 12], q=[0.7071067811865475, 0, 0, 0.7071067811865476]", format!("{}", collection))
    }
}

#[cfg(test)]
mod partial_eq_tests {
    use std::f64::consts::FRAC_PI_2;

    use super::*;
    use crate::{sources::*, testing_util::quat_from_rotvec};

    fn create_magnet1() -> CylinderMagnet<f64> {
        CylinderMagnet::new(
            Point3::new(1.0, 2.0, 3.0),
            UnitQuaternion::identity(),
            Vector3::new(1.0, 2.0, 3.0),
            0.1,
            0.2,
        )
    }

    fn create_magnet2() -> CylinderMagnet<f64> {
        CylinderMagnet::new(
            Point3::new(4.0, 5.0, 6.0),
            quat_from_rotvec(FRAC_PI_2, 0.0, 0.0),
            Vector3::new(7.0, 8.0, 9.0),
            0.3,
            0.4,
        )
    }

    fn create_magnet3() -> CylinderMagnet<f64> {
        CylinderMagnet::new(
            Point3::new(10.0, 11.0, 12.0),
            UnitQuaternion::identity(),
            Vector3::new(0.1, 0.2, 0.3),
            0.5,
            0.6,
        )
    }

    #[test]
    fn test_partial_eq_equal_collections() {
        let magnet1 = create_magnet1();
        let magnet2 = create_magnet2();

        let collection1 = SourceCollection::new(
            Point3::new(0.0, 0.0, 0.0),
            UnitQuaternion::identity(),
            vec![magnet1.clone(), magnet2.clone()],
        );

        let collection2 = SourceCollection::new(
            Point3::new(0.0, 0.0, 0.0),
            UnitQuaternion::identity(),
            vec![magnet1, magnet2],
        );

        assert_eq!(collection1, collection2);
    }

    #[test]
    fn test_partial_eq_different_positions() {
        let magnet1 = create_magnet1();
        let magnet2 = create_magnet2();

        let collection1 = SourceCollection::new(
            Point3::new(0.0, 0.0, 0.0),
            UnitQuaternion::identity(),
            vec![magnet1.clone(), magnet2.clone()],
        );

        let collection2 = SourceCollection::new(
            Point3::new(1.0, 0.0, 0.0),
            UnitQuaternion::identity(),
            vec![magnet1, magnet2],
        );

        assert_ne!(collection1, collection2);
    }

    #[test]
    fn test_partial_eq_different_orientations() {
        let magnet1 = create_magnet1();
        let magnet2 = create_magnet2();

        let collection1 = SourceCollection::new(
            Point3::origin(),
            UnitQuaternion::identity(),
            vec![magnet1.clone(), magnet2.clone()],
        );

        let collection2 = SourceCollection::new(
            Point3::origin(),
            quat_from_rotvec(FRAC_PI_2, 0.0, 0.0),
            vec![magnet1, magnet2],
        );

        assert_ne!(collection1, collection2);
    }

    #[test]
    fn test_partial_eq_different_lengths() {
        let magnet1 = create_magnet1();
        let magnet2 = create_magnet2();

        let collection1 = SourceCollection::new(
            Point3::origin(),
            UnitQuaternion::identity(),
            vec![magnet1.clone(), magnet2.clone()],
        );

        let collection2 =
            SourceCollection::new(Point3::origin(), UnitQuaternion::identity(), vec![magnet1]);

        assert_ne!(collection1, collection2);
    }

    #[test]
    fn test_partial_eq_different_children() {
        let magnet1 = create_magnet1();
        let magnet2 = create_magnet2();
        let magnet3 = create_magnet3();

        let collection1 = SourceCollection::new(
            Point3::origin(),
            UnitQuaternion::identity(),
            vec![magnet1.clone(), magnet2.clone()],
        );

        let collection2 = SourceCollection::new(
            Point3::origin(),
            UnitQuaternion::identity(),
            vec![magnet1, magnet3],
        );

        assert_ne!(collection1, collection2);
    }

    #[test]
    fn test_partial_eq_order_independent() {
        let magnet1 = create_magnet1();
        let magnet2 = create_magnet2();

        let collection1 = SourceCollection::new(
            Point3::origin(),
            UnitQuaternion::identity(),
            vec![magnet1.clone(), magnet2.clone()],
        );

        let collection2 = SourceCollection::new(
            Point3::origin(),
            UnitQuaternion::identity(),
            vec![magnet2, magnet1],
        );

        // Collections with same children in different order should be equal
        assert_eq!(collection1, collection2);
    }

    #[test]
    fn test_partial_eq_duplicate_children() {
        let magnet1 = create_magnet1();
        let magnet2 = create_magnet2();

        // Collection with duplicate children
        let collection1 = SourceCollection::new(
            Point3::origin(),
            UnitQuaternion::identity(),
            vec![magnet1.clone(), magnet1.clone()],
        );

        // Collection with two different children
        let collection2 = SourceCollection::new(
            Point3::origin(),
            UnitQuaternion::identity(),
            vec![magnet1.clone(), magnet2],
        );

        // Should not be equal - duplicates must match duplicates
        assert_ne!(collection1, collection2);

        // But two collections with same duplicates should be equal
        let collection3 = SourceCollection::new(
            Point3::origin(),
            UnitQuaternion::identity(),
            vec![magnet1.clone(), magnet1.clone()],
        );

        assert_eq!(collection1, collection3);
    }

    #[test]
    fn test_partial_eq_empty_collections() {
        let collection1: SourceCollection<CylinderMagnet<f64>, f64> =
            SourceCollection::new(Point3::origin(), UnitQuaternion::identity(), vec![]);

        let collection2: SourceCollection<CylinderMagnet<f64>, f64> =
            SourceCollection::new(Point3::origin(), UnitQuaternion::identity(), vec![]);

        assert_eq!(collection1, collection2);
    }

    #[test]
    fn test_partial_eq_empty_vs_non_empty() {
        let magnet1 = create_magnet1();

        let collection1 =
            SourceCollection::new(Point3::origin(), UnitQuaternion::identity(), vec![]);

        let collection2 =
            SourceCollection::new(Point3::origin(), UnitQuaternion::identity(), vec![magnet1]);

        assert_ne!(collection1, collection2);
    }

    #[test]
    fn test_partial_eq_multiple_duplicates() {
        let magnet1 = create_magnet1();
        let magnet2 = create_magnet2();

        // Collection with two of magnet1 and one of magnet2
        let collection1 = SourceCollection::new(
            Point3::origin(),
            UnitQuaternion::identity(),
            vec![magnet1.clone(), magnet1.clone(), magnet2.clone()],
        );

        // Collection with one of magnet1 and two of magnet2
        let collection2 = SourceCollection::new(
            Point3::origin(),
            UnitQuaternion::identity(),
            vec![magnet1.clone(), magnet2.clone(), magnet2.clone()],
        );

        assert_ne!(collection1, collection2);

        // But same duplicates in different order should be equal
        let collection3 = SourceCollection::new(
            Point3::origin(),
            UnitQuaternion::identity(),
            vec![magnet2, magnet1.clone(), magnet1],
        );

        assert_eq!(collection1, collection3);
    }

    #[test]
    fn test_partial_eq_single_child() {
        let magnet1 = create_magnet1();

        let collection1 = SourceCollection::new(
            Point3::origin(),
            UnitQuaternion::identity(),
            vec![magnet1.clone()],
        );

        let collection2 =
            SourceCollection::new(Point3::origin(), UnitQuaternion::identity(), vec![magnet1]);

        assert_eq!(collection1, collection2);
    }

    #[test]
    fn test_partial_eq_same_position_different_children() {
        let magnet1 = create_magnet1();
        let magnet2 = create_magnet2();

        let collection1 = SourceCollection::new(
            Point3::new(5.0, 5.0, 5.0),
            UnitQuaternion::identity(),
            vec![magnet1],
        );

        let collection2 = SourceCollection::new(
            Point3::new(5.0, 5.0, 5.0),
            UnitQuaternion::identity(),
            vec![magnet2],
        );

        assert_ne!(collection1, collection2);
    }
}

#[cfg(test)]
mod cylinder_collection_tests {
    use std::f64::consts::PI;

    use super::*;
    use crate::{sources::*, testing_util::*};

    fn get_collection() -> SourceCollection<CylinderMagnet<f64>, f64> {
        let mut collection = SourceCollection::default();
        collection.add(CylinderMagnet::new(
            Point3::new(0.009389999999999999, 0.0, -0.006),
            quat_from_rotvec(1.2091995761561452, 1.209199576156145, 1.2091995761561452),
            Vector3::new(1.0, 2.0, 3.0),
            3e-3,
            4e-3,
        ));
        collection.add(CylinderMagnet::new(
            Point3::new(-0.004694999999999998, 0.008131978541535878, -0.006),
            quat_from_rotvec(1.5315599088338596, 0.41038024073191587, 0.4103802407319159),
            Vector3::new(0.4, 0.5, 0.6),
            4e-3,
            5e-3,
        ));
        collection.add(CylinderMagnet::new(
            Point3::new(-0.004695000000000004, -0.008131978541535875, -0.006),
            quat_from_rotvec(1.5315599088338594, -0.410380240731917, -0.41038024073191703),
            Vector3::new(0.9, 0.8, 0.6),
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
        collection.translate(&translation);
        test_B_magnet!(@small, &collection, "cylinder-collection-translate.csv", 1e-8);

        collection.translate(&translation.inverse());
        collection.set_position(Point3::new(0.01, 0.015, 0.02));
        test_B_magnet!(@small, &collection, "cylinder-collection-translate.csv", 1e-8);
    }

    #[test]
    fn test_collection_rotate() {
        let mut collection = get_collection();
        let rotation = quat_from_rotvec(PI / 3.0, PI / 4.0, PI / 5.0);
        collection.rotate(&rotation);
        test_B_magnet!(@small, &collection, "cylinder-collection-rotate.csv", 1e-9);

        collection.rotate(&rotation.inverse());
        collection.set_orientation(rotation);
        test_B_magnet!(@small, &collection, "cylinder-collection-rotate.csv", 5e-8);

        collection.set_position(Point3::new(0.01, 0.015, 0.02));
        test_B_magnet!(@small, &collection, "cylinder-collection-translate-rotate.csv", 5e-8);
    }
}

#[cfg(test)]
mod cuboid_collection_tests {
    use std::f64::consts::{FRAC_PI_3, PI};

    use super::*;
    use crate::{sources::*, testing_util::*};

    fn get_collection() -> SourceCollection<CuboidMagnet<f64>, f64> {
        let mut collection = SourceCollection::default();
        collection.add(CuboidMagnet::new(
            Point3::new(0.005, 0.01, 0.015),
            UnitQuaternion::identity(),
            Vector3::new(0.1, 0.2, 0.3),
            Vector3::new(0.02, 0.02, 0.03),
        ));
        collection.add(CuboidMagnet::new(
            Point3::new(0.015, 0.005, 0.01),
            quat_from_rotvec(0.0, FRAC_PI_3, 0.0),
            Vector3::new(0.1, 0.2, 0.3),
            Vector3::new(0.02, 0.02, 0.03),
        ));
        collection.add(CuboidMagnet::new(
            Point3::new(0.01, 0.015, 0.005),
            quat_from_rotvec(0.0, 0.0, FRAC_PI_3),
            Vector3::new(0.1, 0.2, 0.3),
            Vector3::new(0.02, 0.02, 0.03),
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
        collection.translate(&translation);
        test_B_magnet!(@small, &collection, "cuboid-collection-translate.csv", 2e-13);

        collection.translate(&translation.inverse());
        collection.set_position(Point3::new(0.01, 0.015, 0.02));
        test_B_magnet!(@small, &collection, "cuboid-collection-translate.csv", 2e-13);
    }

    #[test]
    fn test_collection_rotate() {
        let mut collection = get_collection();
        let rotation = quat_from_rotvec(PI / 3.0, PI / 4.0, PI / 5.0);
        collection.rotate(&rotation);
        test_B_magnet!(@small, &collection, "cuboid-collection-rotate.csv", 2e-13);

        collection.rotate(&rotation.inverse());
        collection.set_orientation(rotation);
        test_B_magnet!(@small, &collection, "cuboid-collection-rotate.csv", 2e-13);

        collection.set_position(Point3::new(0.01, 0.015, 0.02));
        test_B_magnet!(@small, &collection, "cuboid-collection-translate-rotate.csv", 2e-13);
    }
}

#[cfg(test)]
mod multi_source_collection_tests {
    use std::f64::consts::{FRAC_PI_3, PI};

    use super::*;
    use crate::{sources::*, testing_util::*};

    fn get_collection() -> MultiSourceCollection<f64> {
        let mut collection = MultiSourceCollection::default();
        collection.add(Box::new(CylinderMagnet::new(
            Point3::new(0.005, 0.01, 0.015),
            UnitQuaternion::identity(),
            Vector3::new(0.1, 0.2, 0.3),
            0.04,
            0.05,
        )));
        collection.add(Box::new(CuboidMagnet::new(
            Point3::new(0.015, 0.005, 0.01),
            quat_from_rotvec(0.0, FRAC_PI_3, 0.0),
            Vector3::new(0.1, 0.2, 0.3),
            Vector3::new(0.02, 0.02, 0.03),
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
        collection.translate(&translation);
        test_B_magnet!(@small, &collection, "multi-collection-translate.csv", 5e-11);

        collection.translate(&translation.inverse());
        collection.set_position(Point3::new(0.01, 0.015, 0.02));
        test_B_magnet!(@small, &collection, "multi-collection-translate.csv", 5e-11);
    }

    #[test]
    fn test_collection_rotate() {
        let mut collection = get_collection();
        let rotation = quat_from_rotvec(PI / 3.0, PI / 4.0, PI / 5.0);
        collection.rotate(&rotation);
        test_B_magnet!(@small, &collection, "multi-collection-rotate.csv", 5e-11);

        collection.rotate(&rotation.inverse());
        collection.set_orientation(rotation);
        test_B_magnet!(@small, &collection, "multi-collection-rotate.csv", 5e-11);

        collection.set_position(Point3::new(0.01, 0.015, 0.02));
        test_B_magnet!(@small, &collection, "multi-collection-translate-rotate.csv", 5e-11);
    }

    #[test]
    fn test_collection_display() {
        let collection = get_collection();

        println!("{}", collection);
        assert_eq!("MultiSourceCollection (2 children) at pos=[0, 0, 0], q=[0, 0, 0, 1]
├── 0: CylinderMagnet (pol=[0.1, 0.2, 0.3], d=0.04, h=0.05) at pos=[0.005, 0.01, 0.015], q=[0, 0, 0, 1]
└── 1: CuboidMagnet (pol=[0.1, 0.2, 0.3], dim=[0.02, 0.02, 0.03]) at pos=[0.015, 0.005, 0.01], q=[0, 0.5, 0, 0.8660254037844386]",
         format!("{}", collection))
    }
}
