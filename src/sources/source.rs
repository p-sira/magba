/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! # Sources
//!
//! Defines the core traits and types for magnetic field sources, including collections of sources.
//!
//! - [`Field`] trait: For objects that can compute the magnetic field at given points.
//! - [`Source`] trait: For magnetic sources, requiring both [`Field`] and [`Transform`].
//! - [`SourceCollection<S>`]: Stack-allocated collection of a single source type, supports adding sources and computing net field.
//! - [`MultiSourceCollection`]: Collection of heterogeneous sources (`Box<dyn Source>`), supports adding sources and computing net field.

use std::{fmt::Debug, fmt::Display};

use nalgebra::{Point3, Translation3, UnitQuaternion, Vector3};

#[cfg(feature = "parallel")]
use rayon::iter::{IntoParallelIterator, IntoParallelRefIterator, ParallelIterator};

use crate::geometry::Transform;

/// Trait shared by objects that generate magnetic field.
#[allow(non_snake_case)]
pub trait Field {
    /// Compute the magnetic field (B) at the given points.
    ///
    /// # Arguments
    /// * `points` - Slice of 3D points where the field is evaluated.
    ///
    /// # Returns
    /// * `Ok(Vec<Vector3<f64>>)` - B-field vectors at each point.
    /// * `Err(&'static str)` - If computation fails.
    fn get_B(&self, points: &[Point3<f64>]) -> Result<Vec<Vector3<f64>>, &'static str>;
}

/// Trait shared by magnetic sources.
///
/// Requires [`Transform`] and [`Field`].
pub trait Source: Transform + Field + Debug + Send + Sync + Display {}

macro_rules! impl_default {
    () => {
        fn default() -> Self {
            Self {
                position: Point3::origin(),
                orientation: UnitQuaternion::identity(),
                children: Vec::new(),
            }
        }
    };
}

/// Implement deep Transform for objects with children.
macro_rules! impl_transform_collection {
    () => {
        fn position(&self) -> Point3<f64> {
            self.position
        }

        fn orientation(&self) -> UnitQuaternion<f64> {
            self.orientation
        }

        fn set_position(&mut self, position: Point3<f64>) {
            let translation = Translation3::from(position - &self.position);
            self.children
                .iter_mut()
                .for_each(|source| source.translate(&translation));

            self.position = position
        }

        fn set_orientation(&mut self, orientation: UnitQuaternion<f64>) {
            let rotation = orientation * &self.orientation.inverse();
            self.children
                .iter_mut()
                .for_each(|source| source.rotate_anchor(&rotation, &self.position));

            self.orientation = orientation;
        }

        fn translate(&mut self, translation: &Translation3<f64>) {
            self.children
                .iter_mut()
                .for_each(|source| source.translate(translation));

            self.position = translation.transform_point(&self.position);
        }

        fn rotate(&mut self, rotation: &UnitQuaternion<f64>) {
            self.children
                .iter_mut()
                .for_each(|source| source.rotate_anchor(rotation, &self.position));

            self.orientation = rotation * self.orientation;
        }

        fn rotate_anchor(&mut self, rotation: &UnitQuaternion<f64>, anchor: &Point3<f64>) {
            self.children
                .iter_mut()
                .for_each(|source| source.rotate_anchor(rotation, anchor));

            let local_position = self.position - anchor;
            self.position = Point3::from(rotation * local_position + Vector3::from(anchor.coords));
            self.orientation = rotation * &self.orientation;
        }
    };
}

/// Implement Field for source collection-like structs.
macro_rules! impl_field_collection {
    () => {
        fn get_B(&self, points: &[Point3<f64>]) -> Result<Vec<Vector3<f64>>, &'static str> {
            let net_field;
            #[cfg(feature = "parallel")]
            {
                let b_fields = self
                    .children
                    .par_iter()
                    .map(|source| source.get_B(points))
                    .collect::<Result<Vec<_>, _>>()?;

                net_field = (0..points.len())
                    .into_par_iter()
                    .map(|i| b_fields.iter().map(|vector| vector[i]).sum())
                    .collect::<Vec<Vector3<_>>>();
            }

            #[cfg(not(feature = "parallel"))]
            {
                let b_fields = self
                    .children
                    .iter()
                    .map(|source| source.get_B(points))
                    .collect::<Result<Vec<_>, _>>()?;

                net_field = (0..points.len())
                    .into_iter()
                    .map(|i| b_fields.iter().map(|vector| vector[i]).sum())
                    .collect::<Vec<Vector3<_>>>();
            }

            Ok(net_field)
        }
    };
}

/// Stack-allocated collection of a single source type.
///
/// # Example
/// ```
/// use magba::sources::{SourceCollection, CylinderMagnet};
/// use nalgebra::{Point3, UnitQuaternion, Vector3};
/// 
/// let magnet = CylinderMagnet::new(Point3::origin(), UnitQuaternion::identity(), Vector3::z(), 0.005, 0.02);
/// let mut collection = SourceCollection::new(Point3::origin(), UnitQuaternion::identity(), vec![magnet]);
/// ```
#[derive(Debug)]
pub struct SourceCollection<S: Source> {
    position: Point3<f64>,
    orientation: UnitQuaternion<f64>,

    children: Vec<S>,
}

impl<S: Source + PartialEq> PartialEq for SourceCollection<S> {
    fn eq(&self, other: &Self) -> bool {
        self.position == other.position
            && self.orientation == other.orientation
            && self.children.len() == other.children.len()
            && self.children.iter().all(|source| {
                other
                    .children
                    .iter()
                    .any(|other_source| source.eq(other_source))
            })
    }
}

impl<S: Source> Source for SourceCollection<S> {}

impl<S: Source> SourceCollection<S> {
    /// Initialize [`SourceCollection`].
    pub fn new(position: Point3<f64>, orientation: UnitQuaternion<f64>, sources: Vec<S>) -> Self {
        Self {
            position,
            orientation,
            children: sources,
        }
    }

    /// Add [`Source`] to the collection.
    pub fn add(&mut self, source: S) {
        self.children.push(source);
    }

    /// Add multiple [`Source`] to the collection.
    pub fn add_sources(&mut self, source: &mut Vec<S>) {
        self.children.append(source);
    }
}

impl<S: Source> Display for SourceCollection<S> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(
            f,
            "SourceCollection at {}, {}",
            self.position, self.orientation
        )?;
        if let Some((last, sources)) = self.children.split_last() {
            for source in sources {
                writeln!(f, "├── {}", source)?;
            }
            writeln!(f, "└── {}", last)?;
        }
        Ok(())
    }
}

impl<S: Source> Default for SourceCollection<S> {
    impl_default!();
}

impl<S: Source> Transform for SourceCollection<S> {
    impl_transform_collection!();
}

impl<S: Source> Field for SourceCollection<S> {
    impl_field_collection!();
}

/// Heap-allocated collection of multiple source types.
///
/// # Example
/// ```
/// use magba::sources::{MultiSourceCollection, CylinderMagnet, Source};
/// use nalgebra::{Point3, UnitQuaternion, Vector3};
/// 
/// let cylinder_magnet: Box<dyn Source> = Box::new(CylinderMagnet::new(Point3::origin(), UnitQuaternion::identity(), Vector3::z(), 0.005, 0.02));
/// let mut collection = MultiSourceCollection::new(Point3::origin(), UnitQuaternion::identity(), vec![cylinder_magnet]);
/// ```
#[derive(Debug)]
pub struct MultiSourceCollection {
    position: Point3<f64>,
    orientation: UnitQuaternion<f64>,

    children: Vec<Box<dyn Source>>,
}

impl Source for MultiSourceCollection {}

impl MultiSourceCollection {
    /// Initialize [`MultiSourceCollection`].
    pub fn new(
        position: Point3<f64>,
        orientation: UnitQuaternion<f64>,
        sources: Vec<Box<dyn Source>>,
    ) -> Self {
        Self {
            position,
            orientation,
            children: sources,
        }
    }

    /// Add [`Source`] to the collection.
    pub fn add(&mut self, source: Box<dyn Source>) {
        self.children.push(source);
    }

    /// Add multiple [`Source`] to the collection.
    pub fn add_sources(&mut self, source: &mut Vec<Box<dyn Source>>) {
        self.children.append(source);
    }
}

impl Default for MultiSourceCollection {
    impl_default!();
}

impl Display for MultiSourceCollection {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(
            f,
            "MultiSourceCollection at {}, {}",
            self.position, self.orientation
        )?;

        if let Some((last, sources)) = self.children.split_last() {
            for source in sources {
                writeln!(f, "├── {}", source)?;
            }
            writeln!(f, "└── {}", last)?;
        }
        Ok(())
    }
}

impl Transform for MultiSourceCollection {
    impl_transform_collection!();
}

impl Field for MultiSourceCollection {
    impl_field_collection!();
}

#[cfg(test)]
mod single_source_collection_tests {
    use std::f64::consts::PI;

    use super::*;
    use crate::{sources::*, testing_util::*};

    fn compare_with_file(
        collection: &SourceCollection<CylinderMagnet>,
        ref_path_str: &str,
        rtol: f64,
    ) {
        compare_B_with_file(
            collection,
            "./tests/test-data/single-collection-points.csv",
            ref_path_str,
            rtol,
        );
    }

    fn get_cylinder_collection() -> SourceCollection<CylinderMagnet> {
        let mut collection = SourceCollection::default();
        collection.add(CylinderMagnet::new(
            Point3::new(0.009389999999999999, 0.0, -0.006),
            quat_from_rotvec(1.2091995761561452, 1.209199576156145, 1.2091995761561452),
            Vector3::new(1.0, 2.0, 3.0),
            1.5e-3,
            4e-3,
        ));
        collection.add(CylinderMagnet::new(
            Point3::new(-0.004694999999999998, 0.008131978541535878, -0.006),
            quat_from_rotvec(1.5315599088338596, 0.41038024073191587, 0.4103802407319159),
            Vector3::new(0.4, 0.5, 0.6),
            2e-3,
            5e-3,
        ));
        collection.add(CylinderMagnet::new(
            Point3::new(-0.004695000000000004, -0.008131978541535875, -0.006),
            quat_from_rotvec(1.5315599088338594, -0.410380240731917, -0.41038024073191703),
            Vector3::new(0.9, 0.8, 0.6),
            2.5e-3,
            6e-3,
        ));
        collection
    }

    #[test]
    fn test_cylinder_collection() {
        let collection = get_cylinder_collection();
        compare_with_file(
            &collection,
            "./tests/test-data/cylinder-collection-result.csv",
            5e-9,
        );
    }

    #[test]
    fn test_cylinder_collection_translate() {
        let mut collection = get_cylinder_collection();
        let translation = Translation3::new(0.01, 0.015, 0.02);
        collection.translate(&translation);
        compare_with_file(
            &collection,
            "./tests/test-data/cylinder-collection-translate-result.csv",
            1e-8,
        );

        collection.translate(&translation.inverse());
        collection.set_position(Point3::new(0.01, 0.015, 0.02));
        compare_with_file(
            &collection,
            "./tests/test-data/cylinder-collection-translate-result.csv",
            1e-8,
        );
    }

    #[test]
    fn test_cylinder_collection_rotate() {
        let mut collection = get_cylinder_collection();
        let rotation = quat_from_rotvec(PI / 3.0, PI / 4.0, PI / 5.0);
        collection.rotate(&rotation);
        compare_with_file(
            &collection,
            "./tests/test-data/cylinder-collection-rotate-result.csv",
            5e-8,
        );

        collection.rotate(&rotation.inverse());
        collection.set_orientation(rotation);
        compare_with_file(
            &collection,
            "./tests/test-data/cylinder-collection-rotate-result.csv",
            5e-8,
        );
    }
}
