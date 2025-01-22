/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use std::fmt::Debug;

use nalgebra::{Point3, Translation3, UnitQuaternion, Vector3};
use rayon::iter::{
    IntoParallelIterator, IntoParallelRefIterator, IntoParallelRefMutIterator, ParallelIterator,
};

use crate::geometry::transform::Transform;

#[allow(non_snake_case)]
pub trait Field {
    fn get_B(&self, points: &[Point3<f64>]) -> Result<Vec<Vector3<f64>>, &'static str>;
}

pub trait Source: Transform + Field + Debug + Send + Sync {}

macro_rules! impl_default {
    () => {
        fn default() -> Self {
            Self {
                position: Point3::origin(),
                orientation: UnitQuaternion::identity(),
                sources: Vec::new(),
            }
        }
    };
}

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
            self.sources
                .par_iter_mut()
                .for_each(|source| source.translate(&translation));
            self.position = position
        }

        fn set_orientation(&mut self, orientation: UnitQuaternion<f64>) {
            let rotation = orientation * &self.orientation.inverse();
            self.sources
                .par_iter_mut()
                .for_each(|source| source.rotate(&rotation));
            self.orientation = orientation;
        }

        fn translate(&mut self, translation: &Translation3<f64>) {
            self.sources
                .par_iter_mut()
                .for_each(|source| source.translate(&translation));
            self.position = translation.transform_point(&self.position);
        }

        fn rotate(&mut self, rotation: &UnitQuaternion<f64>) {
            self.sources
                .par_iter_mut()
                .for_each(|source| source.rotate(rotation));
            self.orientation = rotation * self.orientation;
        }
    };
}

macro_rules! impl_field_collection {
    () => {
        fn get_B(&self, points: &[Point3<f64>]) -> Result<Vec<Vector3<f64>>, &'static str> {
            let b_fields = self
                .sources
                .par_iter()
                .map(|source| source.get_B(points))
                .collect::<Result<Vec<_>, _>>()?;

            let net_field = (0..points.len())
                .into_par_iter()
                .map(|i| b_fields.iter().map(|vector| vector[i]).sum())
                .collect::<Vec<Vector3<_>>>();

            Ok(net_field)
        }
    };
}

#[derive(Debug)]
pub struct SingleSourceCollection<S: Source> {
    position: Point3<f64>,
    orientation: UnitQuaternion<f64>,

    sources: Vec<S>,
}

impl<S: Source> Source for SingleSourceCollection<S> {}

impl<S: Source> SingleSourceCollection<S> {
    pub fn new(position: Point3<f64>, orientation: UnitQuaternion<f64>, sources: Vec<S>) -> Self {
        Self {
            position,
            orientation,
            sources,
        }
    }

    pub fn add(&mut self, source: S) {
        self.sources.push(source);
    }

    pub fn add_sources(&mut self, source: &mut Vec<S>) {
        self.sources.append(source);
    }
}

impl<S: Source> Default for SingleSourceCollection<S> {
    impl_default!();
}

impl<S: Source> Transform for SingleSourceCollection<S> {
    impl_transform_collection!();
}

impl<S: Source> Field for SingleSourceCollection<S> {
    impl_field_collection!();
}

#[derive(Debug)]
pub struct SourceCollection {
    position: Point3<f64>,
    orientation: UnitQuaternion<f64>,

    sources: Vec<Box<dyn Source>>,
}

impl Source for SourceCollection {}

impl SourceCollection {
    pub fn new(
        position: Point3<f64>,
        orientation: UnitQuaternion<f64>,
        sources: Vec<Box<dyn Source>>,
    ) -> Self {
        Self {
            position,
            orientation,
            sources,
        }
    }

    pub fn add(&mut self, source: Box<dyn Source>) {
        self.sources.push(source);
    }

    pub fn add_sources(&mut self, source: &mut Vec<Box<dyn Source>>) {
        self.sources.append(source);
    }
}

impl Default for SourceCollection {
    impl_default!();
}

impl Transform for SourceCollection {
    impl_transform_collection!();
}

impl Field for SourceCollection {
    impl_field_collection!();
}

#[cfg(test)]
mod single_source_collection_tests {
    use super::*;
    use crate::{sources::*, testing_util::*};

    fn compare_with_file(
        collection: &SingleSourceCollection<CylinderMagnet>,
        ref_path_str: &str,
        rtol: f64,
    ) {
        compare_B_with_file(
            collection,
            "./tests/test-data/single-collection-points.mtx",
            ref_path_str,
            rtol,
        );
    }

    fn get_cylinder_collection() -> SingleSourceCollection<CylinderMagnet> {
        let mut collection = SingleSourceCollection::default();
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
            "./tests/test-data/cylinder-collection-result.mtx",
            1e-3,
        );
    }
}
