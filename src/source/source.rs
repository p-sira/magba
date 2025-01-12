/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use nalgebra::{Point3, Translation3, UnitQuaternion, Vector3};
use rayon::iter::{
    IntoParallelIterator, IntoParallelRefIterator, IntoParallelRefMutIterator, ParallelIterator,
};

use crate::geometry::Transform;

#[allow(non_snake_case)]
pub trait Field {
    fn get_B(&self, points: &[Point3<f64>]) -> Result<Vec<Vector3<f64>>, &'static str>;
}

pub trait Source: Transform + Field + Send + Sync {}

pub struct SourceCollection {
    position: Point3<f64>,
    orientation: UnitQuaternion<f64>,

    sources: [Box<dyn Source>],
}

impl Transform for SourceCollection {
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
}

impl Field for SourceCollection {
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
}
