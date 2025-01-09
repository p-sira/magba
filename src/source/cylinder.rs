use std::fmt::Display;

/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */
use nalgebra::{Point3, Translation3, UnitQuaternion, Vector3};

use crate::{
    field::{local_cyl_b, local_cyl_b_vec},
    geometry::{global_vector, local_point, Transform},
    impl_transform,
};

use super::Field;

#[derive(Debug, Clone, PartialEq, Default)]
pub struct CylinderMagnet {
    position: Point3<f64>,
    orientation: UnitQuaternion<f64>,
    polarization: Vector3<f64>,

    // Dimension
    radius: f64,
    height: f64,
}

impl_transform!(CylinderMagnet);

impl Field for CylinderMagnet {
    fn b_field(&self, point: &Point3<f64>) -> Result<Vector3<f64>, &'static str> {
        let local_point = local_point(point, &self.position, &self.orientation);
        let local_vector = local_cyl_b(&local_point, self.radius, self.height, &self.polarization)?;
        Ok(global_vector(
            &local_vector,
            &self.position,
            &self.orientation,
        ))
    }

    fn b_fields(&self, points: &[Point3<f64>]) -> Result<Vec<Vector3<f64>>, &'static str> {
        let local_points = points
            .iter()
            .map(|point| local_point(point, &self.position, &self.orientation))
            .collect::<Vec<Point3<f64>>>();
        let local_vectors = local_cyl_b_vec(&local_points, self.radius, self.height, &self.polarization)?;
        let global_vectors = local_vectors
            .iter()
            .map(|local_vector| global_vector(local_vector, &self.position, &self.orientation))
            .collect();
        Ok(global_vectors)
    }
}

impl Display for CylinderMagnet {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "CylinderMagnet (r={}, h={}) at ({}, {})",
            self.radius, self.height, self.position, self.orientation
        )
    }
}
