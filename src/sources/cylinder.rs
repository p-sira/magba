/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */
use nalgebra::{Point3, Translation3, UnitQuaternion, Vector3};

use super::Field;
use crate::geometry::transform::Transform;
use crate::{fields::cyl_B, impl_transform};

use std::fmt::Display;

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
    fn get_B(&self, points: &[Point3<f64>]) -> Result<Vec<Vector3<f64>>, &'static str> {
        Ok(cyl_B(
            &points,
            &self.position,
            &self.orientation,
            self.radius,
            self.height,
            &self.polarization,
        )?)
    }
}

impl Display for CylinderMagnet {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "CylinderMagnet (r={}, h={}, pol={}) at ({}, {})",
            self.radius, self.height, self.polarization, self.position, self.orientation
        )
    }
}
