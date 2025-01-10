/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use nalgebra::{Point3, Vector3};

use crate::geometry::Transform;

#[allow(non_snake_case)]
pub trait Field {
    fn get_B(&self, points: &[Point3<f64>]) -> Result<Vec<Vector3<f64>>, &'static str>;
}

pub trait Source: Transform + Field {}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct SourceCollection {}
