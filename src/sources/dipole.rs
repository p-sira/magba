/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Defines the [Dipole] struct, representing a magnetic dipole in 3D space.

use getset::{Getters, Setters};
use nalgebra::{Point3, Translation3, UnitQuaternion, Vector3};

use crate::{
    crate_util,
    geometry::{impl_transform, Transform},
    Field, Float, Source,
};

use std::fmt::Display;

#[derive(Debug, Clone, PartialEq, Default, Getters, Setters)]
pub struct Dipole<T: Float> {
    /// Center of the cylinder (m)
    position: Point3<T>,
    /// Orientation as a unit quaternion
    orientation: UnitQuaternion<T>,
    /// Magnetic dipole moment vector (A·m²)
    #[getset(get = "pub", set = "pub")]
    moment: Vector3<T>,
}

impl<T: Float> Dipole<T> {
    pub fn new(position: Point3<T>, orientation: UnitQuaternion<T>, moment: Vector3<T>) -> Self {
        Dipole {
            position,
            orientation,
            moment,
        }
    }
}

impl<T: Float> Source<T> for Dipole<T> {}

impl<T: Float> Transform<T> for Dipole<T> {
    impl_transform!();
}

impl<T: Float> Field<T> for Dipole<T> {
    fn get_B(&self, points: &[Point3<T>]) -> Vec<Vector3<T>> {
        todo!()
    }
}

impl<T: Float> Display for Dipole<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Dipole (m={}) at pos={}, q={}",
            crate_util::format_vector3!(self.moment),
            crate_util::format_point3!(self.position),
            crate_util::format_quat!(self.orientation),
        )
    }
}
