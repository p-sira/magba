/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use nalgebra::{Point3, Translation3, UnitQuaternion, Vector3};

/// Transform the object in 3D CS
pub trait Transform {
    fn position(&self) -> Point3<f64>;
    fn orientation(&self) -> UnitQuaternion<f64>;
    fn set_position(&mut self, position: Point3<f64>);
    fn set_orientation(&mut self, orientation: UnitQuaternion<f64>);
    fn translate(&mut self, translation: &Translation3<f64>);
    fn rotate(&mut self, rotation: &UnitQuaternion<f64>);
}

/// Implement shallow Transform for objects with no children
#[macro_export]
macro_rules! impl_transform {
    ($type:ty) => {
        impl Transform for $type {
            fn position(&self) -> Point3<f64> {
                self.position
            }

            fn orientation(&self) -> UnitQuaternion<f64> {
                self.orientation
            }

            fn set_position(&mut self, position: Point3<f64>) {
                self.position = position;
            }

            fn set_orientation(&mut self, orientation: UnitQuaternion<f64>) {
                self.orientation = orientation;
            }

            fn translate(&mut self, translation: &Translation3<f64>) {
                self.position = translation.transform_point(&self.position);
            }

            fn rotate(&mut self, rotation: &UnitQuaternion<f64>) {
                self.orientation = rotation * &self.orientation;
            }
        }
    };
}

#[macro_export]
macro_rules! compute_in_local {
    ($func: ident, $points: expr, ($($func_args:expr),*), $position: expr, $orientation: expr) => {
        global_vectors(&$func(&local_points($points, $position, $orientation), $($func_args),*)?, $orientation)
    };
}

/// Transform global point to the local frame of the object
pub fn local_point(
    point: &Point3<f64>,
    position: &Point3<f64>,
    orientation: &UnitQuaternion<f64>,
) -> Point3<f64> {
    orientation.inverse() * Point3::from(point.coords - position.coords)
}

pub fn local_points(
    points: &[Point3<f64>],
    position: &Point3<f64>,
    orientation: &UnitQuaternion<f64>,
) -> Vec<Point3<f64>> {
    points
        .iter()
        .map(|point| local_point(point, position, orientation))
        .collect()
}

/// Transform local vector to the global frame
pub fn global_vector(vector: &Vector3<f64>, orientation: &UnitQuaternion<f64>) -> Vector3<f64> {
    orientation * vector
}

pub fn global_vectors(
    local_vectors: &[Vector3<f64>],
    orientation: &UnitQuaternion<f64>,
) -> Vec<Vector3<f64>> {
    local_vectors
        .iter()
        .map(|local_vector| global_vector(local_vector, orientation))
        .collect()
}
