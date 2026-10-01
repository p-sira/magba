/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use crate::base::{Float, Pose, Source};
use nalgebra::{Point3, Vector3};

#[derive(Clone)]
pub struct StableFieldMagnet<T: Float = f64> {
    pose: Pose<T>,
    b_field: Vector3<T>,
}

impl<T: Float> StableFieldMagnet<T> {
    pub fn new(b_field: Vector3<T>) -> Self {
        Self {
            pose: Default::default(),
            b_field,
        }
    }
}

crate::base::transform::impl_transform!(StableFieldMagnet<T> where T: Float);

impl<T: Float> Source<T> for StableFieldMagnet<T> {
    #[allow(non_snake_case)]
    fn compute_B(&self, _point: Point3<T>) -> Vector3<T> {
        self.b_field
    }

    #[allow(non_snake_case)]
    #[cfg(feature = "alloc")]
    fn compute_B_batch(&self, points: &[Point3<T>]) -> alloc::vec::Vec<Vector3<T>> {
        points.iter().map(|_| self.b_field).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_stable_field_batch() {
        let magnet = StableFieldMagnet::new(Vector3::new(1.0, 2.0, 3.0));
        let batch = magnet.compute_B_batch(&[Point3::origin(), Point3::origin()]);
        assert_eq!(batch.len(), 2);
        assert_eq!(batch[0], Vector3::new(1.0, 2.0, 3.0));
    }
}
