/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Defines the [`CuboidMagnet`] struct, representing a uniformly magnetized cylindrical magnet in 3D space.

use getset::{Getters, Setters};
use nalgebra::{Point3, Translation3, UnitQuaternion, Vector3};

use super::{Field, Source};
use crate::crate_util;
use crate::geometry::Transform;
use crate::{fields, impl_transform, Float};

use std::fmt::Display;

/// Uniformly magnetized cylindrical magnet in 3D space.
///
/// # Fields
/// - `position`: Center of the cuboid (m)
/// - `orientation`: Orientation as unit quaternion
/// - `polarization`: Polarization vector (T)
/// - `dimensions`: Cuboid side lengths (m)
///
/// # Example
/// ```
/// use magba::sources::CuboidMagnet;
/// use nalgebra::{Point3, UnitQuaternion, Vector3};
///
/// let magnet = CuboidMagnet::new(
///     Point3::origin(),
///     UnitQuaternion::identity(),
///     Vector3::z(),
///     Vector3::new(0.01, 0.01, 0.02)
/// );
/// ```
#[derive(Debug, Clone, PartialEq, Default, Getters, Setters)]
pub struct CuboidMagnet<T: Float> {
    /// Center of the cuboid (m)
    position: Point3<T>,
    /// Orientation as a unit quaternion
    orientation: UnitQuaternion<T>,
    /// Polarization vector (T)
    #[getset(get = "pub", set = "pub")]
    polarization: Vector3<T>,

    /// Cuboid dimensions (m)
    #[getset(get = "pub", set = "pub")]
    dimensions: Vector3<T>,
}

impl<T: Float> CuboidMagnet<T> {
    /// Create a new [`CuboidMagnet`].
    ///
    /// # Arguments
    /// - `position`: Center of the cuboid (m)
    /// - `orientation`: Orientation as unit quaternion
    /// - `polarization`: Polarization vector (T)
    /// - `dimensions`: Cuboid side lengths (m)
    ///
    /// # Returns
    /// - `CuboidMagnet` instance
    ///
    /// # Example
    /// ```
    /// use magba::sources::CuboidMagnet;
    /// use nalgebra::{Point3, UnitQuaternion, Vector3};
    ///
    /// let magnet = CuboidMagnet::new(
    ///     Point3::origin(),
    ///     UnitQuaternion::identity(),
    ///     Vector3::z(),
    ///     Vector3::new(0.01, 0.01, 0.02)
    /// );
    /// ```
    pub fn new(
        position: Point3<T>,
        orientation: UnitQuaternion<T>,
        polarization: Vector3<T>,
        dimensions: Vector3<T>,
    ) -> Self {
        dimensions.iter().for_each(|&elem| {
            if elem < T::zero() {
                panic!("CuboidMagnet: Dimensions must be non-negative.")
            }
        });
        CuboidMagnet {
            position,
            orientation,
            polarization,
            dimensions,
        }
    }
}

impl<T: Float> Source<T> for CuboidMagnet<T> {}

impl<T: Float> Transform<T> for CuboidMagnet<T> {
    impl_transform!();
}

impl<T: Float> Field<T> for CuboidMagnet<T> {
    fn get_B(&self, points: &[Point3<T>]) -> Vec<Vector3<T>> {
        fields::cuboid_B(
            points,
            &self.position,
            &self.orientation,
            &self.dimensions,
            &self.polarization,
        )
    }
}

impl<T: Float> Display for CuboidMagnet<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "CuboidMagnet (dim={}, pol={}) at pos={}, q={}",
            crate_util::format_vector3!(self.dimensions),
            crate_util::format_vector3!(self.polarization),
            crate_util::format_point3!(self.position),
            crate_util::format_quat!(self.orientation)
        )
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use nalgebra::Translation3;

    use crate::geometry::Transform;
    use crate::sources::CuboidMagnet;
    use crate::testing_util::*;

    use super::*;

    #[test]
    fn test_cuboid() {
        let magnet = CuboidMagnet::new(
            Point3::new(0.05, 0.1, 0.2),
            quat_from_rotvec(PI / 7.0, PI / 6.0, PI / 5.0),
            Vector3::new(1.0, 2.0, 3.0),
            Vector3::new(0.1, 0.2, 0.3),
        );
        test_B_magnet!(&magnet, "cuboid.csv", 2e-12);
    }

    #[test]
    fn test_cuboid_small() {
        let magnet = CuboidMagnet::new(
            Point3::new(0.03, 0.02, 0.01),
            quat_from_rotvec(PI / 8.0, PI / 7.0, PI / 6.0),
            Vector3::new(0.15, 0.15, 0.3),
            Vector3::new(5e-3, 3e-3, 1e-3),
        );
        // Small magnet at far distances have less numerical stability
        test_B_magnet!(@small, &magnet, "cuboid-small.csv", 2e-10);
    }

    #[test]
    fn test_translate_cuboid() {
        let mut magnet = CuboidMagnet::new(
            Point3::new(0.05, 0.1, 0.2),
            quat_from_rotvec(PI / 7.0, PI / 6.0, PI / 5.0),
            Vector3::new(1.0, 2.0, 3.0),
            Vector3::new(0.1, 0.2, 0.3),
        );
        magnet.translate(&Translation3::new(-0.1, -0.2, -0.3));
        test_B_magnet!(&magnet, "cuboid-translate.csv", 1e-12);
    }

    #[test]
    fn test_rotate_cuboid() {
        let rotation = quat_from_rotvec(PI / 7.0, PI / 6.0, PI / 5.0);
        let mut magnet = CuboidMagnet::new(
            Point3::new(0.05, 0.1, 0.2),
            rotation,
            Vector3::new(1.0, 2.0, 3.0),
            Vector3::new(0.1, 0.2, 0.3),
        );
        magnet.rotate(&rotation.inverse());
        test_B_magnet!(&magnet, "cuboid-rotate.csv", 1e-12);
    }

    #[test]
    fn test_rotate_translate_cuboid() {
        let mut magnet = CuboidMagnet::new(
            Point3::new(0.05, 0.1, 0.2),
            quat_from_rotvec(PI / 7.0, PI / 6.0, PI / 5.0),
            Vector3::new(1.0, 2.0, 3.0),
            Vector3::new(0.1, 0.2, 0.3),
        );
        magnet.translate(&Translation3::new(0.3, 0.2, 0.1));
        magnet.rotate(&quat_from_rotvec(PI / 3.0, PI / 2.0, PI));
        test_B_magnet!(&magnet, "cuboid-rotate-translate.csv", 2e-12);
    }

    #[test]
    fn test_cuboid_display() {
        let magnet = CuboidMagnet::<f64>::default();
        assert_eq!(
            "CuboidMagnet (dim=[0, 0, 0], pol=[0, 0, 0]) at pos=[0, 0, 0], q=[0, 0, 0, 1]",
            format!("{}", magnet)
        );
    }
}
