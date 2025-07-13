/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use crate::sources::magnets::define_magnet;
use nalgebra::Vector3;

define_magnet! {
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
    CuboidMagnet
    field_fn: cuboid_B
    args: {polarization:Vector3<T>, dimensions:Vector3<T>}
    arg_display: "pol={}, dim={}";
    arg_fmt: [format_vector3, format_vector3]
    on_new: [
        dimensions.iter().for_each(|&elem| {
            if elem < T::zero() {
                panic!("Dimensions must be non-negative.")
            }
        });
    ]
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use nalgebra::{Point3, Translation3};

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
            "CuboidMagnet (pol=[0, 0, 0], dim=[0, 0, 0]) at pos=[0, 0, 0], q=[0, 0, 0, 1]",
            format!("{}", magnet)
        );
    }
}
