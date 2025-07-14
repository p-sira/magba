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
    /// use nalgebra::*;
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
crate::testing_util::generate_tests! {
    CuboidMagnet
    filename: cuboid
    params: { polarization: Vector3::new(1.0, 2.0, 3.0), dimensions: Vector3::new(0.1, 0.2, 0.3)}
    rtols: {
        static: 2e-10,
        static_small: 2e-10,
        translate: 2e-10,
        rotate: 2e-10,
    }
}

#[cfg(test)]
mod extra_tests {
    use super::*;

    #[test]
    fn test_cuboid_display() {
        let magnet = CuboidMagnet::<f64>::default();
        assert_eq!(
            "CuboidMagnet (pol=[0, 0, 0], dim=[0, 0, 0]) at pos=[0, 0, 0], q=[0, 0, 0, 1]",
            format!("{}", magnet)
        );
    }
}
