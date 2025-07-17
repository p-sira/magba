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
    ///     Point3::origin(),              // position (m)
    ///     UnitQuaternion::identity(),    // orientation
    ///     Vector3::z(),                  // polarization (T)
    ///     Vector3::new(0.01, 0.01, 0.02) // dimensions (m)
    /// );
    /// ```
    /// # References
    /// - Ortner, Michael, and Lucas Gabriel Coliado Bandeira. “Magpylib: A Free Python Package for Magnetic Field Computation.” SoftwareX 11 (January 1, 2020): 100466. <https://doi.org/10.1016/j.softx.2020.100466>.
    CuboidMagnet
    field_fn: cuboid_B
    args: {polarization:Vector3<T> = Vector3::z(), dimensions:Vector3<T> = Vector3::from_element(T::one())}
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
