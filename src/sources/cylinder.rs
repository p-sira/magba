/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use nalgebra::Vector3;

use crate::sources::magnets::define_magnet;

define_magnet! {
    /// Uniformly magnetized cylindrical magnet in 3D space.
    ///
    /// # Fields
    /// - `position`: Center of the cylinder (m)
    /// - `orientation`: Orientation as unit quaternion
    /// - `polarization`: Polarization vector (T)
    /// - `radius`: Cylinder radius (m)
    /// - `height`: Cylinder height (m)
    ///
    /// # Example
    /// ```
    /// use magba::sources::CylinderMagnet;
    /// use nalgebra::*;
    ///
    /// let magnet = CylinderMagnet::new(
    ///     Point3::origin(),           // position (m)
    ///     UnitQuaternion::identity(), // orientation
    ///     Vector3::z(),               // polarization (T)
    ///     0.005,                      // radius (m)
    ///     0.02,                       // height (m)
    /// );
    /// ```
    CylinderMagnet
    field_fn: cylinder_B
    args: {polarization:Vector3<T> = Vector3::z(), radius v:T = T::one(), height v:T = T::one()}
    arg_display: "pol={}, r={}, h={}";
    arg_fmt: [format_vector3, format_float, format_float]
    on_new: [
        if radius < T::zero() || height < T::zero() {
            panic!("Radius and height cannot be negative.")
        }
    ]
}

#[cfg(test)]
crate::testing_util::generate_tests! {
    CylinderMagnet
    filename: cylinder
    params: { polarization: Vector3::new(1.0, 2.0, 3.0), r: 0.05, h: 0.2}
    rtols: {
        static: 5e-10,
        static_small: 5e-10,
        translate: 2e-10,
        rotate: 2e-10,
    }
}
