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
    ///     Point3::origin(),
    ///     UnitQuaternion::identity(),
    ///     Vector3::z(),
    ///     0.005,
    ///     0.02,
    /// );
    /// ```
    CylinderMagnet
    field_fn: cylinder_B
    args: {polarization:Vector3<T>, radius v:T, height v:T}
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

#[cfg(test)]
mod extra_tests {
    use super::*;

    #[test]
    fn test_cylinder_display() {
        let magnet = CylinderMagnet::<f64>::default();
        assert_eq!(
            "CylinderMagnet (pol=[0, 0, 0], r=0, h=0) at pos=[0, 0, 0], q=[0, 0, 0, 1]",
            format!("{}", magnet)
        );
    }
}
