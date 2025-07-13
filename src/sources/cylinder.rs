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
mod tests {
    use std::f64::consts::PI;

    use nalgebra::{Point3, Translation3, UnitQuaternion};

    use crate::geometry::Transform;
    use crate::sources::CylinderMagnet;
    use crate::testing_util::*;

    use super::*;

    #[test]
    fn test_cylinder() {
        let magnet = CylinderMagnet::new(
            Point3::new(0.1, 0.2, 0.3),
            quat_from_rotvec(PI / 7.0, PI / 6.0, PI / 5.0),
            Vector3::new(1.0, 2.0, 3.0),
            0.5,
            2.0,
        );
        test_B_magnet!(@large, &magnet, "cylinder.csv", 6e-11);
    }

    #[test]
    fn test_cylinder_small() {
        let magnet = CylinderMagnet::new(
            Point3::new(0.03, 0.02, 0.01),
            quat_from_rotvec(PI / 8.0, PI / 7.0, PI / 6.0),
            Vector3::new(0.15, 0.15, 0.3),
            14e-3,
            10e-3,
        );
        // Small magnet at far distances have less numerical stability
        test_B_magnet!(&magnet, "cylinder-small.csv", 2e-8);
    }

    #[test]
    fn test_translate_cylinder() {
        let mut magnet = CylinderMagnet::new(
            Point3::new(0.1, 0.2, 0.3),
            quat_from_rotvec(PI / 7.0, PI / 6.0, PI / 5.0),
            Vector3::new(1.0, 2.0, 3.0),
            0.5,
            2.0,
        );
        magnet.translate(&Translation3::new(-0.1, -0.2, -0.3));
        test_B_magnet!(@large, &magnet, "cylinder-translate.csv", 6e-11);
    }

    #[test]
    fn test_rotate_cylinder() {
        let rotation = quat_from_rotvec(PI / 7.0, PI / 6.0, PI / 5.0);
        let mut magnet = CylinderMagnet::new(
            Point3::new(0.1, 0.2, 0.3),
            rotation,
            Vector3::new(1.0, 2.0, 3.0),
            0.5,
            2.0,
        );
        magnet.rotate(&rotation.inverse());
        test_B_magnet!(@large, &magnet, "cylinder-rotate.csv", 6e-11);
    }

    #[test]
    fn test_rotate_translate_cylinder() {
        let mut magnet = CylinderMagnet::new(
            Point3::new(0.1, 0.2, 0.3),
            quat_from_rotvec(PI / 7.0, PI / 6.0, PI / 5.0),
            Vector3::new(1.0, 2.0, 3.0),
            0.5,
            2.0,
        );
        magnet.translate(&Translation3::new(3.0, 2.0, 1.0));
        magnet.rotate(&quat_from_rotvec(PI / 3.0, PI / 2.0, PI));
        test_B_magnet!(@large, &magnet, "cylinder-rotate-translate.csv", 6e-11);
    }

    #[test]
    fn test_axial_cylinder() {
        let magnet = CylinderMagnet::new(
            Point3::origin(),
            UnitQuaternion::identity(),
            Vector3::new(0.0, 0.0, 3.0),
            0.5,
            2.0,
        );
        test_B_magnet!(@large, &magnet, "cylinder-axial.csv", 1e-12);
    }

    #[test]
    fn test_diametric_cylinder() {
        let magnet = CylinderMagnet::new(
            Point3::origin(),
            UnitQuaternion::identity(),
            Vector3::new(0.0, 1.0, 0.0),
            0.5,
            2.0,
        );
        test_B_magnet!(@large, &magnet, "cylinder-diametric.csv", 2e-12);
    }

    #[test]
    fn test_diametric_cylinder_2() {
        let magnet = CylinderMagnet::new(
            Point3::origin(),
            UnitQuaternion::identity(),
            Vector3::new(2.0, 1.0, 0.0),
            0.5,
            2.0,
        );
        test_B_magnet!(@large, &magnet, "cylinder-diametric-2.csv", 5e-12);
    }

    #[test]
    fn test_cylinder_display() {
        let magnet = CylinderMagnet::<f64>::default();
        assert_eq!(
            "CylinderMagnet (pol=[0, 0, 0], r=0, h=0) at pos=[0, 0, 0], q=[0, 0, 0, 1]",
            format!("{}", magnet)
        );
    }
}
