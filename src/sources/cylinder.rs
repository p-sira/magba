/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Defines the [`CylinderMagnet`] struct, representing a uniformly magnetized cylindrical magnet in 3D space.

use getset::{Getters, Setters};
use nalgebra::{Point3, Translation3, UnitQuaternion, Vector3};

use super::{Field, Source};
use crate::crate_util;
use crate::geometry::Transform;
use crate::{fields, impl_transform};

use std::fmt::Display;

/// Uniformly magnetized cylindrical magnet in 3D space.
///
/// # Fields
/// - `position`: Center of the cylinder (m)
/// - `orientation`: Orientation as a unit quaternion
/// - `polarization`: Polarization vector (T)
/// - `radius`: Cylinder radius (m)
/// - `height`: Cylinder height (m)
///
/// # Example
/// ```
/// use magba::sources::CylinderMagnet;
/// use nalgebra::{Point3, UnitQuaternion, Vector3};
///
/// let magnet = CylinderMagnet::new(
///     Point3::origin(),
///     UnitQuaternion::identity(),
///     Vector3::z(),
///     0.005,
///     0.02,
/// );
/// ```
#[derive(Debug, Clone, PartialEq, Default, Getters, Setters)]
pub struct CylinderMagnet {
    /// Center of the cylinder (m)
    position: Point3<f64>,
    /// Orientation as a unit quaternion
    orientation: UnitQuaternion<f64>,
    /// Polarization vector (T)
    #[getset(get = "pub", set = "pub")]
    polarization: Vector3<f64>,

    /// Cylinder radius (m)
    #[getset(get = "pub", set = "pub")]
    radius: f64,
    /// Cylinder height (m)
    #[getset(get = "pub", set = "pub")]
    height: f64,
}

impl CylinderMagnet {
    /// Create a new [`CylinderMagnet`].
    ///
    /// # Arguments
    /// - `position`: Center of the cylinder (m)
    /// - `orientation`: Orientation as a unit quaternion
    /// - `polarization`: Polarization vector (T)
    /// - `radius`: Cylinder radius (m)
    /// - `height`: Cylinder height (m)
    ///
    /// # Returns
    /// - `CylinderMagnet` instance
    ///
    /// # Example
    /// ```
    /// use magba::sources::CylinderMagnet;
    /// use nalgebra::{Point3, UnitQuaternion, Vector3};
    ///
    /// let magnet = CylinderMagnet::new(
    ///     Point3::origin(),
    ///     UnitQuaternion::identity(),
    ///     Vector3::z(),
    ///     0.005,
    ///     0.02,
    /// );
    /// ```
    pub fn new(
        position: Point3<f64>,
        orientation: UnitQuaternion<f64>,
        polarization: Vector3<f64>,
        radius: f64,
        height: f64,
    ) -> Self {
        CylinderMagnet {
            position,
            orientation,
            polarization,
            radius,
            height,
        }
    }
}

impl Source for CylinderMagnet {}

impl_transform!(CylinderMagnet);

impl Field for CylinderMagnet {
    fn get_B(&self, points: &[Point3<f64>]) -> Result<Vec<Vector3<f64>>, &'static str> {
        fields::cyl_B(
            points,
            &self.position,
            &self.orientation,
            self.radius,
            self.height,
            &self.polarization,
        )
    }
}

impl Display for CylinderMagnet {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "CylinderMagnet (r={}, h={}, pol={}) at pos={}, q={}",
            self.radius,
            self.height,
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
        let magnet = CylinderMagnet::default();
        assert_eq!(
            "CylinderMagnet (r=0, h=0, pol=[0, 0, 0]) at pos=[0, 0, 0], q=[0, 0, 0, 1]",
            format!("{}", magnet)
        );
    }
}
