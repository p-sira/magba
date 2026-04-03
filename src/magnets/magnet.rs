/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

#[cfg(feature = "std")]
use derive_more::Display;

use enum_dispatch::enum_dispatch;

use crate::{
    base::{Float, Pose, Source, Transform},
    magnets::{
        CuboidMagnet, CylinderMagnet, Dipole, MeshMagnet, SphereMagnet, TetrahedronMagnet,
        TriangleMagnet,
    },
};
use nalgebra::{Point3, Vector3};

/// Magnetic source variants.
///
/// The Magnet enum is useful for wrapping different magnet structs before using them
/// in stack-allocated composites. See [SourceArray](crate::collections::SourceArray#examples).
///
/// # Examples
///
/// ```
/// # use magba::prelude::*;
/// # use nalgebra::{point, vector};
/// # use approx::assert_relative_eq;
/// let magnet: Magnet = CylinderMagnet::default().into();
/// let b_field = magnet.compute_B(point![0.1, 0.2, 0.3]);
/// assert_relative_eq!(b_field, vector![0.03358623061457353, 0.06717246122914705, 0.6376649834015807]);
/// ```
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "std", derive(Display))]
#[enum_dispatch(Source<T>, Transform<T>,)]
pub enum Magnet<T: Float = f64> {
    Cylinder(CylinderMagnet<T>),
    Cuboid(CuboidMagnet<T>),
    Dipole(Dipole<T>),
    Sphere(SphereMagnet<T>),
    Tetrahedron(TetrahedronMagnet<T>),
    Triangle(TriangleMagnet<T>),
    Mesh(MeshMagnet<T>),
}

#[cfg(all(test, feature = "std"))]
mod tests {
    use super::*;

    #[test]
    fn test_display() {
        let magnet: Magnet = CylinderMagnet::default().into();
        assert_eq!(
            format!("{}", magnet),
            "CylinderMagnet (pol=[0.0, 0.0, 1.0], d=1.0, h=1.0) at pos=[0.0, 0.0, 0.0], rot=[0.0, 0.0, 0.0]"
        );
    }
}
