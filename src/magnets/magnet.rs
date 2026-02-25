/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use crate::{
    Source,
    base::*,
    magnets::{CuboidMagnet, CylinderMagnet, Dipole, ZeroMagnet},
};

#[cfg(feature = "std")]
use derive_more::Display;

use enum_dispatch::enum_dispatch;

/// Magnetic source variants.
///
/// ### Examples
///
/// ```
/// # use magba::*;
/// # use nalgebra::{point, vector};
/// # use approx::assert_relative_eq;
/// let magnet: Magnet = CylinderMagnet::default().into();
/// let b_field = magnet.compute_B(point![0.1, 0.2, 0.3]);
/// assert_relative_eq!(b_field, vector![0.03358623061457353, 0.06717246122914705, 0.6376649834015807]);
/// ```
#[derive(Clone, Debug)]
#[cfg_attr(feature = "std", derive(Display))]
#[enum_dispatch(Source<T>, Transform<T>, Field<T>)]
pub enum Magnet<T: Float = f64> {
    Cylinder(CylinderMagnet<T>),
    Cuboid(CuboidMagnet<T>),
    Dipole(Dipole<T>),
    Zero(ZeroMagnet<T>),

    Custom(Box<dyn Source<T>>),
}

impl<T: Float> PartialEq for Magnet<T> {
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (Self::Cylinder(l0), Self::Cylinder(r0)) => l0 == r0,
            (Self::Cuboid(l0), Self::Cuboid(r0)) => l0 == r0,
            (Self::Dipole(l0), Self::Dipole(r0)) => l0 == r0,
            (Self::Zero(l0), Self::Zero(r0)) => l0 == r0,
            (Self::Custom(_), Self::Custom(_)) => false,
            _ => false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[cfg(feature = "std")]
    #[test]
    fn test_display() {
        let magnet: Magnet = CylinderMagnet::default().into();
        assert_eq!(
            format!("{}", magnet),
            "CylinderMagnet (pol=[0, 0, 1], d=1, h=1) at pos=[0.0, 0.0, 0.0], r=[0.0, 0.0, 0.0]"
        );
    }
}
