/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use enum_dispatch::enum_dispatch;

use crate::{CuboidMagnet, CylinderMagnet, Dipole, ZeroMagnet, core::Float};

#[derive(Clone, Debug, PartialEq)]
#[enum_dispatch(Source<T>, Transform<T>, Field<T>)]
pub enum Magnet<T: Float> {
    Cylinder(CylinderMagnet<T>),
    Cuboid(CuboidMagnet<T>),
    Dipole(Dipole<T>),
    Zero(ZeroMagnet<T>),
}
