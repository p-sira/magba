/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use enum_dispatch::enum_dispatch;

use crate::{
    CuboidMagnet, CylinderMagnet, Dipole, ZeroMagnet,
    core::Float,
    geometry::{Pose, Transform},
    magnets::Magnet,
};

#[derive(Debug, Clone, PartialEq)]
#[enum_dispatch(Source<T>, Transform<T>, Field<T>)]
/// High-level component type used by collections.
pub enum Component<T: Float> {
    Magnet(Magnet<T>),
}

macro_rules! impl_transitive_from {
    ($($primitive:ident),*) => {
        $(
            impl<T: Float> From<$primitive<T>> for Component<T> {
                fn from(p: $primitive<T>) -> Self {
                    // 1. Wrap primitive in Magnet
                    let intermediate: Magnet<T> = p.into();
                    // 2. Wrap Magnet in Component
                    intermediate.into()
                }
            }
        )*
    };
}

impl_transitive_from!(CylinderMagnet, CuboidMagnet, Dipole, ZeroMagnet);
