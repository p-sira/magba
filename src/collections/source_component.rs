/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use enum_dispatch::enum_dispatch;

use crate::{
    base::{Float, Pose, Source, Transform},
    collections::{SourceArray, SourceAssembly},
    magnets::{CuboidMagnet, CylinderMagnet, Dipole, Magnet},
};
use nalgebra::{Point3, Vector3};

#[derive(Debug, Clone)]
#[enum_dispatch(Source<T>, Transform<T>,)]
/// Components that can be grouped into collections.
///
/// ```
/// # use magba::sources;
/// # use magba::prelude::*;
/// let magnet: SourceComponent = CylinderMagnet::default().into();
/// let sources = sources!(magnet);
/// ```
pub enum SourceComponent<T: Float = f64> {
    Magnet(Magnet<T>),
    Assembly(SourceAssembly<T>),
    Custom(Box<dyn Source<T>>),
}

impl<T: Float> PartialEq for SourceComponent<T> {
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (Self::Magnet(l0), Self::Magnet(r0)) => l0 == r0,
            (Self::Assembly(l0), Self::Assembly(r0)) => l0 == r0,
            (Self::Custom(_), Self::Custom(_)) => false,
            _ => false,
        }
    }
}

macro_rules! impl_transitive_from {
    ($($primitive:ident),*) => {
        $(
            impl<T: Float> From<$primitive<T>> for SourceComponent<T> {
                fn from(p: $primitive<T>) -> Self {
                    // 1. Wrap primitive in Magnet
                    let intermediate: Magnet<T> = p.into();
                    // 2. Wrap Magnet in SourceComponent
                    intermediate.into()
                }
            }
        )*
    };
}

impl_transitive_from!(CylinderMagnet, CuboidMagnet, Dipole);

impl<T: Float> Eq for SourceComponent<T> {}

impl<S: Source<T>, const N: usize, T: Float> From<SourceArray<S, N, T>> for SourceComponent<T>
where
    SourceComponent<T>: From<S>,
{
    fn from(value: SourceArray<S, N, T>) -> Self {
        SourceAssembly::from(value).into()
    }
}
