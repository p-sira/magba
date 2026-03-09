/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use enum_dispatch::enum_dispatch;

use crate::{
    base::{Float, Pose, Source, Transform},
    collections::{SourceArray, SourceAssembly},
    currents::{CircularCurrent, Current},
    magnets::{CuboidMagnet, CylinderMagnet, Dipole, Magnet, SphereMagnet},
};
use nalgebra::{Point3, Vector3};

#[derive(Debug, Clone)]
#[enum_dispatch(Source<T>, Transform<T>,)]
/// [Source] components that can be grouped into collections.
///
/// ```
/// # use magba::sources;
/// # use magba::prelude::*;
/// let magnet: SourceComponent = CylinderMagnet::default().into();
/// let sources = sources!(magnet);
/// ```
pub enum SourceComponent<T: Float = f64> {
    Magnet(Magnet<T>),
    Current(Current<T>),
    Assembly(SourceAssembly<T>),
    Custom(Box<dyn Source<T>>),
}

impl<T: Float> PartialEq for SourceComponent<T> {
    /// ```
    /// # use magba::prelude::*;
    /// # use magba::sources;
    /// let cylinder = CylinderMagnet::default();
    /// let cylinder2 = CylinderMagnet::default().with_polarization([1.0, 2.0, 3.0]);
    /// assert_eq!(cylinder, cylinder.clone());
    /// assert_ne!(cylinder, cylinder2);
    ///
    /// let current = CircularCurrent::default();
    /// let current2 = CircularCurrent::<f64>::default().with_current(2.0);
    /// assert_eq!(current, current.clone());
    /// assert_ne!(current, current2);
    ///
    /// let assembly = sources!(cylinder, current);
    /// let assembly2 = sources!(cylinder2, current2);
    /// assert_eq!(assembly, assembly.clone());
    /// assert_ne!(assembly, assembly2);
    ///
    /// // Custom types always return unequal.
    /// let custom = SourceComponent::<f64>::Custom(Box::new(CuboidMagnet::default()));
    /// assert_ne!(custom, custom.clone());
    /// ```
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (Self::Magnet(l0), Self::Magnet(r0)) => l0 == r0,
            (Self::Current(l0), Self::Current(r0)) => l0 == r0,
            (Self::Assembly(l0), Self::Assembly(r0)) => l0 == r0,
            (Self::Custom(_), Self::Custom(_)) => false,
            _ => false,
        }
    }
}

macro_rules! impl_transitive_from_magnet {
    ($($primitive:ident),*) => {
        $(
            impl<T: Float> From<$primitive<T>> for SourceComponent<T> {
                fn from(p: $primitive<T>) -> Self {
                    let intermediate: Magnet<T> = p.into();
                    intermediate.into()
                }
            }
        )*
    };
}

macro_rules! impl_transitive_from_current {
    ($($primitive:ident),*) => {
        $(
            impl<T: Float> From<$primitive<T>> for SourceComponent<T> {
                fn from(p: $primitive<T>) -> Self {
                    let intermediate: Current<T> = p.into();
                    intermediate.into()
                }
            }
        )*
    };
}

impl_transitive_from_magnet!(CylinderMagnet, CuboidMagnet, Dipole, SphereMagnet);
impl_transitive_from_current!(CircularCurrent);

impl<T: Float> Eq for SourceComponent<T> {}

impl<S: Source<T>, const N: usize, T: Float> From<SourceArray<S, N, T>> for SourceComponent<T>
where
    SourceComponent<T>: From<S>,
{
    fn from(value: SourceArray<S, N, T>) -> Self {
        SourceAssembly::from(value).into()
    }
}
