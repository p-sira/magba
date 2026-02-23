/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use enum_dispatch::enum_dispatch;

use crate::{
    Collection,
    base::Float,
    magnets::{CuboidMagnet, CylinderMagnet, Dipole, Magnet, ZeroMagnet},
};

#[derive(Debug, Clone, PartialEq)]
#[enum_dispatch(Source<T>, Transform<T>, Field<T>)]
/// Components that can be grouped into collections.
///
/// ```
/// # use magba::*;
/// let magnet: Component = CylinderMagnet::default().into();
/// let collection = Collection::from(magnet);
/// ```
pub enum Component<T: Float = f64> {
    Magnet(Magnet<T>),

    Collection(Collection<T>),
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

impl<T: Float> Eq for Component<T> {}
