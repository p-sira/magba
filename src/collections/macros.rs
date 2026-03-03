/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

/// Constructs a source composite.
///
/// # Examples
///
/// ```
/// # use magba::*;
/// let cylinder = CylinderMagnet::default();
/// let cuboid = CuboidMagnet::default();
/// let dipole = Dipole::default();
/// let dipole2 = Dipole::default().with_moment([1.0, 0.0, 0.0]);
///
/// let assembly: SourceAssembly = sources!(cylinder, cuboid, dipole);
///
/// let homogenous_array: SourceArray<Dipole, _> = sources!([dipole, dipole2]);
/// let heterogenous_array: SourceArray<Magnet, _> = 
///     sources!([cylinder.into(), cuboid.into(), dipole.into()]);
/// ```
#[macro_export]
macro_rules! sources {
    // sources!([magnet1, magnet2, ...])
    ([$($items:expr),*]) => {
        SourceArray::new([0.0; 3], nalgebra::UnitQuaternion::identity(), [$($items),*])
    };
    // sources!(magnet1, magnet2, ...)
    ($($items:expr),* $(,)?) => {{
        let c: [SourceComponent<_>; _] = [$($items.into()),*];
        SourceAssembly::from(c)
    }};
    () => { SourceAssembly::default() };
}
#[cfg(test)]
pub(crate) use sources;
