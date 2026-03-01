/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

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
