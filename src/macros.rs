/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

#[macro_export]
macro_rules! collection {
    // Heap-allocated homogenous collection
    // collection!(magnet1, magnet2, ...)
    ($($items:expr),* $(,)?) => {
        magba::sources::Collection::from_sources(vec![$($items, )*])
    };
    // Heap-allocated heterogenous collection
    // collection!(boxed: magnet1, magnet2, ...)
    (boxed: $($items:expr),* $(,)?) => {
        magba::sources::BoxedCollection::from_sources(vec![$($items, )*])
    };
    // Stack-allocated collection
    // collection!([magnet1, magnet2, ...])
    ([$($items:expr),* $(,)?]) => {
        {
            let sources = [$($items, )*];
            magba::sources::SCollection::from_sources(sources)
        }
    };
}
