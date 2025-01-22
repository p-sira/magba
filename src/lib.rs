/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

mod geometry;
mod special;
mod util;

pub mod constants;
pub mod fields;

#[cfg(feature = "sources")]
pub mod sources;

#[cfg(test)]
mod testing_util;