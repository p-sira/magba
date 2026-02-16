/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Magnetic source traits and structs.

mod define_magnet;

pub mod collections;
pub mod magnets;
mod source;

pub use collections::*;
pub use magnets::*;
pub use source::{Field, Source};
