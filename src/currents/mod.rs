/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Current source geometries that generate magnetic fields.

mod circular;
mod current;
mod path;
#[cfg(feature = "mesh")]
mod sheet;
mod triangle;

pub use circular::CircularCurrent;
pub use current::Current;
pub use path::PathCurrent;
#[cfg(feature = "mesh")]
pub use sheet::SheetCurrent;
pub use triangle::TriangleCurrent;
