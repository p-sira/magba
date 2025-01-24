/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Magnetic source structs.
//!
//! Permanent magnets:
//! - [CylinderMagnet]
//!
//! Magnetic source collection:
//! - [SourceCollection]
//! - [MultiSourceCollection]

mod cylinder;
mod source;

pub use cylinder::*;
pub use source::*;
