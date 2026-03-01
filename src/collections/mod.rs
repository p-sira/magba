/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Data structures for grouping and management of magnetic components.

mod macros;
#[cfg(test)]
pub(crate) use macros::sources;

mod component;
mod node;
mod source_array;
mod source_assembly;

pub use component::Component;
pub use node::Node;
pub use source_array::SourceArray;
pub use source_assembly::SourceAssembly;

mod utils;
