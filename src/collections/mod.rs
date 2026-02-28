/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Data structures for grouping and management of magnetic components.

mod collection;
mod component;
mod node;
mod source_array;

pub use collection::Collection;
pub use component::Component;
pub use node::Node;
pub use source_array::SourceArray;

mod utils;