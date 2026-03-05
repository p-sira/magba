/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Data structures for grouping and management of magnetic components.
//!
//! This module provides two primary types of collections for both magnetic sources and sensors: **Arrays** and **Assemblies**.
//!
//! # Arrays vs. Assemblies
//!
//! | Feature | Array (`SourceArray`, `ObserverArray`) | Assembly (`SourceAssembly`, `ObserverAssembly`) |
//! |---------|--------------------------------------|-----------------------------------------------|
//! | **Allocation** | Stack-allocated | Heap-allocated |
//! | **Capacity** | Fixed-size (`const N: usize`) | Dynamically-sized (`Vec`) |
//! | **Components** | Uniform type (unless wrapped in an enum like `Magnet`) | Heterogeneous |
//! | **Nesting** | Not supported | Supported (only for `SourceAssembly`) |
//! | **Custom Types** | Not supported | Supported |
//!
//! # Convenience Macros
//!
//! Instead of manually creating collections, you can use the [`sources!`](crate::sources) and [`observers!`](crate::observers) macros.
//! These macros provide a familiar, `vec!`-like syntax to quickly build up arrays and assemblies.

mod macros;
#[cfg(test)]
pub(crate) use macros::observers;
#[cfg(test)]
pub(crate) use macros::sources;

mod node;
mod observer_array;
mod observer_assembly;
mod observer_component;
mod source_array;
mod source_assembly;
mod source_component;

use node::Node;
pub use observer_array::ObserverArray;
pub use observer_assembly::ObserverAssembly;
pub use observer_component::ObserverComponent;
pub use source_array::SourceArray;
pub use source_assembly::SourceAssembly;
pub use source_component::SourceComponent;

mod utils;
