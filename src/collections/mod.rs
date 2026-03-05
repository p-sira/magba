/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Data structures for grouping and management of magnetic components.

mod macros;
#[cfg(test)]
pub(crate) use macros::observers;
#[cfg(test)]
pub(crate) use macros::sources;

mod node;
mod sensor_array;
mod sensor_assembly;
mod sensor_component;
mod source_array;
mod source_assembly;
mod source_component;

use node::Node;
pub use sensor_array::SensorArray;
pub use sensor_assembly::SensorAssembly;
pub use sensor_component::SensorComponent;
pub use source_array::SourceArray;
pub use source_assembly::SourceAssembly;
pub use source_component::SourceComponent;

mod utils;
