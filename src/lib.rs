/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

#![cfg_attr(not(feature = "std"), no_std)]
#![cfg_attr(docsrs, feature(doc_cfg))]

/*!
# Magba
**Magba** is a performant analytical magnetic computation library for Rust.

All physical quantities are assumed to be in SI units.
Python bindings available via [PyMagba](https://github.com/p-sira/pymagba).

## User Guide

### Basic Features

- [**Installing Magba** and controlling feature flags](https://github.com/p-sira/magba?tab=readme-ov-file#installation)
- [Manipulating object **positions and orientations**](base::Transform#examples)
- [Creating **magnets and computing fields**](magnets)
- [**Using sensors** to measure magnetic fields](sensors)
- [Grouping magnets and sensors into **collections**](collections)
- [**Parallelization** using Rayon (enabled by default)](magnets#computing-b-field)

### Advanced Features

- [Calculating fields directly](fields)
- [Using f32](base::Float)
- [Unstable features](fields#internal-functions-unstable)

## Acknowledgment

Most of the field computation used in Magba is based on [MagpyLib](https://github.com/magpylib/magpylib).
We would like to thank MagpyLib contributors for their hard work and contributions to the scientific community.
*/

#[cfg(feature = "alloc")]
extern crate alloc;

pub(crate) mod crate_utils;
use crate::crate_utils::need_std;

pub mod base;
pub mod conversion;
pub mod fields;
pub mod measurement;

pub mod currents;
pub mod magnets;
pub mod sensors;

need_std!(
    pub mod collections;

    #[cfg(test)]
    pub mod testing_util;
);

/// Re-exports of commonly used Magba structs, traits, and methods.
pub mod prelude {
    use super::*;

    pub use base::{Float, Observer, SensorOutput, Source, Transform};
    pub use currents::{CircularCurrent, Current};
    pub use magnets::{CuboidMagnet, CylinderMagnet, Dipole, Magnet, SphereMagnet};
    pub use sensors::{Sensor, hall_effect};

    need_std!(
        pub use collections::{
            SourceComponent, SourceAssembly, ObserverComponent, ObserverAssembly, SourceArray,
            ObserverArray,
        };
    );
}
