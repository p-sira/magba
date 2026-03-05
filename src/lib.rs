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
Python bindings available via [Pymagba](https://github.com/p-sira/pymagba).

## User Guide

### Basic Features

- [Manipulating object positions and orientations](base::Transform#examples)
- [Creating magnets and computing fields](magnets)
- [Using sensors to measure magnetic fields](sensors)
- [Grouping magnets and sensors into collections](collections)
- Parallelization using [Rayon](https://github.com/rayon-rs/rayon) (enabled by default)

### Advanced Features

- [Calculating fields directly](fields)

- Using f32

  Magba supports generic `<T: Float>`, defaulting to `f64`. You can explicitly
  use `f32` for better performance at the cost of precision.

- Unstable features

  The `unstable` cargo feature flag enables experimental field calculation
  functions such as `local_cuboid_B` in [fields::unstable].

## Installation

To install Magba using `cargo`, run:
```bash
cargo add magba
```

By default, Magba installs with all stable features enabled, including parallelization with Rayon.

The available feature flags are:
- `default`: Enable std and rayon.
- `alloc`: Enable heap allocations, allowing collections and batch processing without the full `std` library.
- `std`: Use std features, such as magnet and sources structs.
  Disable the flag to use Magba in `no_std` environments. Without std,
  you can still access the [fields] module to directly compute the fields.
- `rayon`: Parallelization using [Rayon](https://github.com/rayon-rs/rayon).
- `libm`: Use libm as the math backend. Must be enabled when compiling for `no_std`.
- `unstable`: Enable unstable features. These features may change any time.

### No-std

To install for `no_std` environments, you must also enable `libm`, using:

```bash
cargo add magba --no-default-features --features libm
```

## Acknowledgment

Most of the field computation used in Magba is based on [MagpyLib](https://github.com/magpylib/magpylib).
We would like to thank MagpyLib contributors their hard work and contributions to the scientific community.
*/

#[cfg(feature = "alloc")]
extern crate alloc;

mod crate_util;
use crate::crate_util::need_std;

pub mod base;
pub mod conversion;
pub mod fields;
pub mod measurement;

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
    pub use magnets::{CuboidMagnet, CylinderMagnet, Dipole, Magnet};
    pub use sensors::{Sensor, hall_effect};

    need_std!(
        pub use collections::{
            SourceComponent, SourceAssembly, SensorComponent, SensorAssembly, SourceArray,
            SensorArray,
        };
    );
}
