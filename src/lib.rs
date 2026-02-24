/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

#![cfg_attr(not(feature = "std"), no_std)]

/*!
# Magba
**Magba** is a performant analytical magnetic computation library for Rust.

All physical quantities are assumed to be in SI units.

## Features

- Compute [magnetic fields](#field-computation) analytically for various source geometries.
- Create [Magnets](Magnet) and group them as [Collection].
- [Move and rotate objects](#move-and-rotate-objects) in 3D space.
- Increase performance using parallelization with [Rayon](https://docs.rs/crate/rayon/latest).
- Support calculation with [f32] and [f64].
- Python bindings available via [Pymagba](https://github.com/p-sira/pymagba).

## Installation

To install Magba using `cargo`, run:
```bash
cargo add magba
```

By default, Magba installs with all stable features enabled, including parallelization with Rayon.

The available feature flags are:
- `default`: Enable std and rayon.
- `std`: Use std features, such as magnet and collection structs.
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

## User Guide

### Basic Features

- Manipulating object positions and orientations
- Creating magnets and computing fields
- Using sensors
- Grouping components as collections

### Advanced Features

- Grouping magnets and sensors using stack-allocated arrays
- Calculating field directly
- Unstable features

## Acknowledgment

Most of the field computation used in Magba is based on [MagpyLib](https://github.com/magpylib/magpylib).
We would like to thank MagpyLib contributors their hard work and contributions to the scientific community.
*/

#![cfg_attr(
    not(feature = "default"),
    allow(
        rustdoc::broken_intra_doc_links,
        rustdoc::private_intra_doc_links,
        unused,
    )
)]

mod crate_util;

pub mod constants;
pub mod conversion;
pub mod fields;
pub mod geometry;

pub use constants::MagneticConstants;

pub mod base;
pub use base::*;
pub use geometry::Pose;

crate::crate_util::need_std!(
    pub mod collections;
    pub mod magnets;
    pub mod sensors;

    pub use magnets::*;
    pub use collections::*;
);

crate::crate_util::need_std!(
    #[cfg(test)]
    pub mod testing_util;
);

/// Check if two vectors are close using relative Euclidean distance
#[doc(hidden)]
#[macro_export]
macro_rules! assert_close_vec {
    ($a:expr, $b:expr, $rtol:expr) => {{
        use nalgebra::{Point3, distance};

        let dist = distance(&Point3::from($a.clone()), &Point3::from($b.clone()));

        let rel_a: f64 = dist / $a.magnitude();
        let rel_b: f64 = dist / $b.magnitude();
        let rel = rel_a.max(rel_b);
        if rel > $rtol {
            panic!(
                "Assertion failed: a={}, b={}, dist = {:e}, rel = {:e}, rtol = {:e}",
                $a, $b, dist, rel, $rtol
            );
        }
    }};
}
