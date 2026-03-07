<p align="center">
    <a href="https://github.com/p-sira/magba/">
        <img src="https://github.com/p-sira/magba/blob/main/logo/magba-logo.svg?raw=true" alt="Magba" width="300">
    </a>
</p>
<p align="center">
    <a href="https://opensource.org/license/BSD-3-clause" style="text-decoration:none">
        <img src="https://img.shields.io/badge/License-BSD--3--Clause-brightgreen.svg" alt="License">
    </a>
    <a href="https://crates.io/crates/magba" style="text-decoration:none">
        <img src="https://img.shields.io/crates/v/magba" alt="Crate">
    </a>
    <a href="https://crates.io/crates/magba" style="text-decoration: none">
        <img src="https://img.shields.io/crates/d/magba" alt="Total Downloads">
    </a>
    <a href="https://docs.rs/magba" style="text-decoration:none">
        <img src="https://img.shields.io/badge/Docs-docs.rs-blue" alt="Documentation">
    </a>
</p>

-----

**Magba** is a performant analytical magnetic computation library for Rust.

Python bindings available via [PyMagba](https://github.com/p-sira/pymagba).

```bash
cargo add magba
```

Learn more at [docs.rs/magba](https://docs.rs/magba).

## Features

- Creating magnets and computing fields.
- Manipulating object positions and orientations.
- Using sensors to measure magnetic fields.
- Grouping magnets and sensors into collections.
- Parallelization using [Rayon](https://docs.rs/crate/rayon/latest) (enabled by default).
- Support calculation with `f32` and `f64`.

## Installation

To install Magba using `cargo`, run:
```bash
cargo add magba
```

By default, Magba installs with all stable features enabled, including parallelization with Rayon.

### Feature Flags

The available feature flags are:
- `default`: Enable std and rayon.
- `alloc`: Enable heap allocations, allowing collections and batch processing without the full `std` library.
- `std`: Use std features, such as magnet and sources structs.
  Disable the flag to use Magba in `no_std` environments. Without std,
  you can still access the `fields` module to directly compute the fields.
- `rayon`: Parallelization using [Rayon](https://github.com/rayon-rs/rayon).
- `libm`: Use libm as the math backend. Must be enabled when compiling for `no_std`.
- `unstable`: Enable unstable features. These features may change any time.

## Quick Start

```rust
use magba::prelude::*;
use nalgebra::*;
use std::f64::consts::PI;

// Define magnetic sources
let cylinder = CylinderMagnet::default();
let cuboid = CuboidMagnet::new(
    [1.0, 0.0, 0.0],              // position (m)
    UnitQuaternion::identity(),   // orientation
    [0.0, 0.0, 1.0],              // polarization (T)
    [0.1, 0.2, 0.3],              // dimensions (m)
);

// Grouping sources as collection
let mut source_assembly = sources!(cylinder, cuboid);
source_assembly.push(Dipole::default());

// Observer positions
let points = [
    point![0.0, 0.0, 0.020],
    point![0.0, 0.0, 0.025],
    point![0.0, 0.0, 0.030],
];

// Compute B-field (Magnetic flux density [T])
let b_fields = source_assembly.compute_B_batch(&points);
// [
//     [ -2.7063724257464133e-5,  -3.533949646070574e-17,  0.7312215044902747  ],
//     [ -3.381192498299282e-5 ,   0.0,                    0.7187831955877858  ],
//     [ -4.0548326050340215e-5,   0.0,                    0.7130992758498962  ],
// ]

// Move and Rotate
source_assembly.translate([0.0, 0.0, 0.010]);
source_assembly.rotate(UnitQuaternion::from_scaled_axis([PI / 4.0, 0.0, 0.0].into()));

let b_fields = source_assembly.compute_B_batch(&points);
// [
//     [ -9.575129388363597e-6 ,  -0.24516787434696088,  0.4573303607411665  ],
//     [ -1.4358446356125264e-5,  -0.2948988353221851 ,  0.3578212873125478  ],
//     [ -1.9136669915278972e-5,  -0.30697154302354923,  0.33360985034592394 ],
// ]
```

## Testing

Results are validated against MagpyLib and reference data.
See `/tests/test-data` and the accuracy report for details.

---

## Acknowledgment

Most of the field computation used in Magba is based on [MagpyLib](https://github.com/magpylib/magpylib).
We would like to thank MagpyLib contributors for their hard work and contributions to the scientific community.

---

Learn more at [docs.rs/magba](https://docs.rs/magba).
