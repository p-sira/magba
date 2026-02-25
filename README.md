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
    <a href="https://crates.io/crates/magba">
        <img src="https://img.shields.io/crates/d/magba" alt="Total Downloads">
    </a>
    <a href="https://docs.rs/magba" style="text-decoration:none">
        <img src="https://img.shields.io/badge/Docs-docs.rs-blue" alt="Documentation">
    </a>
</p>

-----

**Magba** is a performant analytical magnetic computation library for Rust.

```bash
>> cargo add magba
```

Learn more at [docs.rs/magba](https://docs.rs/magba).

## Features
- Compute [magnetic fields](#field-computation) analytically for various source geometries.
- Create [magnetic sources](Source) and group them as [source collections](#sources-and-collections).
- [Move and rotate objects](#move-and-rotate-objects) in 3D space.
- Increase performance using [parallelization](#installation) with [Rayon](https://docs.rs/crate/rayon/latest).
- Support calculation with [f32] and [f64].
- Python bindings available via [Pymagba](https://github.com/p-sira/pymagba)

## Quick Start

To install, simply: `cargo add magba`.

```rust
use magba::*;
use nalgebra::*;
use std::f64::consts::PI;

// Define magnetic sources
let cylinder = Box::new(CylinderMagnet::default());
let cuboid = Box::new(CuboidMagnet::new(
    point![1.0, 0.0, 0.0],  // position (m)
    UnitQuaternion::identity(),  // orientation
    Vector3::z(),                // polarization (T)
    vector![0.1, 0.2, 0.3], // dimensions (m)
));

// Grouping sources as collection
let mut collection = MultiSourceCollection::from_sources(vec![cylinder, cuboid]);
collection.add(Box::new(Dipole::default()));

// Observer positions
let points = [
    point![0.0, 0.0, 0.020],
    point![0.0, 0.0, 0.025],
    point![0.0, 0.0, 0.030],
];

// Compute B-field (Magnetic flux density [T])
let b_fields = collection.compute_B(&points);
// [
//      [ -2.7063724257464133e-5,  -3.533949646070574e-17,  0.4715809600578704  ],
//      [ -3.381192498299282e-5 ,   0.0,                    0.4592848558923018  ],
//      [ -4.0548326050340215e-5,   0.0,                    0.45377483361434284 ],
// ]

// Move and Rotate
collection.translate(&Translation3::new(0.0, 0.0, 0.010));
collection.rotate(&UnitQuaternion::from_scaled_axis(vector![PI / 4.0, 0.0, 0.0]));

let b_fields = collection.compute_B(&points);
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

Learn more at [docs.rs/magba](https://docs.rs/magba).
