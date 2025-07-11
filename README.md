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

# Quick Start

To install, simply: `cargo add magba`.

Compute the B-field of a cylindrical magnet at a point:

```rust
use magba::{CylinderMagnet, Field};
use nalgebra::{Point3, UnitQuaternion, Vector3};

let magnet = CylinderMagnet::new(
    Point3::origin(), // position
    UnitQuaternion::identity(), // orientation
    Vector3::new(0.0, 0.0, 1.0), // polarization
    0.005, // radius
    0.020, // height
);

let b = magnet.get_B(&[Point3::new(0.01, 0.0, 0.0)]);
println!("B-field: {:?}", b[0]);
```

## Features

- Analytical magnetic field computation for:
  - Cylindrical magnets (axial and diametric)
  - Source collections (grouping magnets)
- Transformations: Move and rotate sources in 3D space
- Parallel computation (optional, via Rayon)
- Tested against MagpyLib and reference data
- Python bindings available via [Pymagba](https://github.com/p-sira/pymagba)

## Testing

Results are validated against MagpyLib and reference data.
See `/tests/test-data` and the accuracy report for details.

## Documentation

- [API Docs (docs.rs)](https://docs.rs/magba)
- [GitHub Repository](https://github.com/p-sira/magba)

---

Learn more at [docs.rs/magba](https://docs.rs/magba).
