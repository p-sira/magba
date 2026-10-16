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
    <a href="https://codecov.io/gh/p-sira/magba">
        <img src="https://codecov.io/gh/p-sira/magba/graph/badge.svg?token=S0CJ922MG0"/> 
    </a>
    <a href="https://app.codspeed.io/p-sira/magba?utm_source=badge" style="text-decoration:none">
        <img src="https://img.shields.io/endpoint?url=https://codspeed.io/badge.json" alt="CodSpeed"/>
    </a>
    <a href="https://doi.org/10.5281/zenodo.21150887" style="text-decoration:none">
        <img src="https://zenodo.org/badge/DOI/10.5281/zenodo.21150887.svg" alt="Preprint DOI">
    </a>
</p>

-----

**Magba** is a performant analytical magnetic computation library for Rust.

Python bindings available via [PyMagba](https://github.com/p-sira/pymagba).

```bash
cargo add magba
```

See the user guide at [docs.rs/magba](https://docs.rs/magba).

## Features

- Creating magnets and computing fields.
- Manipulating object positions and orientations.
- Using sensors to measure magnetic fields.
- Grouping magnets and sensors into collections.
- Parallelization using [Rayon](https://docs.rs/crate/rayon/latest) (enabled by default).
- Adaptive parallel scheduling for source collections, based on their relative computational complexity.
- Support calculation with `f32` and `f64`.

## Installation

To install Magba using `cargo`, run:
```bash
cargo add magba
```

By default, Magba enables `std`, `rayon`, and `mesh`. Graphics and file I/O are opt-in.

### Feature Flags

The available feature flags are:
- `default`: Enable std, rayon, and mesh.
- `alloc`: Enable heap allocations, allowing collections and batch processing without the full `std` library.
- `std`: Use std features, such as magnet and sources structs.
  Disable the flag to use Magba in `no_std` environments. Without std,
  you can still access the `fields` module to directly compute the fields.
- `rayon`: Parallelization using [Rayon](https://github.com/rayon-rs/rayon).
- `mesh`: Enable mesh features, such as `MeshMagnet` and `SheetCurrent`.
- `io-stl`: Enable STL mesh loading features.
- `graphics`: Enable interactive 3D visualization with Plotly (requires `std`).
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

## Interactive 3D Graphics

Enable visualization with `cargo add magba --features graphics`:

```rust,no_run
use magba::prelude::*;

let magnet = CylinderMagnet::<f64>::default();
let mut plot = magnet.plot();
CuboidMagnet::<f64>::default()
    .with_position([2.0, 0.0, 0.0])
    .add_to_plot(&mut plot);
std::fs::write("scene.html", plot.to_html()).expect("Cannot write scene.html");
```

Open the HTML file in a browser to rotate, pan, and zoom. Plotly.js is
embedded, so viewing works offline. Built-in magnets, currents, sensors,
and collections support `Graphics`; mesh objects require `mesh` as well.
Dipoles, sensors, and custom collection components use point markers.
Plots capture the objects' current geometry and poses in meters.
Use the re-exported `magba::graphics::plotly` API to customize a plot.

Magnet bodies are grey with red north and green south poles by default, using
[Magpylib's DIN SPEC 91411 palette](https://magpylib.readthedocs.io/en/stable/_pages/user_guide/docs/docs_styles.html).
Colors follow local polarization and rotate with the magnet. Neutral surfaces
remain grey; curved surfaces have a grey equatorial belt.
Choose orange/blue (`okabe-ito`) or red/blue (`trad`) with
`MagnetColorScheme::OkabeIto` or `MagnetColorScheme::Trad`:

```rust,no_run
use magba::{graphics::MagnetColorScheme, prelude::*};
let magnets = SourceArray::from([CuboidMagnet::<f64>::default()]);
let plot = magnets.plot_with_color_scheme(MagnetColorScheme::OkabeIto);
// To add to an existing plot, use add_to_plot_with_color_scheme(&mut plot, scheme).
```

Run `cargo run --example graphics --features graphics` for a combined scene.

Field plotting is available on every `Source`, including collections:

```rust,no_run
use magba::prelude::*;
use magba::graphics::{FieldComponent, FieldGrid, Plane, QuiverOptions};

let magnet = SphereMagnet::<f64>::default().with_diameter(0.02);
let grid = FieldGrid::new(Plane::XZ, 0.0, [-0.04, 0.04], [-0.04, 0.04], [21, 21])
    .expect("Valid grid");
let slice = magnet.sample_field(&grid).expect("Valid sample positions");
let mut plot = slice.heatmap(FieldComponent::Magnitude);
slice.add_quiver(&mut plot, QuiverOptions { scale: 0.002, normalize: true })
    .expect("Valid arrow scale");
std::fs::write("field.html", plot.to_html()).expect("Cannot write field.html");
```

Use `plot_field_2d` and `plot_quiver_2d` for direct plotting, or reuse a
`FieldSlice` for several views without resampling. `plot_quiver` and
`add_quiver` draw 3D arrows. `plot_field_lines` and `add_field_lines` trace 3D
field lines; their `_2d` variants draw projected streamlines within a grid.
`trace_field_lines` returns the trajectory coordinates for further analysis.
All helpers support `f32` and `f64` sources; plotting coordinates use `f64` meters.

See [runnable examples and rendered figures](examples/figures/README.md):

| Example | Plot | Run with `--features graphics` |
| --- | --- | --- |
| [2D field](examples/field_2d.rs) | Magnitude heatmap and streamlines | `cargo run --example field_2d --features graphics` |
| [Quiver](examples/field_quiver.rs) | Current-loop field arrows | `cargo run --example field_quiver --features graphics` |
| [3D field](examples/field_3d.rs) | Source, field lines, and arrows | `cargo run --example field_3d --features graphics` |
| [Cuboid](examples/field_cuboid.rs) | Geometry and 3D field lines | `cargo run --example field_cuboid --features graphics` |
| [Cylinder](examples/field_cylinder.rs) | Geometry and 3D field lines | `cargo run --example field_cylinder --features graphics` |
| [Tetrahedron](examples/field_tetrahedron.rs) | Geometry and 3D field lines | `cargo run --example field_tetrahedron --features graphics` |
| [Triangle](examples/field_triangle.rs) | Geometry and 3D field lines | `cargo run --example field_triangle --features graphics` |
| [Dipole](examples/field_dipole.rs) | Geometry and 3D field lines | `cargo run --example field_dipole --features graphics` |
| [Path current](examples/field_path_current.rs) | Geometry and 3D field lines | `cargo run --example field_path_current --features graphics` |
| [Triangle current](examples/field_triangle_current.rs) | Geometry and 3D field lines | `cargo run --example field_triangle_current --features graphics` |
| [Source collection](examples/field_collection.rs) | Geometry and 3D field lines | `cargo run --example field_collection --features graphics` |

## Testing

Results are validated against MagpyLib and reference data.
See `testing/data` and the accuracy report for details.

---

## Acknowledgment

Most of the field computation used in Magba is based on [MagpyLib](https://github.com/magpylib/magpylib).
We would like to thank MagpyLib contributors for their hard work and contributions to the scientific community.

---

Learn more at [docs.rs/magba](https://docs.rs/magba).
