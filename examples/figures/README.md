# Magnetic field examples

These PNG figures are exported from the runnable Plotly examples. The HTML output
supports zoom, pan, hover, and PNG export; 3D scenes also support rotation.
Plotly.js is embedded for offline viewing. All positions are in meters and B is
in tesla. Magnet surfaces are opaque. The examples exclude magnet interiors
before evaluating fields: heatmaps have gaps, arrows are omitted, and field lines
terminate at the surfaces.

Run from the repository root:

```sh
cargo run --example field_2d --features graphics-plotly
cargo run --example field_quiver --features graphics-plotly
cargo run --example field_3d --features graphics-plotly
cargo run --example field_cuboid --features graphics-plotly
cargo run --example field_cylinder --features graphics-plotly
cargo run --example field_tetrahedron --features graphics-plotly
cargo run --example field_triangle --features graphics-plotly
cargo run --example field_dipole --features graphics-plotly
cargo run --example field_path_current --features graphics-plotly
cargo run --example field_triangle_current --features graphics-plotly
cargo run --example field_collection --features graphics-plotly
```

Each command writes a matching HTML file in the current directory. Pass an output
path as the first argument, for example:

```sh
cargo run --example field_2d --features graphics-plotly -- /tmp/sphere.html
```

Use the plot toolbar's camera button to export a PNG. The figures below were
exported at 1100 × 800 with a scale factor of 1.5; the showcase uses 1400 × 1100
with a scale factor of 1.25. Titles use 32 px, and legends and subplot titles use
24 px.

## 2D magnitude and streamlines

[Source](../graphics/fields/2d.rs). A sphere magnet with a 20 mm diameter and 1 T polarization
along z, sampled in the xz plane. Orange lines follow the projected B field and
the white circle marks the magnet surface.

![Sphere field magnitude and streamlines](field_2d.png)

## 2D quiver

[Source](../graphics/fields/quiver.rs). A 30 mm current loop carrying 10 A. Equal-length arrows
show the in-plane field direction; orange dots mark the wire cross-section.
Set `QuiverOptions::normalize` to false to make length proportional to field
magnitude, using `scale` in meters per tesla.

![Current-loop field arrows](field_quiver.png)

## 3D field lines and quiver

[Source](../graphics/fields/3d.rs). The same sphere magnet, with trajectories integrated in
both directions from two rings of equatorial seeds (14 and 35 mm radii),
with four fan directions near each pole and a central axial line.
Normalized B arrows are sampled above it.

![Sphere field in 3D](field_3d.png)

Field lines use adaptive arc-length RK4 integration with step doubling.
`step_size` sets the maximum step and `tolerance` controls the local error.
Decrease either to assess convergence, especially near material boundaries or thin wires. Tracing stops at
bounds, zero or non-finite fields, unresolved turns, closed loops, or the step limit. A 2D streamline
follows the projected field and matches a 3D field line only where the normal
field component vanishes.

## Additional primitive and collection scenes

The cuboid, cylinder, tetrahedron, dipole, and path-current examples each trace
field lines from three rings of seeds.
The collection example combines two oppositely polarized magnets and a current
loop, tracing their total field. All share the output helper in `examples/graphics/common`.

Magnet surfaces blend from green south poles through a grey central belt to red
north poles along the local polarization direction. The gradient continues across
face boundaries and indicates local polarity.
The `graphics` example also shows the optional orange/blue (`OkabeIto`) and
red/blue (`Trad`) palettes. Run it with `cargo run --example graphics --features graphics-plotly`.

[Cuboid magnet source](../graphics/fields/cuboid.rs)

![Cuboid magnet with 3D field lines](field_cuboid.png)

[Cylinder magnet source](../graphics/fields/cylinder.rs)

![Cylinder magnet with 3D field lines](field_cylinder.png)

[Tetrahedron magnet source](../graphics/fields/tetrahedron.rs)

![Tetrahedron magnet with 3D field lines](field_tetrahedron.png)

[Triangle magnet source](../graphics/fields/triangle.rs)

The triangle example seeds field lines on both sides of the charged surface.
Coplanar seeds would keep the trajectories in the triangle's plane.

![Triangle magnet with 3D field lines](field_triangle.png)

[Point dipole source](../graphics/fields/dipole.rs)

![Point dipole with 3D field lines](field_dipole.png)

[Square current loop source](../graphics/fields/path_current.rs)

![Square current loop with 3D field lines](field_path_current.png)

[Triangular current sheet source](../graphics/fields/triangle_current.rs)

The sheet carries a uniform surface current of 1000 A/m along x. Six seeds
outside the sheet trace two loops in each of three planes of constant x.

![Triangular current sheet with 3D field lines](field_triangle_current.png)

[Mixed source collection source](../graphics/fields/collection.rs)

![Mixed source collection with 3D field lines](field_collection.png)

## 2×2 graphics showcase

[Source](../graphics/showcase.rs). Geometry, collections, heatmaps,
normalized 2D/3D quivers, and field lines with direction arrowheads in one figure.
The sphere panel uses the same equatorial rings, polar fans, and axial line.

```sh
cargo run --example graphics_showcase --features graphics-plotly
```

![Magba graphics showcase](graphics_showcase.png)

## Rustdoc previews

The `rustdoc_*.png` figures show the corresponding code snippets in
[`graphics::plotly`](../../src/graphics/plotly.rs) and the
[`GraphicsView`](../../src/graphics/options.rs) documentation:

- `rustdoc_scene.png`: cuboid magnet and circular current, shared by the
  [graphics module example](../../src/graphics/mod.rs) and the Plotly module example.
- `rustdoc_appearance.png`: cylinder with the default DIN palette, opacity 0.5, and 96 segments; also illustrates `GraphicsOptions`.
- `rustdoc_view.png`: f32 cylinder view with opacity 0.5 and 64 segments.
- `rustdoc_field_lines.png`: sphere magnitude heatmap and projected field line.
- `rustdoc_field_quiver.png`: reused sphere samples with magnitude and in-plane arrows.

Render the documented plots through Plotly's PNG export at 880 × 600 pixels.
Use Arial at 16 px, margins of 65/35/45/65 px (left/right/top/bottom), and a
3D camera eye of (1.9, 1.9, 1.7) to keep geometry and labels within the image.
Rustdoc uses absolute GitHub `blob/main` URLs with `?raw=true`, following ellip's
figure convention, so image resolution is independent of the generated page path.

## Magnet type examples

Each magnet type's Graphics section shows a generated `magnet_*.png` preview
using the default DIN palette. The separate runnable examples trace the full
field; opaque surfaces hide interior line segments.

- [Cuboid](../graphics/magnets/cuboid.rs): `magnet_cuboid.png`.
- [Cylinder](../graphics/magnets/cylinder.rs): `magnet_cylinder.png`.
- [Sphere](../graphics/magnets/sphere.rs): `magnet_sphere.png`.
- [Tetrahedron](../graphics/magnets/tetrahedron.rs): `magnet_tetrahedron.png`.
- [Triangle](../graphics/magnets/triangle.rs): `magnet_triangle.png`.
- [Dipole](../graphics/magnets/dipole.rs): `magnet_dipole.png` (point marker).
- [Mesh](../graphics/magnets/mesh.rs): `magnet_mesh.png` (validated octahedron).

Export these figures using the same 880 × 600 layout and camera as the Rustdoc
previews above. Run
`cargo run --example magnet_cuboid --features graphics-plotly` to generate a
scene's HTML; substitute another example name as needed. The mesh example
also requires `mesh`. The source files define each scene's seed positions,
step size, and integration bounds.
