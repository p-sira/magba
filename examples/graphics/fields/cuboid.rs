//! Cuboid magnet geometry and 3D field lines.
#[path = "../common/mod.rs"]
mod common;

use common::exterior::exterior;

use magba::prelude::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let source = CuboidMagnet::<f64>::default().with_dimensions([0.02, 0.014, 0.012]);
    let source = exterior(source);
    common::field_scene(
        &source,
        0.04,
        "Cuboid magnet: 3D field lines",
        "field_cuboid.html",
    )
}
