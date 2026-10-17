//! Cylinder magnet geometry and 3D field lines.
mod common;

use common::exterior::Exterior;

use magba::prelude::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let source = CylinderMagnet::<f64>::default()
        .with_diameter(0.02)
        .with_height(0.012);
    let source = Exterior(source);
    common::field_scene(
        &source,
        0.04,
        "Cylinder magnet: 3D field lines",
        "field_cylinder.html",
    )
}
