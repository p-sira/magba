//! Tetrahedron magnet geometry and 3D field lines.
mod common;

use common::exterior::Exterior;

use magba::prelude::*;
use nalgebra::Vector3;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let source = TetrahedronMagnet::<f64>::default().with_vertices([
        Vector3::new(-0.01, -0.008, -0.006),
        Vector3::new(0.01, -0.008, -0.006),
        Vector3::new(0.0, 0.01, -0.006),
        Vector3::new(0.0, 0.0, 0.01),
    ]);
    let source = Exterior(source);
    common::field_scene(
        &source,
        0.04,
        "Tetrahedron magnet: 3D field lines",
        "field_tetrahedron.html",
    )
}
