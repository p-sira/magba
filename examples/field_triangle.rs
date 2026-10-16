//! Triangle magnet geometry and 3D field lines.
mod common;

use magba::prelude::*;
use nalgebra::Vector3;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let source = TriangleMagnet::<f64>::default().with_vertices([
        Vector3::new(-0.01, -0.008, 0.0),
        Vector3::new(0.01, -0.008, 0.0),
        Vector3::new(0.0, 0.01, 0.0),
    ]);
    common::field_scene(
        &source,
        0.04,
        "Triangle magnet: 3D field lines",
        "field_triangle.html",
    )
}
