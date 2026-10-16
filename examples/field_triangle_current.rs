//! Triangular current sheet geometry and 3D field lines.
mod common;

use magba::prelude::*;
use nalgebra::Vector3;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let source = TriangleCurrent::<f64>::default()
        .with_current_density([1000.0, 0.0, 0.0])
        .with_vertices([
            Vector3::new(-0.01, -0.008, 0.0),
            Vector3::new(0.01, -0.008, 0.0),
            Vector3::new(0.0, 0.01, 0.0),
        ]);
    common::field_scene(
        &source,
        0.04,
        "Triangular current sheet: 3D field lines",
        "field_triangle_current.html",
    )
}
