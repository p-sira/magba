//! Square current loop geometry and 3D field lines.
#[path = "../common/mod.rs"]
mod common;

use magba::prelude::*;
use nalgebra::Vector3;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let source = PathCurrent::<f64>::default()
        .with_current(10.0)
        .with_vertices(vec![
            Vector3::new(-0.01, -0.01, 0.0),
            Vector3::new(0.01, -0.01, 0.0),
            Vector3::new(0.01, 0.01, 0.0),
            Vector3::new(-0.01, 0.01, 0.0),
            Vector3::new(-0.01, -0.01, 0.0),
        ]);
    common::field_scene(
        &source,
        0.04,
        "Square current loop: 3D field lines",
        "field_path_current.html",
    )
}
