//! Triangle magnet geometry and 3D field lines.
mod common;

use magba::{graphics::FieldLineOptions, prelude::*};
use nalgebra::{Point3, Vector3};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let source = TriangleMagnet::<f64>::default().with_vertices([
        Vector3::new(-0.01, -0.008, 0.0),
        Vector3::new(0.01, -0.008, 0.0),
        Vector3::new(0.0, 0.01, 0.0),
    ]);
    // Coplanar seeds remain in z = 0 and collapse the 3D scene. Seed both
    // sides of the charged surface, away from its discontinuity and edges.
    let vertices = source.vertices();
    let mut seeds = Vec::new();
    for i in 1..5 {
        for j in 1..(6 - i) {
            let point = (vertices[0] * i as f64
                + vertices[1] * j as f64
                + vertices[2] * (6 - i - j) as f64)
                / 6.0;
            for z in [-0.001, 0.001] {
                seeds.push(Point3::new(point.x, point.y, z));
            }
        }
    }
    let mut plot = source.plot();
    source.add_field_lines(
        &mut plot,
        &seeds,
        FieldLineOptions {
            step_size: 0.0004,
            max_steps: 400,
            bounds: Some([[-0.02, 0.02]; 3]),
            ..Default::default()
        },
    )?;
    common::save(
        plot,
        "Triangle magnet: 3D field lines",
        "field_triangle.html",
    )
}
