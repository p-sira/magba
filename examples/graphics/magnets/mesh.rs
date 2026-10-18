//! Generate the mesh magnet Rustdoc field-line preview.

use magba::{graphics::FieldLineOptions, prelude::*};
use nalgebra::{point, vector};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let vertices = vec![
        vector![0.01, 0.0, 0.0],
        vector![-0.01, 0.0, 0.0],
        vector![0.0, 0.01, 0.0],
        vector![0.0, -0.01, 0.0],
        vector![0.0, 0.0, 0.012],
        vector![0.0, 0.0, -0.012],
    ];
    let faces = vec![
        [4, 0, 2],
        [4, 2, 1],
        [4, 1, 3],
        [4, 3, 0],
        [5, 2, 0],
        [5, 1, 2],
        [5, 3, 1],
        [5, 0, 3],
    ];
    let magnet = MeshMagnet::from_vertices_and_faces(vertices, faces, [0.0, 0.0, 1.0])?;
    let seeds: Vec<_> = (0..8)
        .flat_map(|i| {
            let angle = std::f64::consts::TAU * i as f64 / 8.0;
            [0.014, 0.025].map(|r| point![r * angle.cos(), r * angle.sin(), 0.0])
        })
        .collect();
    let mut plot = magnet.plot();
    magnet.add_field_lines(
        &mut plot,
        &seeds,
        FieldLineOptions {
            step_size: 0.0004,
            max_steps: 600,
            bounds: Some([[-0.04, 0.04]; 3]),
            ..Default::default()
        },
    )?;
    std::fs::write("mesh_field.html", plot.to_html())?;
    Ok(())
}
