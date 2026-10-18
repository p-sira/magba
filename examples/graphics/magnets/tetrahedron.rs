//! Generate the tetrahedron magnet Rustdoc field-line preview.

use magba::{graphics::FieldLineOptions, prelude::*};
use nalgebra::{point, vector};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let magnet = TetrahedronMagnet::<f64>::default().with_vertices([
        vector![-0.01, -0.008, -0.006],
        vector![0.01, -0.008, -0.006],
        vector![0.0, 0.01, -0.006],
        vector![0.0, 0.0, 0.01],
    ]);
    let seeds: Vec<_> = (0..4)
        .flat_map(|i| {
            let angle = std::f64::consts::TAU * i as f64 / 4.0;
            [0.014, 0.025].map(|r| point![r * angle.cos(), r * angle.sin(), 0.0])
        })
        .collect();
    let mut plot = magnet.plot();
    magnet.add_field_lines(
        &mut plot,
        &seeds,
        FieldLineOptions {
            step_size: 0.0004,
            max_steps: 150,
            bounds: Some([[-0.04, 0.04]; 3]),
            ..Default::default()
        },
    )?;
    std::fs::write("tetrahedron_field.html", plot.to_html())?;
    Ok(())
}
