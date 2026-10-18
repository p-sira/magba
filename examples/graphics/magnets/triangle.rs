//! Generate the triangle magnet Rustdoc field-line preview.

use magba::{graphics::FieldLineOptions, prelude::*};
use nalgebra::{point, vector};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let magnet = TriangleMagnet::<f64>::default().with_vertices([
        vector![-0.01, -0.008, 0.0],
        vector![0.01, -0.008, 0.0],
        vector![0.0, 0.01, 0.0],
    ]);
    // Seed both sides of the charged surface, away from its discontinuity.
    let seeds: Vec<_> = [-0.004, 0.0, 0.004]
        .into_iter()
        .flat_map(|x| {
            [-0.003, 0.003]
                .into_iter()
                .flat_map(move |y| [-0.001, 0.001].map(|z| point![x, y, z]))
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
    std::fs::write("triangle_field.html", plot.to_html())?;
    Ok(())
}
