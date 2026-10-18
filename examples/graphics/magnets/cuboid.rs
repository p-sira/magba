//! Generate the cuboid magnet Rustdoc field-line preview.

use magba::{graphics::FieldLineOptions, prelude::*};
use nalgebra::point;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let magnet = CuboidMagnet::<f64>::default().with_dimensions([0.02, 0.02, 0.02]);
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
    std::fs::write("cuboid_field.html", plot.to_html())?;
    Ok(())
}
