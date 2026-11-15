//! Generate the cylinder segment magnet Rustdoc field-line preview.

#[path = "../common/mod.rs"]
mod common;

use magba::{
    graphics::{FieldLineOptions, GraphicsOptions, MagnetColorScheme},
    prelude::*,
    sources,
};
use nalgebra::point;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Two broad DIN sectors flank a smaller, oppositely polarized Okabe–Ito sector.
    let first = CylinderSegmentMagnet::<f64>::default()
        .with_dimensions([0.004, 0.019, 0.012, -100.0, 25.0])
        .with_polarization([0.0, 0.0, 1.0]);
    let middle = CylinderSegmentMagnet::<f64>::default()
        .with_dimensions([0.005, 0.015, 0.012, 25.0, 65.0])
        .with_polarization([0.0, 0.0, -1.0]);
    let last = CylinderSegmentMagnet::<f64>::default()
        .with_dimensions([0.004, 0.020, 0.012, 65.0, 190.0])
        .with_polarization([0.0, 0.0, 1.0]);

    let seeds: Vec<_> = (0..8)
        .flat_map(|i| {
            let angle = std::f64::consts::TAU * i as f64 / 8.0;
            [0.025].map(|r| point![r * angle.cos(), r * angle.sin(), 0.0])
        })
        .collect();
    let mut plot = first.plot();
    middle.add_to_plot_with_options(
        &mut plot,
        &GraphicsOptions::default().with_color_scheme(MagnetColorScheme::OkabeIto),
    );
    last.add_to_plot(&mut plot);
    sources!(first, middle, last).add_field_lines(
        &mut plot,
        &seeds,
        FieldLineOptions {
            step_size: 0.0006,
            max_steps: 260,
            bounds: Some([[-0.045, 0.045]; 3]),
            ..Default::default()
        },
    )?;
    common::save_preview(plot, "cylinder_segment_field.html")
}
