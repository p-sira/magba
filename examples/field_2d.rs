//! Sphere cross-section: sampled magnitude, integrated streamlines, and source outline.
mod common;

use common::exterior::Exterior;

use magba::graphics::plotly::{
    Scatter,
    common::{Line, Mode},
};
use magba::{
    graphics::{FieldComponent, FieldGrid, FieldLineOptions, Plane},
    prelude::*,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let magnet = SphereMagnet::<f64>::default().with_diameter(0.02);
    let magnet = Exterior(magnet);
    let grid = FieldGrid::new(Plane::XZ, 0.0, [-0.04, 0.04], [-0.04, 0.04], [181, 181])?;
    let mut plot = magnet.plot_field_2d(&grid, FieldComponent::Magnitude)?;
    let seeds: Vec<_> = [-1.0, 1.0]
        .into_iter()
        .flat_map(|sign| [0.013, 0.018, 0.025, 0.034].map(|x| [sign * x, 0.0]))
        .collect();
    magnet.add_field_lines_2d(
        &mut plot,
        &grid,
        &seeds,
        FieldLineOptions {
            step_size: 0.0003,
            max_steps: 1200,
            ..Default::default()
        },
    )?;
    let angles: Vec<_> = (0..=100)
        .map(|i| std::f64::consts::TAU * i as f64 / 100.0)
        .collect();
    plot.add_trace(
        Scatter::new(
            angles.iter().map(|t| 0.01 * t.cos()).collect(),
            angles.iter().map(|t| 0.01 * t.sin()).collect(),
        )
        .mode(Mode::Lines)
        .name("Magnet surface")
        .line(Line::new().color("white").width(2.0)),
    );
    common::save(
        plot,
        "Sphere magnet: field magnitude and streamlines (y = 0)",
        "field_2d.html",
    )
}
