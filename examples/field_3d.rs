//! A 3D source scene with true field lines and sampled field arrows.
mod common;

use magba::{
    graphics::{FieldGrid, FieldLineOptions, Plane, QuiverOptions},
    prelude::*,
};
use nalgebra::Point3;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let magnet = SphereMagnet::<f64>::default().with_diameter(0.02);
    let seeds: Vec<_> = (0..8)
        .flat_map(|i| {
            let angle = std::f64::consts::TAU * i as f64 / 8.0;
            [0.014, 0.025, 0.034].map(|r| Point3::new(r * angle.cos(), r * angle.sin(), 0.0))
        })
        .collect();
    let mut plot = magnet.plot();
    magnet.add_field_lines(
        &mut plot,
        &seeds,
        FieldLineOptions {
            step_size: 0.0004,
            max_steps: 1000,
            bounds: Some([[-0.04, 0.04]; 3]),
            ..Default::default()
        },
    )?;
    let grid = FieldGrid::new(Plane::XY, 0.025, [-0.025, 0.025], [-0.025, 0.025], [6, 6])?;
    magnet.add_quiver(
        &mut plot,
        &grid.points(),
        QuiverOptions {
            scale: 0.004,
            normalize: true,
        },
    )?;
    common::save(
        plot,
        "Sphere magnet: 3D field lines and B-field arrows",
        "field_3d.html",
    )
}
