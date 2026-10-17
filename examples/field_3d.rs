//! A 3D source scene with true field lines and sampled field arrows.
mod common;

use common::exterior::Exterior;

use magba::{
    graphics::{FieldGrid, FieldLineOptions, Plane, QuiverOptions},
    prelude::*,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let magnet = SphereMagnet::<f64>::default().with_diameter(0.02);
    let magnet = Exterior(magnet);
    let seeds = common::sphere_field_seeds(8);
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
