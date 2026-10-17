//! Triangular current sheet geometry and 3D field lines.
mod common;

use magba::{graphics::FieldLineOptions, prelude::*};
use nalgebra::{Point3, Vector3};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let source = TriangleCurrent::<f64>::default()
        .with_current_density([1000.0, 0.0, 0.0])
        .with_vertices([
            Vector3::new(-0.01, -0.008, 0.0),
            Vector3::new(0.01, -0.008, 0.0),
            Vector3::new(0.0, 0.01, 0.0),
        ]);
    // Current flows along x, so B has no x component. Seed two loops in each
    // of three yz planes, outside the sheet, without mirrored duplicates.
    let seeds: Vec<_> = [-0.005, 0.0, 0.005]
        .into_iter()
        .flat_map(|x| [-0.012, -0.022].map(|y| Point3::new(x, y, 0.0)))
        .collect();
    let mut plot = source.plot();
    source.add_field_lines(
        &mut plot,
        &seeds,
        FieldLineOptions {
            step_size: 0.0003,
            max_steps: 700,
            bounds: Some([[-0.03, 0.03]; 3]),
            ..Default::default()
        },
    )?;
    common::save(
        plot,
        "Triangular current sheet: 3D field lines",
        "field_triangle_current.html",
    )
}
