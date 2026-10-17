//! Combined field of a heterogeneous source collection.
mod common;

use common::exterior::Exterior;

use magba::{prelude::*, sources};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let sources = sources!(
        CuboidMagnet::<f64>::default()
            .with_dimensions([0.012, 0.012, 0.02])
            .with_position([-0.015, 0.0, 0.0]),
        CylinderMagnet::default()
            .with_diameter(0.012)
            .with_height(0.02)
            .with_polarization([0.0, 0.0, -1.0])
            .with_position([0.015, 0.0, 0.0]),
        CircularCurrent::default()
            .with_diameter(0.06)
            .with_current(10.0),
    );
    let sources = Exterior(sources);
    common::field_scene(
        &sources,
        0.08,
        "Source collection: combined 3D field lines",
        "field_collection.html",
    )
}
