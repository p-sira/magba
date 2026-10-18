//! Current-loop cross-section: normalized arrows show direction, independently of magnitude.
#[path = "../common/mod.rs"]
mod common;

use magba::graphics::plotly::{
    Scatter,
    common::{Marker, Mode},
};
use magba::{
    graphics::{FieldGrid, Plane, QuiverOptions},
    prelude::*,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let current = CircularCurrent::<f64>::default()
        .with_diameter(0.03)
        .with_current(10.0);
    let grid = FieldGrid::new(Plane::XZ, 0.0, [-0.04, 0.04], [-0.04, 0.04], [21, 21])?;
    let mut plot = current.plot_quiver_2d(
        &grid,
        QuiverOptions {
            scale: 0.0027,
            normalize: true,
        },
    )?;
    plot.add_trace(
        Scatter::new(vec![-0.015, 0.015], vec![0.0, 0.0])
            .mode(Mode::Markers)
            .name("Wire cross-section")
            .marker(Marker::new().size(12).color("#ea580c")),
    );
    common::save(
        plot,
        "Current loop: normalized B-field arrows (y = 0)",
        "field_quiver.html",
    )
}
