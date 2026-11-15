//! Point dipole geometry and 3D field lines.
#[path = "../common/mod.rs"]
mod common;

use magba::prelude::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let source = Dipole::<f64>::default().with_moment([0.0, 0.0, 1.0]);
    common::field_scene(
        &source,
        0.04,
        "Point dipole: 3D field lines",
        "field_dipole.html",
    )
}
