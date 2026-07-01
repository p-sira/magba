/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use magba::{prelude::*, sources};

fn main() {
    // Create magnets and group into a source assembly
    let cylinder = CylinderMagnet::default()
        .with_diameter(0.05)
        .with_height(0.1);
    let cuboid = CuboidMagnet::<f64>::default();
    let mut assembly = sources!(cylinder, cuboid);

    // Apply transformation on the source assembly
    assembly.translate([0.0, 0.0, 0.1]);

    // Compute the magnetic field at an observation point
    let b_field = assembly.compute_B(nalgebra::Point3::new(0.0, 0.0, 0.3));

    println!("B = [{}, {}, {}]", b_field[0], b_field[1], b_field[2]);
    // B = [0, 0, 0.6316701187086277]
}
