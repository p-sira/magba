/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use magba::{prelude::*, sources};
use nalgebra::{Point3, UnitQuaternion};

fn main() {
    // Create a cylindrical magnet with polarization (0, 0, 1) T,
    // radius 0.05 m, and height 0.1 m.
    let cylinder = CylinderMagnet::<f64>::new(
        [0.0, 0.0, 0.0],            // Position [m]
        UnitQuaternion::identity(), // Rotation as unit quaternion
        [0.0, 0.0, 1.0],            // Polarization [T]
        0.05,                       // Radius [m]
        0.1,                        // Height [m]
    );

    // Create a cuboid magnet with default parameters.
    let cuboid = CuboidMagnet::<f64>::default();

    // Group magnets into a source assembly
    let mut assembly = sources!(cylinder, cuboid);

    // Apply transformation on the source assembly
    assembly.translate([0.0, 0.0, 0.1]);

    // Compute the magnetic field at an observation point
    let observer_point = Point3::new(0.0, 0.0, 0.3);
    let b_field = assembly.compute_B(observer_point);

    println!("B-field at {:?}: {:?}", observer_point, b_field);
    // B-field at [0.0, 0.0, 0.3]: [[0.0, 0.0, 0.6316701187086277]]
}
