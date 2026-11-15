//! Analytical cylinder and current-loop invariants complementing the Wolfram fixtures.

use magba::{
    base::Float,
    fields::{circular_B, circular_B_batch, cylinder_B, cylinder_B_batch},
};
use nalgebra::{Point3, UnitQuaternion, Vector3};

const DIAMETER: f64 = 0.02;
const HEIGHT: f64 = 0.02;
const CURRENT: f64 = 2.5;

fn axial_cylinder(z: f64, polarization: f64) -> f64 {
    let r = DIAMETER / 2.0;
    let h = HEIGHT / 2.0;
    polarization / 2.0
        * ((z + h) / (r * r + (z + h).powi(2)).sqrt() - (z - h) / (r * r + (z - h).powi(2)).sqrt())
}

fn axial_loop(z: f64, current: f64) -> f64 {
    let r = DIAMETER / 2.0;
    <f64 as Float>::mu0() * current * r * r / (2.0 * (r * r + z * z).powf(1.5))
}

fn close(actual: Vector3<f64>, expected_z: f64) {
    assert!(actual.x.abs() < 1e-12 && actual.y.abs() < 1e-12);
    assert!(
        (actual.z - expected_z).abs() < 2e-12 + 2e-11 * expected_z.abs(),
        "actual={actual:?}, expected Bz={expected_z}"
    );
}

#[test]
fn axial_fields_match_independent_closed_forms() {
    for z in [-0.03, -0.007, 0.0, 0.007, 0.03] {
        let point = Point3::new(0.0, 0.0, z);
        close(
            cylinder_B(
                point,
                Point3::origin(),
                UnitQuaternion::identity(),
                Vector3::new(0.0, 0.0, 1.2),
                DIAMETER,
                HEIGHT,
            ),
            axial_cylinder(z, 1.2),
        );
        close(
            circular_B(
                point,
                Point3::origin(),
                UnitQuaternion::identity(),
                DIAMETER,
                CURRENT,
            ),
            axial_loop(z, CURRENT),
        );
    }
}

#[test]
fn zero_sources_remain_zero_in_rotated_batches() {
    let points = [
        Point3::origin(),
        Point3::new(0.001, 0.002, 0.003),
        Point3::new(0.025, -0.011, 0.017),
    ];
    let position = Point3::new(0.003, -0.004, 0.002);
    let rotation = UnitQuaternion::from_scaled_axis(Vector3::new(0.24, -0.15, 0.37));
    let mut out = [Vector3::repeat(f64::NAN); 3];
    cylinder_B_batch(
        &points,
        position,
        rotation,
        Vector3::zeros(),
        DIAMETER,
        HEIGHT,
        &mut out,
    );
    for (&point, field) in points.iter().zip(&out) {
        assert_eq!(*field, Vector3::zeros());
        assert_eq!(
            cylinder_B(
                point,
                position,
                rotation,
                Vector3::zeros(),
                DIAMETER,
                HEIGHT
            ),
            *field
        );
    }
    circular_B_batch(&points, position, rotation, DIAMETER, 0.0, &mut out);
    for (&point, field) in points.iter().zip(&out) {
        assert_eq!(*field, Vector3::zeros());
        assert_eq!(circular_B(point, position, rotation, DIAMETER, 0.0), *field);
    }
}

#[cfg(feature = "rayon")]
#[test]
fn parallel_batches_match_axis_closed_forms() {
    const N: usize = 512;
    let z = [-0.03, -0.007, 0.0, 0.007, 0.03];
    let points: Vec<_> = (0..N)
        .map(|i| Point3::new(0.0, 0.0, z[i % z.len()]))
        .collect();
    let mut out = vec![Vector3::zeros(); N];
    cylinder_B_batch(
        &points,
        Point3::origin(),
        UnitQuaternion::identity(),
        Vector3::new(0.0, 0.0, 1.2),
        DIAMETER,
        HEIGHT,
        &mut out,
    );
    for (point, field) in points.iter().zip(&out) {
        close(*field, axial_cylinder(point.z, 1.2));
    }
    circular_B_batch(
        &points,
        Point3::origin(),
        UnitQuaternion::identity(),
        DIAMETER,
        CURRENT,
        &mut out,
    );
    for (point, field) in points.iter().zip(&out) {
        close(*field, axial_loop(point.z, CURRENT));
    }
}
