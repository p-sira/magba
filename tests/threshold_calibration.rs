#![cfg(feature = "threshold-calibration")]

use std::sync::Mutex;

use magba::{
    base::Source,
    collections::SourceArray,
    magnets::Dipole,
    threshold_calibration::{self::*, ExecutionMode::*},
};
use nalgebra::{Point3, UnitQuaternion, Vector3};

// Calibration controls are process-wide. Keep these tests in their own binary
// so unrelated field computations cannot change their branch counts.
static CALIBRATION: Mutex<()> = Mutex::new(());

#[test]
fn modes_override_automatic_choice() {
    let _lock = CALIBRATION.lock().unwrap();
    let source = Dipole::<f64>::default();
    let points = vec![Point3::new(1.0, 2.0, 3.0); 6251];
    set_instrumentation(true);
    reset_branch_counts();

    set_execution_mode(Serial);
    let expected = source.compute_B_batch(&points);
    set_execution_mode(Parallel);
    assert_eq!(source.compute_B_batch(&points[..1]), expected[..1]);
    set_execution_mode(Auto);
    assert_eq!(source.compute_B_batch(&points), expected);
    assert_eq!(source.compute_B_batch(&points[..1]), expected[..1]);

    assert_eq!(branch_counts(), (2, 2));
    set_instrumentation(false);
}

#[test]
fn collections_require_outer_parallelism() {
    let _lock = CALIBRATION.lock().unwrap();
    let single = SourceArray::from([Dipole::<f64>::default()]);
    let multiple = SourceArray::from([Dipole::<f64>::default(), Dipole::default()]);
    let points = [Point3::new(1.0, 2.0, 3.0)];
    set_instrumentation(true);
    reset_branch_counts();
    set_execution_mode(Parallel);

    let field = single.compute_B_batch(&points);
    assert_eq!(multiple.compute_B_batch(&points), vec![field[0] * 2.0]);
    assert_eq!(collection_branch_counts(), (1, 1));

    set_execution_mode(Auto);
    set_collection_execution_mode(Auto);
    set_instrumentation(false);
}

#[test]
fn collection_modes_preserve_fields_and_respect_primitive_serial_override() {
    let _lock = CALIBRATION.lock().unwrap();
    let primitive = Dipole::<f64>::default();
    let collection = SourceArray::from([primitive.clone(), primitive.clone()]);
    let point = Point3::new(1.0, 2.0, 3.0);
    let expected = vec![primitive.compute_B(point) * 2.0];
    set_instrumentation(true);
    for (primitive_mode, collection_mode, counts) in [
        (Auto, Serial, (1, 0)),
        (Auto, Parallel, (0, 1)),
        (Serial, Parallel, (1, 0)),
    ] {
        set_execution_mode(primitive_mode);
        set_collection_execution_mode(collection_mode);
        reset_branch_counts();
        assert_eq!(collection.compute_B_batch(&[point]), expected);
        assert_eq!(collection_branch_counts(), counts);
    }
    set_execution_mode(Auto);
    set_collection_execution_mode(Auto);
    set_instrumentation(false);
}

#[test]
fn serial_and_parallel_superposition_match_individual_fields() {
    let _lock = CALIBRATION.lock().unwrap();
    let positions = [Point3::origin(), Point3::new(-1.0, 0.5, 0.0)];
    let orientations = [UnitQuaternion::identity(); 2];
    let moments = [Vector3::z(), Vector3::x()];
    let points = [Point3::new(1.0, 2.0, 3.0), Point3::new(-2.0, 1.0, 4.0)];
    let expected: Vec<_> = points
        .iter()
        .map(|&point| {
            positions
                .iter()
                .zip(moments)
                .map(|(&position, moment)| {
                    Dipole::default()
                        .with_position(position)
                        .with_moment(moment)
                        .compute_B(point)
                })
                .sum::<Vector3<f64>>()
        })
        .collect();
    for mode in [Serial, Parallel] {
        set_execution_mode(mode);
        let mut fields = [Vector3::zeros(); 2];
        magba::fields::sum_multiple_dipole_B(
            &points,
            &positions,
            &orientations,
            &moments,
            &mut fields,
        );
        assert_eq!(fields.as_slice(), expected);
    }
    set_execution_mode(Auto);
}
