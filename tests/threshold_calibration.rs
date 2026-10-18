#![cfg(feature = "threshold-calibration")]

use std::sync::Mutex;

use magba::{
    base::Source,
    collections::SourceArray,
    magnets::Dipole,
    threshold_calibration::{self::*, ExecutionMode::*},
};
use nalgebra::Point3;

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
