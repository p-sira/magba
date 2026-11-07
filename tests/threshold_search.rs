#[allow(dead_code)]
#[path = "../benches/common.rs"]
mod common;

use common::{ExitCond, Timing, find_threshold};

#[test]
fn search_finds_crossing_without_duplicate_measurements() {
    let mut sampled = std::collections::HashSet::new();
    let found = find_threshold(240, 300, 5, |n| {
        assert!(sampled.insert(n), "duplicate sample: {n}");
        Some(Timing {
            parallel_ns: 150.0,
            serial_ns: n as f64,
        })
    })
    .unwrap();

    assert!(matches!(found.exit, ExitCond::Converged));
    assert_eq!(found.threshold, 150);
    assert_eq!(found.timing.ratio(), 1.0);
}

#[test]
fn search_respects_upper_bound_with_no_crossing() {
    let found = find_threshold(21, 96, 7, |n| {
        Some(Timing {
            parallel_ns: n as f64 * 2.0,
            serial_ns: n as f64,
        })
    })
    .unwrap();

    assert!(matches!(found.exit, ExitCond::MaxThres));
    assert_eq!(found.threshold, 91);
    assert_eq!(found.timing.ratio(), 2.0);
}

#[test]
fn search_returns_measured_minimum_when_always_faster() {
    let found = find_threshold(100, 300, 5, |_| {
        Some(Timing {
            parallel_ns: 1.0,
            serial_ns: 2.0,
        })
    })
    .unwrap();

    assert!(matches!(found.exit, ExitCond::MinStep));
    assert_eq!(found.threshold, 5);
}
