#![cfg(feature = "std")]

//! Independent physical-reference regression tests for the cuboid trace shortcut.
//!
//! `data/cuboid_trace_reference.csv` contains 76 deterministic cases generated
//! independently of Magba's cuboid atan2/log formula. The reference is the
//! surface-magnetic-charge integral over all six faces of a uniformly magnetized
//! cuboid, `B = M * inside + (1 / 4π) ∮ (M·n) (p-s) / |p-s|³ dA`.
//! Regenerate or verify with `python tests/data/cuboid_trace_generator.py
//! --write` or `--verify` (requires NumPy; generator is not a test dependency).
//! One face coordinate was integrated analytically, while the remaining
//! coordinate was integrated with adaptive 16/32-node Gauss–Legendre quadrature
//! to an absolute agreement target of 2e-12. The generator used NumPy 2.5.4;
//! the frozen CSV is checked in so tests require no Python runtime.
//!
//! As an independent cross-check of the generator, its output for the first
//! `testing/data/cuboid.csv` observation (the existing MagpyLib reference
//! pipeline) agrees to 2.2e-16 T. Rotations are Rodrigues rotation vectors,
//! matching `UnitQuaternion::from_scaled_axis`; polarization is expressed in
//! the cuboid's local axes and expected B in world axes.
//!
//! The points cover mixed-sign XYZ polarization, all three single-axis
//! polarization branches, zero polarization, different rotations/translations,
//! both sides of faces and nearby edges/corners, the center, and distant points.
//! Face/edge/corner *nearby* samples stay >= 3e-4 m from the boundary to
//! preserve meaningful f32 accuracy. Six extra exact-face/non-edge samples
//! reference the physical half-value convention, independently calculated as
//! the two-sided surface-charge limit and Richardson extrapolated from offsets
//! of 1e-5 and 2e-5 m. These deliberately exercise the trace fallback for
//! `xma == 0`, `ypb == 0`, and `zpc == 0`. The exact-face test checks only the
//! unambiguous normal B component: the direct kernel's tangential boundary
//! convention chooses an inside or outside limit depending on which face axis
//! is involved, whereas the independent fixture stores the two-sided average.

use magba::fields::{cuboid_B, cuboid_B_batch};
use nalgebra::{Point3, UnitQuaternion, Vector3};

type Numbers = [f64; 18];

struct CuboidCase64 {
    dimensions: Vector3<f64>,
    polarization: Vector3<f64>,
    position: Point3<f64>,
    orientation: UnitQuaternion<f64>,
    observer: Point3<f64>,
    expected: Vector3<f64>,
}

fn fixture_cases() -> impl Iterator<Item = (&'static str, Numbers)> {
    include_str!("data/cuboid_trace_reference.csv")
        .lines()
        .skip(1)
        .map(|line| {
            let (name, data) = line.split_once(',').expect("case name and CSV data");
            let values: Vec<_> = data
                .split(',')
                .map(|v| v.parse::<f64>().expect("valid reference number"))
                .collect();
            let values = values.try_into().expect("18 numeric columns");
            (name, values)
        })
}

fn unpack64(n: Numbers) -> CuboidCase64 {
    let dim = Vector3::new(n[0], n[1], n[2]);
    let pol = Vector3::new(n[3], n[4], n[5]);
    let pos = Point3::new(n[6], n[7], n[8]);
    let ori = UnitQuaternion::from_scaled_axis(Vector3::new(n[9], n[10], n[11]));
    let local = Vector3::new(n[12], n[13], n[14]);
    let world = Point3::from(pos.coords + ori * local);
    let expected = Vector3::new(n[15], n[16], n[17]);
    CuboidCase64 {
        dimensions: dim,
        polarization: pol,
        position: pos,
        orientation: ori,
        observer: world,
        expected,
    }
}

fn f32_tolerance(name: &str, expected: f32) -> f32 {
    // Discontinuity-adjacent samples are limited by f32 geometry precision;
    // interior/far-field cases should not inherit that absolute allowance.
    let near_boundary = ["face_", "edge_", "corner_", "outside_near"]
        .iter()
        .any(|tag| name.contains(tag));
    if near_boundary {
        2e-4_f32 + 4e-4 * expected.abs()
    } else {
        1e-6_f32 + 3e-4 * expected.abs()
    }
}

#[test]
fn cuboid_trace_matches_independent_surface_charge_integrals_f64_f32() {
    let mut count = 0;
    for (name, values) in fixture_cases() {
        let case = unpack64(values);
        let CuboidCase64 {
            dimensions: dim,
            polarization: pol,
            position: pos,
            orientation: ori,
            observer: world,
            expected,
        } = case;
        let actual = cuboid_B(world, pos, ori, pol, dim);
        let normal_axis = if name.contains("exact_face_") {
            Some(match name.rsplit('_').next() {
                Some("x") => 0,
                Some("y") => 1,
                Some("z") => 2,
                _ => unreachable!("exact-face case must name its normal axis"),
            })
        } else {
            None
        };
        for component in 0..3 {
            assert!(actual[component].is_finite(), "nonfinite f64 {name}");
            if normal_axis.is_some_and(|axis| component != axis) {
                continue;
            }
            let error = (actual[component] - expected[component]).abs();
            let tolerance = 3e-10 + 1e-9 * expected[component].abs();
            assert!(
                error < tolerance,
                "f64 {name} component={component}: actual={actual:?} ref={expected:?}, error={error:e}"
            );
        }

        // Construct the world observation from f32 local inputs, rather than
        // casting the f64 world position, to exercise f32 pose transforms too.
        let dim32 = dim.cast::<f32>();
        let pol32 = pol.cast::<f32>();
        let pos32 = pos.cast::<f32>();
        let ori32 = UnitQuaternion::from_scaled_axis(Vector3::new(
            values[9] as f32,
            values[10] as f32,
            values[11] as f32,
        ));
        let local32 = Vector3::new(values[12] as f32, values[13] as f32, values[14] as f32);
        let world32 = Point3::from(pos32.coords + ori32 * local32);
        let actual32 = cuboid_B(world32, pos32, ori32, pol32, dim32);
        for component in 0..3 {
            assert!(actual32[component].is_finite(), "nonfinite f32 {name}");
            if normal_axis.is_some_and(|axis| component != axis) {
                continue;
            }
            let target = expected[component] as f32;
            if name.starts_with("zero_pol_") {
                assert_eq!(actual32[component], 0.0, "zero field {name}");
                continue;
            }
            let error = (actual32[component] - target).abs();
            let tolerance = f32_tolerance(name, target);
            assert!(
                error < tolerance,
                "f32 {name} component={component}: actual={actual32:?} ref={expected:?}, error={error:e}"
            );
        }
        count += 1;
    }
    assert_eq!(count, 76, "all fixed surface-integral cases must be used");
}

#[test]
fn cuboid_batch_over_parallel_threshold_matches_independent_reference() {
    // Four repetitions of 24 distinct mixed-polarization identity-pose cases
    // exceed the cuboid batch kernel's Rayon threshold of 80 positions.
    let selected: Vec<_> = fixture_cases()
        .filter(|(name, _)| {
            name.starts_with("identity_mixed_") && !name.starts_with("identity_mixed_exact_face_")
        })
        .collect();
    assert_eq!(selected.len(), 24);

    let mut points64 = Vec::new();
    let mut expected64 = Vec::new();
    for _ in 0..4 {
        for (_, n) in &selected {
            let case = unpack64(*n);
            points64.push(case.observer);
            expected64.push(case.expected);
        }
    }
    let CuboidCase64 {
        dimensions: dim,
        polarization: pol,
        position: pos,
        orientation: ori,
        ..
    } = unpack64(selected[0].1);
    let mut actual64 = vec![Vector3::<f64>::zeros(); points64.len()];
    cuboid_B_batch(&points64, pos, ori, pol, dim, &mut actual64);
    for (index, (actual, expected)) in actual64.iter().zip(&expected64).enumerate() {
        for component in 0..3 {
            let tolerance = 3e-10 + 1e-9 * expected[component].abs();
            assert!(
                (actual[component] - expected[component]).abs() < tolerance,
                "batch index={index}: actual={actual:?}, ref={expected:?}"
            );
        }
    }

    let points32: Vec<_> = points64.iter().map(|p| p.cast::<f32>()).collect();
    let mut actual32 = vec![Vector3::<f32>::zeros(); points32.len()];
    cuboid_B_batch(
        &points32,
        pos.cast(),
        UnitQuaternion::<f32>::identity(),
        pol.cast(),
        dim.cast(),
        &mut actual32,
    );
    for (index, (actual, expected)) in actual32.iter().zip(&expected64).enumerate() {
        let case_name = selected[index % selected.len()].0;
        for component in 0..3 {
            let target = expected[component] as f32;
            let tolerance = f32_tolerance(case_name, target);
            assert!(
                (actual[component] - target).abs() < tolerance,
                "f32 batch index={index}, case={case_name}: actual={actual:?}, ref={expected:?}"
            );
        }
    }
}

#[test]
fn cuboid_exact_edge_and_zero_polarization_controls() {
    let dimensions = Vector3::new(0.12, 0.20, 0.28);
    let polarization = Vector3::new(0.37, -0.61, 0.89);
    // Exact edges/corners follow Magba's documented zero-vector convention,
    // whereas the independent integral fixture intentionally avoids them.
    for edge in [
        Point3::new(0.03, 0.10, 0.14),
        Point3::new(0.06, -0.10, 0.031),
        Point3::new(-0.06, -0.10, -0.14),
    ] {
        assert_eq!(
            cuboid_B(
                edge,
                Point3::origin(),
                UnitQuaternion::identity(),
                polarization,
                dimensions,
            ),
            Vector3::zeros(),
            "edge={edge:?}"
        );
    }
    assert_eq!(
        cuboid_B(
            Point3::new(f64::NAN, 0.03, 0.05),
            Point3::origin(),
            UnitQuaternion::identity(),
            Vector3::zeros(),
            Vector3::new(0.0, f64::INFINITY, 0.0),
        ),
        Vector3::zeros(),
        "zero polarization must bypass degenerate/nonfinite geometry"
    );
}
