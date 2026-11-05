//! Independent reference regression tests for cylinder magnet projection and
//! the circular loop's fused Bulirsch CEL evaluator.
//!
//! Fixture provenance: MagPyLib 6.1.0 (Python), using independent
//! `magpylib.magnet.Cylinder.getB` and `magpylib.current.Circle.getB` calls.
//! The expected numbers were generated offline, not by Magba, and are stored
//! with 17 significant digits in tests/data. All positions are metres and
//! B-fields are tesla. The rotated source uses the same physical rotation
//! vector in MagPyLib's SciPy Rotation and nalgebra's UnitQuaternion.

use magba::fields::{circular_B, circular_B_batch, cylinder_B, cylinder_B_batch};
use nalgebra::{Point3, UnitQuaternion, Vector3};

const CYLINDER_CSV: &str = include_str!("data/cylinder_projection_magpylib.csv");
const CIRCULAR_CSV: &str = include_str!("data/circular_fused_magpylib.csv");

const DIAMETER: f64 = 0.02;
const CYLINDER_HEIGHT: f64 = 0.02;
const LOOP_CURRENT: f64 = 2.5;

#[derive(Clone, Debug)]
struct ReferenceCase {
    scenario: String,
    name: String,
    point: Point3<f64>,
    expected: Vector3<f64>,
}

fn load_cases(csv: &str) -> Vec<ReferenceCase> {
    let mut cases = Vec::new();
    for line in csv.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') || line.starts_with("scenario,") {
            continue;
        }
        let columns: Vec<_> = line.split(',').map(str::trim).collect();
        assert_eq!(columns.len(), 8, "unexpected fixture row: {line}");
        let numbers: Vec<f64> = columns[2..]
            .iter()
            .map(|field| field.parse::<f64>().expect("finite decimal fixture"))
            .collect();
        assert!(
            numbers.iter().all(|v| v.is_finite()),
            "nonfinite fixture: {line}"
        );
        cases.push(ReferenceCase {
            scenario: columns[0].into(),
            name: columns[1].into(),
            point: Point3::new(numbers[0], numbers[1], numbers[2]),
            expected: Vector3::new(numbers[3], numbers[4], numbers[5]),
        });
    }
    cases
}

fn source_pose_f64(rotated: bool) -> (Point3<f64>, UnitQuaternion<f64>) {
    if rotated {
        (
            Point3::new(0.003, -0.004, 0.002),
            UnitQuaternion::from_scaled_axis(Vector3::new(0.24, -0.15, 0.37)),
        )
    } else {
        (Point3::origin(), UnitQuaternion::identity())
    }
}

fn source_pose_f32(rotated: bool) -> (Point3<f32>, UnitQuaternion<f32>) {
    if rotated {
        (
            Point3::new(0.003, -0.004, 0.002),
            UnitQuaternion::from_scaled_axis(Vector3::new(0.24, -0.15, 0.37)),
        )
    } else {
        (Point3::origin(), UnitQuaternion::identity())
    }
}

fn cylinder_polarization_f64(scenario: &str) -> Vector3<f64> {
    match scenario {
        "mixed_local" | "mixed_rotated" => Vector3::new(0.8, -0.6, 1.2),
        "axial_local" => Vector3::new(0.0, 0.0, 1.2),
        "transverse_local" => Vector3::new(0.8, -0.6, 0.0),
        _ => panic!("unknown cylinder scenario: {scenario}"),
    }
}

fn assert_components_close(
    label: &str,
    actual: Vector3<f64>,
    expected: Vector3<f64>,
    abs_tol: f64,
    rel_tol: f64,
) {
    assert!(
        actual.iter().all(|v| v.is_finite()),
        "{label}: nonfinite computed field {actual:?}"
    );
    for axis in 0..3 {
        let error = (actual[axis] - expected[axis]).abs();
        let bound = abs_tol + rel_tol * expected[axis].abs();
        assert!(
            error <= bound,
            "{label} axis {axis}: got {}, expected {}, error {error}, bound {bound}",
            actual[axis],
            expected[axis]
        );
    }
}

#[test]
fn cylinder_mixed_projection_f64_matches_independent_magpylib() {
    let cases = load_cases(CYLINDER_CSV);
    assert_eq!(cases.len(), 52);
    for scenario in [
        "mixed_local",
        "axial_local",
        "transverse_local",
        "mixed_rotated",
    ] {
        let selected: Vec<_> = cases
            .iter()
            .filter(|case| case.scenario == scenario)
            .collect();
        assert!(!selected.is_empty());
        let (position, orientation) = source_pose_f64(scenario == "mixed_rotated");
        let polarization = cylinder_polarization_f64(scenario);
        let points: Vec<_> = selected.iter().map(|case| case.point).collect();
        let mut batch = vec![Vector3::zeros(); points.len()];
        cylinder_B_batch(
            &points,
            position,
            orientation,
            polarization,
            DIAMETER,
            CYLINDER_HEIGHT,
            &mut batch,
        );
        for (case, batched) in selected.iter().zip(batch.iter()) {
            let direct = cylinder_B(
                case.point,
                position,
                orientation,
                polarization,
                DIAMETER,
                CYLINDER_HEIGHT,
            );
            let label = format!("{} / {}", case.scenario, case.name);
            // Stricter for regular points; near the rim, the analytical
            // elliptic expression is ill-conditioned but never singular here.
            let (abs, rel) = if case.name.contains("rim") || case.name.contains("side_r") {
                (3e-7, 3e-6)
            } else {
                (2e-9, 2e-7)
            };
            assert_components_close(&label, direct, case.expected, abs, rel);
            assert_components_close(&label, *batched, direct, 3e-12, 3e-12);
        }
    }
}

#[test]
fn cylinder_mixed_projection_f32_matches_f64_reference_away_from_rim() {
    // The f64 MagPyLib values are also an independent single-precision
    // reference. Exclude near-rim points where f32 elliptic cancellation
    // dominates and the source has a separate boundary convention.
    let cases = load_cases(CYLINDER_CSV);
    let mut tested = 0;
    for scenario in [
        "mixed_local",
        "axial_local",
        "transverse_local",
        "mixed_rotated",
    ] {
        let selected: Vec<_> = cases
            .iter()
            .filter(|case| {
                case.scenario == scenario
                    && !case.name.contains("rim")
                    && !case.name.contains("side_r")
                    && !case.name.contains("sidewall")
            })
            .collect();
        let (position, orientation) = source_pose_f32(scenario == "mixed_rotated");
        let polarization = cylinder_polarization_f64(scenario).map(|v| v as f32);
        let points: Vec<_> = selected
            .iter()
            .map(|case| case.point.map(|v| v as f32))
            .collect();
        let mut batch = vec![Vector3::zeros(); points.len()];
        cylinder_B_batch(
            &points,
            position,
            orientation,
            polarization,
            DIAMETER as f32,
            CYLINDER_HEIGHT as f32,
            &mut batch,
        );
        for (case, batched) in selected.iter().zip(batch.iter()) {
            let point = case.point.map(|v| v as f32);
            let direct = cylinder_B(
                point,
                position,
                orientation,
                polarization,
                DIAMETER as f32,
                CYLINDER_HEIGHT as f32,
            );
            let label = format!("f32 {} / {}", case.scenario, case.name);
            // Keep a wider absolute budget only near the small-r Taylor
            // transition, where f32 subtraction of elliptic terms is most
            // sensitive. Ordinary points should not pass with a weak field.
            let abs_tol = if case.name.contains("taylor_r") || case.name.contains("general_r") {
                3e-4
            } else {
                1e-5
            };
            assert_components_close(
                &label,
                direct.map(|v| v as f64),
                case.expected,
                abs_tol,
                5e-3,
            );
            assert_components_close(
                &label,
                batched.map(|v| v as f64),
                direct.map(|v| v as f64),
                2e-6,
                5e-6,
            );
            tested += 1;
        }
    }
    assert!(tested >= 30, "too few single-precision regression cases");
}

#[test]
fn axial_cylinder_on_axis_matches_independent_closed_form() {
    // A uniformly axially polarized finite cylinder is equivalent to two
    // uniformly charged end disks. Their on-axis B field is elementary; no
    // MagPyLib or Magba field evaluator is involved in the expected result.
    // See Derby & Olbert, American Journal of Physics 78 (2010), 229–235.
    let radius = DIAMETER / 2.0;
    let half_height = CYLINDER_HEIGHT / 2.0;
    let polarization = Vector3::new(0.0, 0.0, 1.2);
    let z_values = [-0.03, -0.007, 0.0, 0.007, 0.03];
    for z in z_values {
        let positive = z + half_height;
        let negative = z - half_height;
        let expected_z = polarization.z / 2.0
            * (positive / (radius * radius + positive * positive).sqrt()
                - negative / (radius * radius + negative * negative).sqrt());
        let actual = cylinder_B(
            Point3::new(0.0, 0.0, z),
            Point3::origin(),
            UnitQuaternion::identity(),
            polarization,
            DIAMETER,
            CYLINDER_HEIGHT,
        );
        assert_components_close(
            &format!("axial closed form z={z}"),
            actual,
            Vector3::new(0.0, 0.0, expected_z),
            2e-13,
            2e-12,
        );
    }
}

#[test]
fn zero_polarization_and_zero_current_are_exactly_zero_off_singularities() {
    // Independent physical invariant: B is linear in source polarization
    // and current. Include axis, non-axis and translated/rotated observers.
    let points = [
        Point3::origin(),
        Point3::new(0.001, 0.002, 0.003),
        Point3::new(0.025, -0.011, 0.017),
    ];
    let (position, orientation) = source_pose_f64(true);
    let mut out = [Vector3::repeat(f64::NAN); 3];
    cylinder_B_batch(
        &points,
        position,
        orientation,
        Vector3::zeros(),
        DIAMETER,
        CYLINDER_HEIGHT,
        &mut out,
    );
    for (&point, batched) in points.iter().zip(out.iter()) {
        let direct = cylinder_B(
            point,
            position,
            orientation,
            Vector3::zeros(),
            DIAMETER,
            CYLINDER_HEIGHT,
        );
        assert_eq!(direct, Vector3::zeros());
        assert_eq!(*batched, direct);
    }

    circular_B_batch(&points, position, orientation, DIAMETER, 0.0, &mut out);
    for (&point, batched) in points.iter().zip(out.iter()) {
        let direct = circular_B(point, position, orientation, DIAMETER, 0.0);
        assert_eq!(direct, Vector3::zeros());
        assert_eq!(*batched, direct);
    }
}

fn circular_current(scenario: &str) -> f64 {
    match scenario {
        "positive_local" | "positive_rotated" => LOOP_CURRENT,
        "negative_local" => -1.25,
        _ => panic!("unknown circular scenario: {scenario}"),
    }
}

#[test]
fn circular_fused_cel_f64_matches_independent_magpylib() {
    let cases = load_cases(CIRCULAR_CSV);
    assert_eq!(cases.len(), 32);
    for scenario in ["positive_local", "negative_local", "positive_rotated"] {
        let selected: Vec<_> = cases
            .iter()
            .filter(|case| case.scenario == scenario)
            .collect();
        assert!(!selected.is_empty());
        let (position, orientation) = source_pose_f64(scenario == "positive_rotated");
        let current = circular_current(scenario);
        let points: Vec<_> = selected.iter().map(|case| case.point).collect();
        let mut batch = vec![Vector3::zeros(); points.len()];
        circular_B_batch(
            &points,
            position,
            orientation,
            DIAMETER,
            current,
            &mut batch,
        );
        for (case, batched) in selected.iter().zip(batch.iter()) {
            let direct = circular_B(case.point, position, orientation, DIAMETER, current);
            let label = format!("{} / {}", case.scenario, case.name);
            assert_components_close(&label, direct, case.expected, 2e-10, 2e-7);
            assert_components_close(&label, *batched, direct, 5e-13, 5e-12);
        }
    }
}

#[test]
fn circular_fused_cel_f32_matches_f64_reference() {
    let cases = load_cases(CIRCULAR_CSV);
    let mut tested = 0;
    for scenario in ["positive_local", "negative_local", "positive_rotated"] {
        let selected: Vec<_> = cases
            .iter()
            .filter(|case| case.scenario == scenario)
            .collect();
        let (position, orientation) = source_pose_f32(scenario == "positive_rotated");
        let current = circular_current(scenario) as f32;
        let points: Vec<_> = selected
            .iter()
            .map(|case| case.point.map(|v| v as f32))
            .collect();
        let mut batch = vec![Vector3::zeros(); points.len()];
        circular_B_batch(
            &points,
            position,
            orientation,
            DIAMETER as f32,
            current,
            &mut batch,
        );
        for (case, batched) in selected.iter().zip(batch.iter()) {
            let point = case.point.map(|v| v as f32);
            let direct = circular_B(point, position, orientation, DIAMETER as f32, current);
            let label = format!("f32 {} / {}", case.scenario, case.name);
            // Allow single-precision error near the loop where k^2 approaches
            // one and the CEL terms are individually large. Far-field
            // magnitudes are below 1 µT, so a blanket 5e-7 T tolerance
            // would erroneously accept a zero output there.
            let abs_tol = if case.name.contains("far") {
                1e-8
            } else {
                5e-7
            };
            assert_components_close(
                &label,
                direct.map(|v| v as f64),
                case.expected,
                abs_tol,
                6e-3,
            );
            assert_components_close(
                &label,
                batched.map(|v| v as f64),
                direct.map(|v| v as f64),
                1e-8,
                5e-6,
            );
            tested += 1;
        }
    }
    assert_eq!(tested, 32);
}

#[cfg(feature = "rayon")]
#[test]
fn large_parallel_batches_match_independent_reference() {
    // Repeated deterministic reference points exceed both per-source Rayon
    // thresholds. This catches differences between parallel batch evaluation
    // and scalar reference behavior without relying on either as the oracle.
    const N: usize = 512;
    let cylinder_cases: Vec<_> = load_cases(CYLINDER_CSV)
        .into_iter()
        .filter(|case| case.scenario == "mixed_local")
        .collect();
    let cylinder_points: Vec<_> = (0..N)
        .map(|i| cylinder_cases[i % cylinder_cases.len()].point)
        .collect();
    let mut cylinder_batch = vec![Vector3::zeros(); N];
    cylinder_B_batch(
        &cylinder_points,
        Point3::origin(),
        UnitQuaternion::identity(),
        Vector3::new(0.8, -0.6, 1.2),
        DIAMETER,
        CYLINDER_HEIGHT,
        &mut cylinder_batch,
    );
    for (i, actual) in cylinder_batch.into_iter().enumerate() {
        let case = &cylinder_cases[i % cylinder_cases.len()];
        let (abs, rel) = if case.name.contains("rim") || case.name.contains("side_r") {
            (3e-7, 3e-6)
        } else {
            (2e-9, 2e-7)
        };
        assert_components_close(
            &format!("parallel cylinder {}", case.name),
            actual,
            case.expected,
            abs,
            rel,
        );
    }

    let circular_cases: Vec<_> = load_cases(CIRCULAR_CSV)
        .into_iter()
        .filter(|case| case.scenario == "positive_local")
        .collect();
    let circular_points: Vec<_> = (0..N)
        .map(|i| circular_cases[i % circular_cases.len()].point)
        .collect();
    let mut circular_batch = vec![Vector3::zeros(); N];
    circular_B_batch(
        &circular_points,
        Point3::origin(),
        UnitQuaternion::identity(),
        DIAMETER,
        LOOP_CURRENT,
        &mut circular_batch,
    );
    for (i, actual) in circular_batch.into_iter().enumerate() {
        let case = &circular_cases[i % circular_cases.len()];
        assert_components_close(
            &format!("parallel circular {}", case.name),
            actual,
            case.expected,
            2e-10,
            2e-7,
        );
    }
}
