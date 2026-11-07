//! Independent Wolfram high-precision field references (SI units).

use magba::prelude::{CircularCurrent, CuboidMagnet, Source};
use nalgebra::{Point3, UnitQuaternion};

fn fixtures(name: &str, columns: &[&str]) -> Vec<(String, Vec<f64>)> {
    let path = format!(
        "{}/testing/data/high-precision/{name}.csv",
        env!("CARGO_MANIFEST_DIR")
    );
    let mut reader = csv::Reader::from_path(&path)
        .unwrap_or_else(|err| panic!("cannot read {path} (checkout testing submodule): {err}"));
    assert_eq!(
        reader.headers().unwrap().iter().collect::<Vec<_>>(),
        columns,
        "{path}: unexpected CSV columns"
    );
    let rows = reader
        .records()
        .map(|record| {
            let row = record.unwrap_or_else(|err| panic!("{path}: {err}"));
            let case = row[0].to_owned();
            assert!(!case.is_empty(), "{path}: empty case name");
            let values = (1..columns.len())
                .map(|i| {
                    row[i].parse::<f64>().unwrap_or_else(|err| {
                        panic!("{path}: {case} {} = {:?}: {err}", columns[i], &row[i])
                    })
                })
                .collect();
            (case, values)
        })
        .collect::<Vec<_>>();
    assert!(!rows.is_empty(), "{path}: no reference cases");
    rows
}

fn assert_field(
    case: &str,
    precision: &str,
    actual: &[f64],
    expected: [f64; 3],
    rel_tol: f64,
    abs_tol: f64,
) {
    let scale = expected.iter().copied().map(f64::abs).fold(0.0, f64::max);
    let allowed = abs_tol + rel_tol * scale;
    for (axis, (&got, want)) in ["bx", "by", "bz"].iter().zip(actual.iter().zip(expected)) {
        assert!(
            got.is_finite() && want.is_finite() && (got - want).abs() <= allowed,
            "{case} {precision} {axis}: got {got:.16e}, expected {want:.16e}, error {:.3e} > tolerance {allowed:.3e} T",
            (got - want).abs()
        );
    }
}

#[test]
fn circular_current_wolfram_reference() {
    let columns = [
        "case", "diameter", "current", "x", "y", "z", "bx", "by", "bz",
    ];
    for (case, v) in fixtures("circular_current", &columns) {
        let expected = [v[5], v[6], v[7]];
        let f64_source = CircularCurrent::new([0.0; 3], UnitQuaternion::identity(), v[0], v[1]);
        let f32_source = CircularCurrent::new(
            [0.0; 3],
            UnitQuaternion::identity(),
            v[0] as f32,
            v[1] as f32,
        );
        let position64 = Point3::new(v[2], v[3], v[4]);
        let position32 = Point3::new(v[2] as f32, v[3] as f32, v[4] as f32);
        assert_field(
            &case,
            "f64",
            f64_source.compute_B(position64).as_slice(),
            expected,
            1e-12,
            1e-19,
        );
        assert_field(
            &case,
            "f32",
            f32_source.compute_B(position32).map(f64::from).as_slice(),
            expected,
            3e-5,
            1e-12,
        );
    }
}

#[test]
fn cuboid_wolfram_reference() {
    let columns = [
        "case", "jx", "jy", "jz", "dx", "dy", "dz", "x", "y", "z", "bx", "by", "bz",
    ];
    for (case, v) in fixtures("cuboid", &columns) {
        let expected = [v[9], v[10], v[11]];
        let f64_source = CuboidMagnet::new(
            [0.0; 3],
            UnitQuaternion::identity(),
            [v[0], v[1], v[2]],
            [v[3], v[4], v[5]],
        );
        let f32_source = CuboidMagnet::new(
            [0.0; 3],
            UnitQuaternion::identity(),
            [v[0] as f32, v[1] as f32, v[2] as f32],
            [v[3] as f32, v[4] as f32, v[5] as f32],
        );
        let position64 = Point3::new(v[6], v[7], v[8]);
        let position32 = Point3::new(v[6] as f32, v[7] as f32, v[8] as f32);
        assert_field(
            &case,
            "f64",
            f64_source.compute_B(position64).as_slice(),
            expected,
            1e-12,
            1e-15,
        );
        assert_field(
            &case,
            "f32",
            f32_source.compute_B(position32).map(f64::from).as_slice(),
            expected,
            5e-5,
            1e-8,
        );
    }
}
