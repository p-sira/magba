/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use magba::base::mesh::TriMesh;
use magba::prelude::*;
use nalgebra::{Point3, UnitQuaternion, Vector3, vector};
use std::error::Error;
use std::f64::consts::PI;
use std::path::Path;

fn create_source<T: magba::base::Float>(name: &str) -> Box<dyn Source<T>> {
    let f = |v: f64| T::from(v).unwrap();

    // Pose used in generate_tests! macro
    let pos = [f(0.1), f(0.2), f(0.3)];
    let rot = UnitQuaternion::from_scaled_axis(vector![f(PI / 7.0), f(PI / 6.0), f(PI / 5.0)]);
    let pol = [f(1.0), f(2.0), f(3.0)];

    let tetrahedron_vertices = [
        vector![f(-0.1), f(-0.1), f(-0.1)],
        vector![f(0.1), f(-0.1), f(-0.1)],
        vector![f(0.0), f(0.1), f(-0.1)],
        vector![f(0.0), f(0.0), f(0.1)],
    ];

    match name {
        "CircularCurrent" => Box::new(CircularCurrent::new(pos, rot, f(1.0), f(1.0))),
        "PathCurrent" => Box::new(PathCurrent::new(
            pos,
            rot,
            f(1.0),
            vec![
                vector![f(-0.1), f(-0.1), f(-0.1)],
                vector![f(0.1), f(-0.1), f(-0.1)],
                vector![f(0.0), f(0.1), f(-0.1)],
                vector![f(0.0), f(0.0), f(0.1)],
            ],
        )),
        "SheetCurrent" => {
            let mesh = TriMesh::<T>::new_unchecked(
                tetrahedron_vertices,
                vec![[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2]],
            );
            Box::new(SheetCurrent::new(
                pos,
                rot,
                vec![vector![f(1.0), f(2.0), f(3.0)]; 4],
                mesh,
            ))
        }
        "TriangleCurrent" => Box::new(TriangleCurrent::new(
            pos,
            rot,
            vector![f(1.0), f(2.0), f(3.0)],
            [
                vector![f(-0.1), f(-0.1), f(-0.1)],
                vector![f(0.1), f(-0.1), f(-0.1)],
                vector![f(0.0), f(0.1), f(-0.1)],
            ],
        )),
        "CylinderMagnet" => Box::new(CylinderMagnet::new(pos, rot, pol, f(0.1), f(0.2))),
        "CylinderSegmentMagnet" => Box::new(CylinderSegmentMagnet::new(
            pos,
            rot,
            pol,
            [f(0.01), f(0.03), f(0.02), f(-30.0), f(120.0)],
        )),
        "CuboidMagnet" => Box::new(CuboidMagnet::new(pos, rot, pol, [f(0.1), f(0.2), f(0.3)])),
        "Dipole" => Box::new(Dipole::new(pos, rot, pol)),
        "SphereMagnet" => Box::new(SphereMagnet::new(pos, rot, pol, f(0.1))),
        "TetrahedronMagnet" => {
            Box::new(TetrahedronMagnet::new(pos, rot, pol, tetrahedron_vertices))
        }
        "TriangleMagnet" => Box::new(TriangleMagnet::new(
            pos,
            rot,
            pol,
            [
                vector![f(-0.1), f(-0.1), f(-0.1)],
                vector![f(0.1), f(-0.1), f(0.1)],
                vector![f(0.0), f(0.2), f(0.0)],
            ],
        )),
        "MeshMagnet" => {
            let mesh = TriMesh::<T>::new_unchecked(
                tetrahedron_vertices,
                vec![[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2]],
            );
            Box::new(MeshMagnet::new(pos, rot, pol, mesh))
        }
        _ => panic!("Unknown magnet: {}", name),
    }
}

fn read_vectors(path: &Path) -> Result<Vec<Vector3<f64>>, Box<dyn Error>> {
    let mut reader = csv::ReaderBuilder::new()
        .has_headers(false)
        .from_path(path)?;
    reader
        .records()
        .enumerate()
        .map(|(row, record)| {
            let record = record?;
            if record.len() != 3 {
                return Err(format!(
                    "{} row {}: expected three components",
                    path.display(),
                    row + 1
                )
                .into());
            }
            let mut vector = Vector3::zeros();
            for (component, raw) in vector.iter_mut().zip(record.iter()) {
                *component = raw.parse()?;
            }
            if !vector.iter().all(|v: &f64| v.is_finite()) {
                return Err(format!("{} row {}: nonfinite vector", path.display(), row + 1).into());
            }
            Ok(vector)
        })
        .collect()
}

fn norm(v: Vector3<f64>) -> f64 {
    v.x.hypot(v.y).hypot(v.z)
}

// Keep the dimensionless relative error separate from the absolute error in T.
// A nonzero error against a zero reference has infinite relative error.
fn errors(actual: Vector3<f64>, expected: Vector3<f64>) -> (f64, f64) {
    let absolute = norm(actual - expected);
    let relative = if absolute == 0.0 {
        0.0
    } else {
        absolute / norm(expected)
    };
    (relative, absolute)
}

fn accuracy_row<T: magba::base::Float>(
    name: &str,
    points: &[Vector3<f64>],
    reference: &[Vector3<f64>],
) -> Result<String, Box<dyn Error>> {
    if points.is_empty() || reference.len() != points.len() {
        return Err(
            format!("{name}: expected one reference per observer in a nonempty grid").into(),
        );
    }
    let source = create_source::<T>(name);
    let observers: Vec<_> = points
        .iter()
        .map(|p| Point3::from(p.map(|v| T::from(v).unwrap())))
        .collect();
    let actual = source.compute_B_batch(&observers);
    let mut relative = Vec::with_capacity(points.len());
    let mut max_absolute = 0.0_f64;
    let mut worst = 0;
    let mut max_relative = -1.0_f64;
    for (i, (got, expected)) in actual.iter().zip(reference).enumerate() {
        let got = got.map(|v| v.to_f64().unwrap());
        if !got.iter().all(|v| v.is_finite()) {
            return Err(format!("{name}: nonfinite field at observer {:?}", points[i]).into());
        }
        let (rel, abs) = errors(got, *expected);
        relative.push(rel);
        max_absolute = max_absolute.max(abs);
        if rel > max_relative {
            max_relative = rel;
            worst = i;
        }
    }
    relative.sort_by(f64::total_cmp);
    let p = points[worst];
    Ok(format!(
        "| {name} | {:.3e} | {:.3e} | {max_relative:.3e} | {max_absolute:.3e} | ({:.6}, {:.6}, {:.6}) |\n",
        relative[relative.len() / 2],
        relative[relative.len() * 95 / 100],
        p.x,
        p.y,
        p.z
    ))
}

fn main() -> Result<(), Box<dyn Error>> {
    let sources = [
        ("CircularCurrent", "circularcurrent.csv"),
        ("PathCurrent", "polyline.csv"),
        ("SheetCurrent", "sheetcurrent.csv"),
        ("TriangleCurrent", "trianglecurrent.csv"),
        ("CylinderMagnet", "cylinder.csv"),
        ("CylinderSegmentMagnet", "cylindersegment.csv"),
        ("CuboidMagnet", "cuboid.csv"),
        ("Dipole", "dipole.csv"),
        ("SphereMagnet", "sphere.csv"),
        ("TetrahedronMagnet", "tetrahedron.csv"),
        ("TriangleMagnet", "triangle.csv"),
        ("MeshMagnet", "triangularmesh.csv"),
    ];
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let data = root.join("testing/data");
    let points = read_vectors(&data.join("points.csv"))?;
    let header = "| Source | Median relative | P95 relative | Max relative | Max absolute (T) | Worst relative observer (m) |\n|---|---:|---:|---:|---:|---|\n";
    let mut report64 = header.to_owned();
    let mut report32 = header.to_owned();
    for (name, file) in sources {
        let reference = read_vectors(&data.join(file))?;
        report64.push_str(&accuracy_row::<f64>(name, &points, &reference)?);
        report32.push_str(&accuracy_row::<f32>(name, &points, &reference)?);
    }
    let content = std::fs::read_to_string(root.join("tests/report_template.md"))?
        .replace(
            "{{TEST_ENV}}",
            &format!(
                "magba {} ({}, {})",
                env!("CARGO_PKG_VERSION"),
                std::env::consts::ARCH,
                std::env::consts::OS
            ),
        )
        .replace("{{ERROR_F64}}", report64.trim_end())
        .replace("{{ERROR_F32}}", report32.trim_end());
    std::fs::write(root.join("tests/README.md"), content)?;
    println!("tests/README.md generated successfully");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn weak_fields_keep_dimensionless_relative_error() {
        let expected = Vector3::new(1e-20, 0.0, 0.0);
        let (relative, absolute) = errors(expected * 1.001, expected);
        assert!((relative - 1e-3).abs() < 1e-15);
        assert!((absolute - 1e-23).abs() < 1e-35);
        assert_eq!(errors(Vector3::zeros(), Vector3::zeros()), (0.0, 0.0));
        assert!(errors(expected, Vector3::zeros()).0.is_infinite());
    }
}
