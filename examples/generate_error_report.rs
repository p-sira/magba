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
use std::process::Command;
use tabled::{builder::Builder, settings::Style};

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
    error_scale: f64,
    criterion_root: &Path,
) -> Result<Vec<String>, Box<dyn Error>> {
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
    for (i, (got, expected)) in actual.iter().zip(reference).enumerate() {
        let got = got.map(|v| v.to_f64().unwrap());
        if !got.iter().all(|v| v.is_finite()) {
            return Err(format!("{name}: nonfinite field at observer {:?}", points[i]).into());
        }
        relative.push(errors(got, *expected).0 * error_scale);
    }
    relative.sort_by(f64::total_cmp);
    // Match the original report's averaged median and interpolated P95.
    let median = percentile(&relative, 0.5);
    let mean = relative
        .iter()
        .map(|value| value / relative.len() as f64)
        .sum();
    let p95 = percentile(&relative, 0.95);
    let max = relative[relative.len() - 1];
    let performance = criterion_performance(criterion_root, name, std::any::type_name::<T>())?;
    Ok(vec![
        name.to_owned(),
        format_error(median),
        format_error(mean),
        format_error(p95),
        format_error(max),
        performance,
    ])
}

fn percentile(sorted: &[f64], fraction: f64) -> f64 {
    let position = (sorted.len() - 1) as f64 * fraction;
    let index = position.floor() as usize;
    let weight = position - index as f64;
    if weight == 0.0 || sorted[index].is_infinite() {
        sorted[index]
    } else {
        sorted[index] * (1.0 - weight) + sorted[index + 1] * weight
    }
}

fn format_error(value: f64) -> String {
    if value != 0.0 && !(1e-3..1e4).contains(&value.abs()) {
        format!("{value:.3e}")
    } else {
        format!("{value:.3}")
    }
}

fn criterion_performance(
    root: &Path,
    name: &str,
    precision: &str,
) -> Result<String, Box<dyn Error>> {
    let directory = root
        .join(format!("fields_{precision}"))
        .join(name)
        .join("new");
    let path = directory.join("estimates.json");
    let file = match std::fs::File::open(&path) {
        Ok(file) => file,
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => return Ok("—".to_owned()),
        Err(error) => return Err(error.into()),
    };
    let estimates: serde_json::Value = serde_json::from_reader(file)?;
    let benchmark: serde_json::Value =
        serde_json::from_reader(std::fs::File::open(directory.join("benchmark.json"))?)?;
    let mean = estimates["mean"]["point_estimate"]
        .as_f64()
        .filter(|value| value.is_finite() && *value >= 0.0)
        .ok_or_else(|| format!("{}: missing or invalid mean time", path.display()))?;
    let count = benchmark["throughput"]["Elements"]
        .as_u64()
        .filter(|count| *count > 0)
        .ok_or_else(|| format!("{}: missing observer throughput", directory.display()))?;
    let ns = mean / count as f64;
    let (value, unit) = if ns < 1e3 {
        (ns, "ns")
    } else if ns < 1e6 {
        (ns / 1e3, "µs")
    } else if ns < 1e9 {
        (ns / 1e6, "ms")
    } else {
        (ns / 1e9, "s")
    };
    Ok(format!("{value:.1} {unit}"))
}

fn environment_context() -> Result<String, Box<dyn Error>> {
    // reproducible used sysinfo for hardware and rustc -vV for compiler details.
    let mut system = sysinfo::System::new();
    system.refresh_cpu_frequency();
    system.refresh_memory_specifics(sysinfo::MemoryRefreshKind::nothing().with_ram());
    let cpu = system.cpus().first();
    let compiler = Command::new(std::env::var_os("RUSTC").unwrap_or_else(|| "rustc".into()))
        .arg("-vV")
        .output()?;
    if !compiler.status.success() {
        return Err(format!(
            "rustc -vV failed: {}",
            String::from_utf8_lossy(&compiler.stderr)
        )
        .into());
    }
    format_environment(
        cpu.map_or("Unknown CPU", |cpu| cpu.brand()),
        cpu.map_or(0, |cpu| cpu.frequency()),
        system.total_memory(),
        &String::from_utf8(compiler.stdout)?,
    )
}

fn format_environment(
    cpu: &str,
    frequency_mhz: u64,
    memory_bytes: u64,
    compiler: &str,
) -> Result<String, Box<dyn Error>> {
    let field = |prefix: &str| {
        compiler
            .lines()
            .find_map(|line| line.strip_prefix(prefix))
            .ok_or_else(|| format!("rustc -vV output is missing {prefix}"))
    };
    let version = field("release: ")?;
    let host = field("host: ")?;
    let clock = if frequency_mhz == 0 {
        String::new()
    } else if frequency_mhz > 1000 {
        format!(" @{:.1} GHz", frequency_mhz as f64 / 1000.0)
    } else {
        format!(" @{frequency_mhz} MHz")
    };
    Ok(format!(
        "{cpu}{clock} RAM {} GB running {host} rustc {version} using {} v{}",
        memory_bytes / 1_000_000_000,
        env!("CARGO_PKG_NAME"),
        env!("CARGO_PKG_VERSION")
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
    let criterion_root = [
        root.join("target/criterion"),
        root.join("../target/criterion"),
    ]
    .into_iter()
    .find(|path| path.is_dir())
    .unwrap_or_else(|| root.join("target/criterion"));
    let mut report64 = Builder::default();
    report64.push_record(["Function", "Median", "Mean", "P95", "Max", "Performance"]);
    let mut report32 = Builder::default();
    report32.push_record([
        "Function",
        "Median (ε)",
        "Mean (ε)",
        "P95 (ε)",
        "Max (ε)",
        "Performance",
    ]);
    for (name, file) in sources {
        let reference = read_vectors(&data.join(file))?;
        report64.push_record(accuracy_row::<f64>(
            name,
            &points,
            &reference,
            1.0,
            &criterion_root,
        )?);
        report32.push_record(accuracy_row::<f32>(
            name,
            &points,
            &reference,
            1.0 / f32::EPSILON as f64,
            &criterion_root,
        )?);
    }
    let report64 = report64.build().with(Style::markdown()).to_string();
    let report32 = report32.build().with(Style::markdown()).to_string();
    let content = std::fs::read_to_string(root.join("tests/report_template.md"))?
        .replace("{{TEST_ENV}}", &environment_context()?)
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
    fn environment_matches_the_original_report_format() {
        let compiler = "rustc 1.98.1\nhost: x86_64-unknown-linux-gnu\nrelease: 1.98.1\n";
        let context = format_environment(
            "AMD Ryzen 5 4600H with Radeon Graphics",
            4000,
            16_000_000_000,
            compiler,
        )
        .unwrap();
        assert_eq!(
            context,
            format!(
                "AMD Ryzen 5 4600H with Radeon Graphics @4.0 GHz RAM 16 GB running x86_64-unknown-linux-gnu rustc 1.98.1 using magba v{}",
                env!("CARGO_PKG_VERSION")
            )
        );
    }

    #[test]
    fn report_percentiles_preserve_interpolation_and_infinite_errors() {
        let values = [0.0, 10.0, 20.0, 30.0];
        assert_eq!(percentile(&values, 0.5), 15.0);
        assert!((percentile(&values, 0.95) - 28.5).abs() < 1e-14);
        assert!(percentile(&[1.0, f64::INFINITY], 0.95).is_infinite());
        assert_eq!(percentile(&[0.0], 0.95), 0.0);
    }

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
