/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2026 Sira Pornsiriprasert <code@psira.me>
 */

use std::fs;
use std::path::{Path, PathBuf};

const OBSERVER_COUNT: usize = 5_000;

#[derive(Clone, Copy)]
struct FixedSource {
    display_name: &'static str,
    benchmark_name: &'static str,
    source_name: &'static str,
}

const FIXED_SOURCES: [FixedSource; 9] = [
    FixedSource {
        display_name: "Sphere",
        benchmark_name: "sphere_two",
        source_name: "SphereMagnet",
    },
    FixedSource {
        display_name: "Dipole",
        benchmark_name: "dipole_two",
        source_name: "Dipole",
    },
    FixedSource {
        display_name: "Circular current",
        benchmark_name: "circular_two",
        source_name: "CircularCurrent",
    },
    FixedSource {
        display_name: "Cylinder",
        benchmark_name: "cylinder_two",
        source_name: "CylinderMagnet",
    },
    FixedSource {
        display_name: "Triangle magnet",
        benchmark_name: "triangle-magnet_two",
        source_name: "TriangleMagnet",
    },
    FixedSource {
        display_name: "Triangle current",
        benchmark_name: "triangle-current_two",
        source_name: "TriangleCurrent",
    },
    FixedSource {
        display_name: "Cuboid",
        benchmark_name: "cuboid_two",
        source_name: "CuboidMagnet",
    },
    FixedSource {
        display_name: "Tetrahedron",
        benchmark_name: "tetrahedron_two",
        source_name: "TetrahedronMagnet",
    },
    FixedSource {
        display_name: "Path (2 segments)",
        benchmark_name: "path_2-segments",
        source_name: "PathCurrent",
    },
];

fn criterion_root() -> PathBuf {
    if Path::new("target/criterion").exists() || !Path::new("../target/criterion").exists() {
        PathBuf::from("target/criterion")
    } else {
        PathBuf::from("../target/criterion")
    }
}

fn output_path() -> PathBuf {
    if Path::new("benches").exists() || !Path::new("../benches").exists() {
        PathBuf::from("benches/relative_complexity.txt")
    } else {
        PathBuf::from("../benches/relative_complexity.txt")
    }
}

fn calibration_result(root: &Path, benchmark_name: &str) -> PathBuf {
    let serial_dir = root
        .join(format!("collection_{benchmark_name}"))
        .join("serial");
    let prefix = format!("points={OBSERVER_COUNT},score=");

    fs::read_dir(&serial_dir)
        .unwrap_or_else(|error| panic!("cannot read {}: {error}", serial_dir.display()))
        .filter_map(Result::ok)
        .filter(|entry| {
            entry.file_type().is_ok_and(|kind| kind.is_dir())
                && entry.file_name().to_string_lossy().starts_with(&prefix)
        })
        .filter_map(|entry| {
            let path = entry.path().join("new/estimates.json");
            let modified = path.metadata().ok()?.modified().ok()?;
            Some((path, modified))
        })
        .max_by_key(|(_, modified)| *modified)
        .map(|(path, _)| path)
        .unwrap_or_else(|| {
            panic!(
                "no {OBSERVER_COUNT}-observer sample found for {benchmark_name}; run the collection_threshold benchmark first"
            )
        })
}

fn serial_median_ns(root: &Path, benchmark_name: &str) -> f64 {
    let path = calibration_result(root, benchmark_name);
    let content = fs::read_to_string(&path)
        .unwrap_or_else(|error| panic!("cannot read {}: {error}", path.display()));
    let estimates: serde_json::Value = serde_json::from_str(&content)
        .unwrap_or_else(|error| panic!("cannot parse {}: {error}", path.display()));

    estimates["median"]["point_estimate"]
        .as_f64()
        .unwrap_or_else(|| panic!("missing median point estimate in {}", path.display()))
}

fn rounded_weight(relative_time: f64) -> usize {
    relative_time.round().max(1.0) as usize
}

fn generate_code(root: &Path) -> String {
    let dipole_time = serial_median_ns(root, "dipole_two");
    let mut measurements = Vec::new();

    for source in FIXED_SOURCES {
        let median = serial_median_ns(root, source.benchmark_name);
        let relative_time = median / dipole_time;
        let weight = if source.source_name == "PathCurrent" {
            (relative_time / 2.0).ceil().max(1.0) as usize
        } else {
            rounded_weight(relative_time)
        };
        measurements.push((source, median, weight));
    }

    let triangle_magnet_weight = measurements
        .iter()
        .find(|(source, _, _)| source.source_name == "TriangleMagnet")
        .map(|(_, _, weight)| *weight)
        .unwrap();
    let triangle_current_weight = measurements
        .iter()
        .find(|(source, _, _)| source.source_name == "TriangleCurrent")
        .map(|(_, _, weight)| *weight)
        .unwrap();
    let path_weight = measurements
        .iter()
        .find(|(source, _, _)| source.source_name == "PathCurrent")
        .map(|(_, _, weight)| *weight)
        .unwrap();

    let mut output = String::from(
        "// Generated from Criterion serial medians at 5,000 observers.\n\n\
         // Fixed-cost sources\n",
    );
    for (source, median, weight) in &measurements {
        if source.source_name != "PathCurrent" {
            output.push_str(&format!(
                "// {} ({:.1} ns; {})\nrelative_complexity: |_source| {};\n",
                source.source_name, median, source.display_name, weight
            ));
        }
    }

    output.push_str(&format!(
        "\n// Geometry-scaled sources\n\
         // PathCurrent\n\
         relative_complexity: |source| source.vertices.len().saturating_sub(1).saturating_mul({path_weight});\n\
         // MeshMagnet\n\
         relative_complexity: |source| source.mesh.triangles().len().saturating_mul({triangle_magnet_weight});\n\
         // SheetCurrent\n\
         relative_complexity: |source| source.mesh.triangles().len().saturating_mul({triangle_current_weight});\n"
    ));
    output
}

fn main() {
    let generated = generate_code(&criterion_root());
    print!("{generated}");

    let save_path = output_path();
    fs::write(&save_path, generated)
        .unwrap_or_else(|error| panic!("cannot write {}: {error}", save_path.display()));
    println!("// Saved to {}", save_path.display());
}

#[cfg(test)]
mod tests {
    use super::rounded_weight;

    #[test]
    fn weights_are_rounded_and_never_zero() {
        assert_eq!(rounded_weight(0.49), 1);
        assert_eq!(rounded_weight(2.51), 3);
    }
}
