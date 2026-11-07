/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use std::path::{Path, PathBuf};

use criterion::{BenchmarkId, Criterion, criterion_group, criterion_main};

mod common;
use common::{Record, load_results, save_results, update_or_push_record};
use magba::fields::*;
use nalgebra::{Point3, UnitQuaternion, Vector3, point, vector};

#[cfg(feature = "rayon")]
use rayon::prelude::*;

const MAX_THRESHOLD: usize = 30000;

fn get_points(n: usize) -> Vec<Point3<f64>> {
    (0..n)
        .map(|i| {
            let i = i as f64;
            point![i * 0.01, i * 0.01, i * 0.01]
        })
        .collect()
}

fn get_record_file() -> PathBuf {
    if Path::new("benches/par_threshold.md").exists()
        || !Path::new("../benches/par_threshold.md").exists()
    {
        PathBuf::from("benches/par_threshold.md")
    } else {
        PathBuf::from("../benches/par_threshold.md")
    }
}

macro_rules! find_threshold {
    ($c:ident, $results:ident, $func_name:ident, ($($args:expr),*), $init_step:expr, $record_file:expr) => {
        {
            let func_str = stringify!($func_name);
            let group_name = format!("threshold_{func_str}");
            let mut group = $c.benchmark_group(&group_name);
            let mut seen = std::collections::HashSet::new();

            let result = common::find_threshold($init_step, MAX_THRESHOLD, common::MIN_STEP, |size| {
                if seen.insert(size) {
                    let points = get_points(size);
                    let mut out = vec![Vector3::zeros(); size];
                    group.bench_with_input(BenchmarkId::new("serial", size), &size, |b, _| {
                        b.iter(|| {
                            points.iter().zip(out.iter_mut()).for_each(|(p, o)| {
                                *o = $func_name(*p, $($args),*);
                            });
                            std::hint::black_box(&out);
                        });
                    });
                    group.bench_with_input(BenchmarkId::new("batch", size), &size, |b, _| {
                        b.iter(|| {
                            points.par_iter().zip(out.par_iter_mut()).for_each(|(p, o)| {
                                *o = $func_name(*p, $($args),*);
                            });
                            std::hint::black_box(&out);
                        });
                    });
                }
                let id = size.to_string();
                Some(common::Timing {
                    parallel_ns: common::criterion_mean_ns(&group_name, "batch", &id)?,
                    serial_ns: common::criterion_mean_ns(&group_name, "serial", &id)?,
                })
            });
            group.finish();

            if let Some(result) = result {
                update_or_push_record(&mut $results, Record {
                    function_name: func_str.to_string(),
                    threshold: result.threshold,
                    par_time: result.timing.parallel_ns,
                    ser_time: result.timing.serial_ns,
                    ratio: result.timing.ratio(),
                    exit_cond: result.exit,
                });
                save_results($record_file, &$results);
            }
        }
    };
}

macro_rules! find_thresholds {
    ($c:ident, $results:ident, $record_file:expr => { $(( $func:ident, $n_args:tt $(: $test_file_name:expr)?, $init_step:expr )),* $(,)? }) => {
        $(
            find_threshold!($c, $results, $func, $n_args $(: $test_file_name)?, $init_step, &$record_file);
        )*
    };
}

fn bench_thresholds(c: &mut Criterion) {
    let pos = Point3::origin();
    let ori = UnitQuaternion::identity();
    let pol = vector![1.0, -1.0, 1.0];
    let dim = vector![0.01, 0.01, 0.01];
    let height = 0.02;
    let diameter = 0.02;
    let moment = vector![1e-3, -1e-3, 1e-3];
    let current = 1.0;

    let vertices_triangle = [
        vector![-0.1, -0.1, -0.1],
        vector![0.1, -0.1, 0.1],
        vector![0.0, 0.2, 0.0],
    ];

    let vertices_tetra = [
        vector![-0.1, -0.1, -0.1],
        vector![0.1, -0.1, -0.1],
        vector![0.0, 0.1, -0.1],
        vector![0.0, 0.0, 0.1],
    ];
    let vertices_path_current = vec![
        vector![-0.1, -0.1, -0.1],
        vector![0.1, -0.1, -0.1],
        vector![0.0, 0.1, -0.1],
        vector![0.0, 0.0, 0.1],
    ];
    let (vertices_tetra, mat_inv_tetra) = magba::fields::precompute_tetrahedron(vertices_tetra);

    let base_path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("testing/data");
    let stl_path = base_path.join("suzanne.stl"); // About 700 triangles
    let mut file = std::fs::File::open(stl_path).expect("Failed to open mesh.stl");
    let openmesh = openmesh::Mesh::from_stl(&mut file).expect("Failed to read STL");

    let mesh = magba::base::mesh::TriMesh::from(openmesh);

    let vertices_triangle_current = [
        vector![0.0, 0.0, 0.0],
        vector![1.0, 0.0, 0.0],
        vector![0.0, 1.0, 0.0],
    ];
    let current_density = vector![1.0, 2.0, 3.0];
    // The sheet-current API requires one density per face. Keep only four
    // active triangles to match the sparse-current threshold workload.
    let mut sheet_current_densities = vec![Vector3::zeros(); mesh.triangles().len()];
    for density in sheet_current_densities.iter_mut().take(4) {
        *density = current_density;
    }

    let record_file = get_record_file();
    let mut results = load_results(&record_file);

    find_thresholds!( c, results, record_file => {
        (circular_B, (pos, ori, diameter, current), 370),
        (path_current_B, (pos, ori, current, &vertices_path_current), 250),
        (cuboid_B, (pos, ori, pol, dim), 55),
        (cylinder_B, (pos, ori, pol, diameter, height), 100),
        (dipole_B, (pos, ori, moment), 6250),
        (sphere_B, (pos, ori, pol, diameter), 9060),
        (triangle_B, (pos, ori, pol, vertices_triangle), 310),
        (tetrahedron_B_precomputed, (pos, ori, pol, vertices_tetra, mat_inv_tetra), 50),
        (triangle_current_B, (pos, ori, current_density, vertices_triangle_current), 200),
        (mesh_B, (pos, ori, pol, &mesh), 10),
        (sheet_current_B, (pos, ori, &sheet_current_densities, &mesh), 45),
    });
}

criterion_group!(benches, bench_thresholds);
criterion_main!(benches);
