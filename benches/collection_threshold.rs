/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use std::collections::HashSet;
use std::hint::black_box;

use criterion::{BenchmarkId, Criterion, criterion_group, criterion_main};

mod common;
use magba::{
    base::{Source, mesh::TriMesh},
    collections::{SourceAssembly, SourceComponent},
    currents::{CircularCurrent, PathCurrent, SheetCurrent, TriangleCurrent},
    magnets::{
        CuboidMagnet, CylinderMagnet, Dipole, MeshMagnet, SphereMagnet, TetrahedronMagnet,
        TriangleMagnet,
    },
    threshold_calibration::{self, ExecutionMode},
};
use nalgebra::{Point3, UnitQuaternion, point, vector};

const CALIBRATION_THRESHOLD: usize = 40_000;
const MAX_WORK: usize = 160_000;
const COMPLEXITY_CALIBRATION_OBSERVERS: usize = 5_000;
const RELATIVE_COMPLEXITY_CASES: [&str; 9] = [
    "sphere/two",
    "dipole/two",
    "circular/two",
    "cylinder/two",
    "triangle-magnet/two",
    "triangle-current/two",
    "cuboid/two",
    "tetrahedron/two",
    "path/2-segments",
];

fn points(count: usize) -> Vec<Point3<f64>> {
    (0..count)
        .map(|i| {
            let x = i as f64 * 1e-4 + 0.2;
            point![x, x * 0.7, x * 1.3]
        })
        .collect()
}

fn repeated<S>(source: S, count: usize) -> SourceAssembly
where
    S: Clone + Into<SourceComponent>,
{
    (0..count).map(|_| source.clone().into()).collect()
}

fn tetra_mesh(repetitions: usize) -> TriMesh<f64> {
    let vertices = vec![
        vector![-0.1, -0.1, -0.1],
        vector![0.1, -0.1, -0.1],
        vector![0.0, 0.1, -0.1],
        vector![0.0, 0.0, 0.1],
    ];
    let tetra_faces = [[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2]];
    let faces = (0..repetitions)
        .flat_map(|_| tetra_faces)
        .collect::<Vec<_>>();
    TriMesh::new_unchecked(vertices, faces)
}

fn sheet_current(mesh: TriMesh<f64>) -> SheetCurrent<f64> {
    let densities = vec![vector![1.0, 0.0, 0.0]; mesh.triangles().len()];
    SheetCurrent::new([0.0; 3], UnitQuaternion::identity(), densities, mesh)
}

fn workloads() -> Vec<(&'static str, SourceAssembly)> {
    let triangle_vertices = [
        vector![-0.1, -0.1, -0.1],
        vector![0.1, -0.1, 0.1],
        vector![0.0, 0.2, 0.0],
    ];
    let tetra_vertices = [
        vector![-0.1, -0.1, -0.1],
        vector![0.1, -0.1, -0.1],
        vector![0.0, 0.1, -0.1],
        vector![0.0, 0.0, 0.1],
    ];
    let short_path = PathCurrent::default().with_current(1.0).with_vertices(vec![
        vector![-0.1, 0.0, 0.0],
        vector![0.0, 0.1, 0.0],
        vector![0.1, 0.0, 0.0],
    ]);
    let long_path = PathCurrent::default().with_current(1.0).with_vertices(
        (0..17)
            .map(|i| vector![i as f64 * 0.01, (i % 3) as f64 * 0.01, 0.0])
            .collect(),
    );
    let mesh4 = tetra_mesh(1);
    let mesh32 = tetra_mesh(8);
    let nested_child = repeated(Dipole::<f64>::default(), 4);
    let nested = SourceAssembly::from([
        SourceComponent::from(nested_child.clone()),
        SourceComponent::from(nested_child),
    ]);
    let mixed = SourceAssembly::from(vec![
        SourceComponent::from(Dipole::<f64>::default()),
        SourceComponent::from(CylinderMagnet::<f64>::default()),
        SourceComponent::from(long_path.clone()),
        SourceComponent::from(MeshMagnet::default().with_mesh(mesh32.clone())),
    ]);
    vec![
        ("empty", SourceAssembly::default()),
        ("dipole/one", repeated(Dipole::<f64>::default(), 1)),
        ("dipole/two", repeated(Dipole::<f64>::default(), 2)),
        ("dipole/many", repeated(Dipole::<f64>::default(), 16)),
        (
            "circular/two",
            repeated(CircularCurrent::<f64>::default().with_current(1.0), 2),
        ),
        (
            "triangle-current/two",
            repeated(
                TriangleCurrent::<f64>::default().with_current_density(vector![1.0, 2.0, 3.0]),
                2,
            ),
        ),
        ("cuboid/two", repeated(CuboidMagnet::<f64>::default(), 2)),
        (
            "cylinder/two",
            repeated(CylinderMagnet::<f64>::default(), 2),
        ),
        ("sphere/two", repeated(SphereMagnet::<f64>::default(), 2)),
        (
            "triangle-magnet/two",
            repeated(
                TriangleMagnet::default().with_vertices(triangle_vertices),
                2,
            ),
        ),
        (
            "tetrahedron/two",
            repeated(
                TetrahedronMagnet::default().with_vertices(tetra_vertices),
                2,
            ),
        ),
        ("path/2-segments", repeated(short_path, 2)),
        ("path/16-segments", repeated(long_path, 2)),
        (
            "mesh/4-triangles",
            repeated(MeshMagnet::default().with_mesh(mesh4.clone()), 2),
        ),
        (
            "mesh/32-triangles",
            repeated(MeshMagnet::default().with_mesh(mesh32.clone()), 2),
        ),
        ("sheet/4-triangles", repeated(sheet_current(mesh4), 2)),
        ("sheet/32-triangles", repeated(sheet_current(mesh32), 2)),
        ("mixed", mixed),
        ("nested", nested),
    ]
}

fn bench_collection_threshold(c: &mut Criterion) {
    let relative_only = std::env::var("MAGBA_RELATIVE_COMPLEXITY").as_deref() == Ok("1");
    threshold_calibration::set_instrumentation(false);
    threshold_calibration::set_execution_mode(ExecutionMode::Auto);

    let result_path = std::path::Path::new("benches/collection_threshold_results.md");
    let mut results = common::load_results(result_path);

    for (name, sources) in workloads() {
        if relative_only && !RELATIVE_COMPLEXITY_CASES.contains(&name) {
            continue;
        }
        let complexity = sources.relative_complexity();
        let group_name = format!("collection/{name}");
        // Criterion's on-disk path flattens slashes in benchmark group names.
        let criterion_group = format!("collection_{}", name.replace('/', "_"));
        let mut group = c.benchmark_group(&group_name);
        let mut seen = HashSet::new();

        if relative_only {
            let observer_count = COMPLEXITY_CALIBRATION_OBSERVERS;
            let observers = points(observer_count);
            threshold_calibration::set_collection_execution_mode(ExecutionMode::Serial);
            group.bench_with_input(
                BenchmarkId::new(
                    "serial",
                    format!(
                        "points={observer_count},score={}",
                        observer_count.saturating_mul(complexity)
                    ),
                ),
                &observer_count,
                |b, _| b.iter(|| black_box(sources.compute_B_batch(&observers))),
            );
            group.finish();
            continue;
        }

        // An empty or singleton assembly cannot use outer parallelism.
        if sources.components().take(2).count() < 2 || complexity == 0 {
            let observer_count = if complexity == 0 {
                100
            } else {
                COMPLEXITY_CALIBRATION_OBSERVERS
            };
            threshold_calibration::set_collection_execution_mode(ExecutionMode::Auto);
            let observers = points(observer_count);
            group.bench_with_input(
                BenchmarkId::new(
                    "auto",
                    format!(
                        "points={observer_count},score={}",
                        observer_count.saturating_mul(complexity)
                    ),
                ),
                &observer_count,
                |b, _| b.iter(|| black_box(sources.compute_B_batch(&observers))),
            );
            group.finish();
            continue;
        }

        let step = (CALIBRATION_THRESHOLD / complexity / 20).clamp(common::MIN_STEP, 1000);
        let initial = CALIBRATION_THRESHOLD.div_ceil(complexity).max(step);
        let maximum = MAX_WORK.div_ceil(complexity).max(initial);
        let found = common::find_threshold(initial, maximum, step, |observer_count| {
            let score = observer_count.saturating_mul(complexity);
            let id = format!("points={observer_count},score={score}");
            if seen.insert(observer_count) {
                let observers = points(observer_count);
                for (mode_name, mode) in [
                    ("serial", ExecutionMode::Serial),
                    ("parallel", ExecutionMode::Parallel),
                ] {
                    threshold_calibration::set_collection_execution_mode(mode);
                    group.bench_with_input(
                        BenchmarkId::new(mode_name, &id),
                        &observer_count,
                        |b, _| b.iter(|| black_box(sources.compute_B_batch(&observers))),
                    );
                }
            }
            Some(common::Timing {
                serial_ns: common::criterion_mean_ns(&criterion_group, "serial", &id)?,
                parallel_ns: common::criterion_mean_ns(&criterion_group, "parallel", &id)?,
            })
        });

        if let Some(found) = found {
            let observer_count = found.threshold;
            let score = observer_count.saturating_mul(complexity);
            threshold_calibration::set_collection_execution_mode(ExecutionMode::Auto);
            let observers = points(observer_count);
            group.bench_with_input(
                BenchmarkId::new("auto", format!("points={observer_count},score={score}")),
                &observer_count,
                |b, _| b.iter(|| black_box(sources.compute_B_batch(&observers))),
            );
            common::update_or_push_record(
                &mut results,
                common::Record {
                    function_name: name.to_string(),
                    threshold: score,
                    par_time: found.timing.parallel_ns,
                    ser_time: found.timing.serial_ns,
                    ratio: found.timing.ratio(),
                    exit_cond: found.exit,
                },
            );
            common::save_results(result_path, &results);
        }
        group.finish();
    }

    threshold_calibration::set_collection_execution_mode(ExecutionMode::Auto);
}

criterion_group!(benches, bench_collection_threshold);
criterion_main!(benches);
