/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use std::collections::BTreeSet;
use std::hint::black_box;

use criterion::{BenchmarkId, Criterion, criterion_group, criterion_main};
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
use nalgebra::{Point3, point, vector};

const CALIBRATION_THRESHOLD: usize = 40_000;

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
        (
            "sheet/4-triangles",
            repeated(
                SheetCurrent::default()
                    .with_current_densities(vec![vector![1.0, 0.0, 0.0]; 4])
                    .with_mesh(mesh4),
                2,
            ),
        ),
        (
            "sheet/32-triangles",
            repeated(
                SheetCurrent::default()
                    .with_current_densities(vec![vector![1.0, 0.0, 0.0]; 32])
                    .with_mesh(mesh32),
                2,
            ),
        ),
        ("mixed", mixed),
        ("nested", nested),
    ]
}

fn bench_collection_threshold(c: &mut Criterion) {
    threshold_calibration::set_instrumentation(false);
    threshold_calibration::set_execution_mode(ExecutionMode::Auto);

    for (name, sources) in workloads() {
        let complexity = sources.relative_complexity();
        let observer_counts = if complexity == 0 {
            BTreeSet::from([0, 1, 100])
        } else {
            [
                CALIBRATION_THRESHOLD.saturating_sub(1),
                CALIBRATION_THRESHOLD,
                CALIBRATION_THRESHOLD + 1,
                CALIBRATION_THRESHOLD * 2,
            ]
            .map(|work| work.div_ceil(complexity).max(1))
            .into_iter()
            .collect()
        };

        let mut group = c.benchmark_group(format!("collection/{name}"));
        for observer_count in observer_counts {
            let observers = points(observer_count);
            let score = observer_count.saturating_mul(complexity);
            for (mode_name, mode) in [
                ("serial", ExecutionMode::Serial),
                ("parallel", ExecutionMode::Parallel),
                ("auto", ExecutionMode::Auto),
            ] {
                threshold_calibration::set_collection_execution_mode(mode);
                group.bench_with_input(
                    BenchmarkId::new(mode_name, format!("points={observer_count},score={score}")),
                    &observer_count,
                    |b, _| b.iter(|| black_box(sources.compute_B_batch(&observers))),
                );
            }
        }
        group.finish();
    }

    threshold_calibration::set_collection_execution_mode(ExecutionMode::Auto);
}

criterion_group!(benches, bench_collection_threshold);
criterion_main!(benches);
