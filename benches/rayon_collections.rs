/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Temporary multi-threaded Rayon benchmarks for source collections.
//!
//! Only uses public API available both before and after the relative
//! complexity dispatch change so the two revisions can be compared directly.

use std::hint::black_box;

use criterion::{BenchmarkId, Criterion, criterion_group, criterion_main};
use magba::{
    base::{Source, mesh::TriMesh},
    collections::{SourceAssembly, SourceComponent},
    currents::{CircularCurrent, PathCurrent},
    magnets::{CuboidMagnet, CylinderMagnet, Dipole, MeshMagnet, SphereMagnet},
};
use nalgebra::{Point3, point, vector};

const THREADS: usize = 4;

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

fn workloads() -> Vec<(&'static str, SourceAssembly, Vec<usize>)> {
    let long_path = PathCurrent::default().with_current(1.0).with_vertices(
        (0..17)
            .map(|i| vector![i as f64 * 0.01, (i % 3) as f64 * 0.01, 0.0])
            .collect(),
    );
    let nested_child = repeated(Dipole::<f64>::default(), 4);
    let nested = SourceAssembly::from([
        SourceComponent::from(nested_child.clone()),
        SourceComponent::from(nested_child),
    ]);
    let mixed = SourceAssembly::from(vec![
        SourceComponent::from(Dipole::<f64>::default()),
        SourceComponent::from(CylinderMagnet::<f64>::default()),
        SourceComponent::from(long_path.clone()),
        SourceComponent::from(MeshMagnet::default().with_mesh(tetra_mesh(8))),
    ]);

    vec![
        ("dipole/one", repeated(Dipole::<f64>::default(), 1), vec![100, 10_000]),
        ("dipole/two", repeated(Dipole::<f64>::default(), 2), vec![100, 1_000, 10_000, 100_000]),
        ("dipole/many", repeated(Dipole::<f64>::default(), 16), vec![100, 1_000, 10_000]),
        ("sphere/two", repeated(SphereMagnet::<f64>::default(), 2), vec![1_000, 30_000]),
        (
            "circular/two",
            repeated(CircularCurrent::<f64>::default().with_current(1.0), 2),
            vec![1_000, 10_000],
        ),
        ("cuboid/two", repeated(CuboidMagnet::<f64>::default(), 2), vec![1_000, 10_000]),
        ("path/16-segments", repeated(long_path, 2), vec![100, 1_000, 10_000]),
        ("mixed", mixed, vec![100, 1_000, 10_000]),
        ("nested", nested, vec![100, 1_000, 10_000]),
    ]
}

fn bench_rayon_collections(c: &mut Criterion) {
    let pool = rayon::ThreadPoolBuilder::new()
        .num_threads(THREADS)
        .build()
        .unwrap();

    for (name, sources, observer_counts) in workloads() {
        let mut group = c.benchmark_group(format!("rayon-collection/{name}"));
        for observer_count in observer_counts {
            let observers = points(observer_count);
            group.bench_with_input(
                BenchmarkId::new(format!("threads={THREADS}"), format!("points={observer_count}")),
                &observer_count,
                |b, _| {
                    pool.install(|| b.iter(|| black_box(sources.compute_B_batch(&observers))))
                },
            );
        }
        group.finish();
    }
}

criterion_group!(benches, bench_rayon_collections);
criterion_main!(benches);
