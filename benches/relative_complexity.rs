/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2026 Sira Pornsiriprasert <code@psira.me>
 */

use std::hint::black_box;
use std::time::Duration;

use criterion::{Criterion, criterion_group, criterion_main};
use magba::{
    base::Source,
    threshold_calibration::{self, ExecutionMode},
};

mod collection_workloads;
use collection_workloads::{points, workloads};

const OBSERVERS: usize = 5_000;
const SOURCES: [&str; 11] = [
    "sphere/two",
    "dipole/two",
    "circular/two",
    "cylinder/two",
    "triangle-magnet/two",
    "triangle-current/two",
    "cuboid/two",
    "tetrahedron/two",
    "path/2-segments",
    "mesh/4-triangles",
    "sheet/4-triangles",
];

fn relative_complexity(c: &mut Criterion) {
    threshold_calibration::set_instrumentation(false);
    threshold_calibration::set_execution_mode(ExecutionMode::Auto);
    threshold_calibration::set_collection_execution_mode(ExecutionMode::Serial);

    let observers = points(OBSERVERS);
    let mut group = c.benchmark_group("relative_complexity/outer_serial");
    group.sample_size(10);
    group.warm_up_time(Duration::from_secs(1));
    group.measurement_time(Duration::from_secs(2));

    for (name, source) in workloads() {
        if SOURCES.contains(&name) {
            group.bench_function(name, |b| {
                b.iter(|| black_box(source.compute_B_batch(&observers)))
            });
        }
    }
    group.finish();

    threshold_calibration::set_collection_execution_mode(ExecutionMode::Auto);
    threshold_calibration::set_execution_mode(ExecutionMode::Auto);
}

criterion_group!(benches, relative_complexity);
criterion_main!(benches);
