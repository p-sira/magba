/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use std::collections::HashSet;
use std::hint::black_box;
use std::time::Duration;

use criterion::{BenchmarkId, Criterion, criterion_group, criterion_main};

mod collection_workloads;
mod common;
use collection_workloads::{points, workloads};
use magba::{
    base::Source,
    threshold_calibration::{self, ExecutionMode},
};

const CALIBRATION_THRESHOLD: usize = 50_000;
const MAX_WORK: usize = 160_000;
const STEP_WORK: usize = 10_000;
const COMPLEXITY_CALIBRATION_OBSERVERS: usize = 5_000;

fn bench_collection_threshold(c: &mut Criterion) {
    threshold_calibration::set_instrumentation(false);
    threshold_calibration::set_execution_mode(ExecutionMode::Auto);

    let result_path = std::path::Path::new("benches/collection_threshold_results.md");
    let mut results = common::load_results(result_path);

    for (name, sources) in workloads() {
        let complexity = sources.relative_complexity();
        let group_name = format!("collection/{name}");
        // Criterion's on-disk path flattens slashes in benchmark group names.
        let criterion_group = format!("collection_{}", name.replace('/', "_"));
        let mut group = c.benchmark_group(&group_name);
        group.sample_size(10);
        group.warm_up_time(Duration::from_secs(1));
        group.measurement_time(Duration::from_secs(2));
        let mut seen = HashSet::new();

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

        let step = (STEP_WORK / complexity).max(common::MIN_STEP);
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
