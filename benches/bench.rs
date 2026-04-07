/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use std::io::Write;
use std::path::Path;

use criterion::{BenchmarkId, Criterion, criterion_group, criterion_main};
use magba::fields::*;
use nalgebra::{Point3, UnitQuaternion, Vector3, point, vector};
use tabled::{Table, Tabled, settings::Style};

#[cfg(feature = "rayon")]
use rayon::prelude::*;

const MAX_THRESHOLD: usize = 10000;
const ACCEPT_THRESHOLD: f64 = 0.1;
const STEP: usize = 50;
const MAX_ITER: usize = 15;

fn get_points(n: usize) -> Vec<Point3<f64>> {
    (0..n)
        .map(|i| {
            let i = i as f64;
            point![i * 0.01, i * 0.01, i * 0.01]
        })
        .collect()
}

enum ExitCond {
    None,
    Converged,
    MinStep,
    MaxIter,
    MaxThres,
}

impl ExitCond {
    fn to_string(&self) -> String {
        match self {
            Self::None => "-".to_string(),
            Self::Converged => "Converged".to_string(),
            Self::MinStep => "Min step size".to_string(),
            Self::MaxIter => "Max iteration".to_string(),
            Self::MaxThres => "Max threshold".to_string(),
        }
    }
}

fn format_float(val: &f64) -> String {
    if val.is_nan() {
        "-".to_string()
    } else {
        format!("{:.3}", val)
    }
}

fn format_performance(val: &f64) -> String {
    let val = *val / 1e9;
    if val.is_nan() {
        "-".to_string()
    } else if val < 1e-6 {
        format!("{:.3} ns", val * 1e9)
    } else if val < 1e-3 {
        format!("{:.3} µs", val * 1e6)
    } else if val < 1.0 {
        format!("{:.3} ms", val * 1e3)
    } else {
        format!("{:.3} s", val)
    }
}

#[derive(Tabled)]
struct Record {
    #[tabled(rename = "Function")]
    function_name: String,
    #[tabled(rename = "Threshold")]
    threshold: usize,
    #[tabled(rename = "Parallel Time", display = "format_performance")]
    par_time: f64,
    #[tabled(rename = "Serial Time", display = "format_performance")]
    ser_time: f64,
    #[tabled(rename = "Ratio", display = "format_float")]
    ratio: f64,
    #[tabled(rename = "Exit Condition", display = "ExitCond::to_string")]
    exit_cond: ExitCond,
}

fn save_results(path: &Path, results: &[Record]) {
    let result_str = Table::new(results).with(Style::markdown()).to_string();
    let mut file = std::fs::File::create(path).unwrap();
    file.write_all(result_str.as_bytes()).unwrap();
}

macro_rules! find_threshold {
    ($c:ident, $results:ident, $func_name:ident, ($($args:expr),*), $init_step:expr, $record_file:expr) => {
        let mut group = $c.benchmark_group(format!("threshold_{}", stringify!($func_name)));
        let root_path = Path::new("target/criterion");
        let func_str = stringify!($func_name);
        let par_path = root_path.join(format!("threshold_{}/batch", func_str));
        let ser_path = root_path.join(format!("threshold_{}/serial", func_str));

        let mut record = Record {
            function_name: func_str.to_string(),
            threshold: $init_step,
            par_time: f64::NAN,
            ser_time: f64::NAN,
            ratio: f64::NAN,
            exit_cond: ExitCond::None,
        };

        // --- 1. Expansion phase ---
        let mut low = STEP;
        let mut high = $init_step;

        loop {
            record.threshold = ((high + STEP / 2) / STEP) * STEP;
            record.threshold = record.threshold.max(STEP);

            let points = get_points(record.threshold);
            let mut out = vec![Vector3::zeros(); record.threshold];

            group.bench_with_input(BenchmarkId::new("serial", record.threshold), &record.threshold, |b, _| {
                b.iter(|| {
                    points.iter().zip(out.iter_mut()).for_each(|(p, o)| {
                        *o = $func_name(*p, $($args),*);
                    });
                    criterion::black_box(&out);
                });
            });

            group.bench_with_input(BenchmarkId::new("batch", record.threshold), &record.threshold, |b, _| {
                b.iter(|| {
                    points.par_iter().zip(out.par_iter_mut()).for_each(|(p, o)| {
                        *o = $func_name(*p, $($args),*);
                    });
                    criterion::black_box(&out);
                });
            });

            record.par_time = reproducible::extract_criterion_mean_ns(
                &par_path.join(record.threshold.to_string()).join("new").join("estimates.json")
            ).unwrap();
            record.ser_time = reproducible::extract_criterion_mean_ns(
                &ser_path.join(record.threshold.to_string()).join("new").join("estimates.json")
            ).unwrap();
            record.ratio = record.par_time / record.ser_time;

            if record.ratio < 1.0 {
                // Overshoot found
                // --- 2. Binary search phase ---
                let mut last_mid = record.threshold;

                for n in 0..MAX_ITER {
                    let mid = ((low + high) / 2 / STEP) * STEP;
                    if mid == last_mid {
                        record.exit_cond = ExitCond::MinStep;
                        break;
                    }

                    let points = get_points(mid);
                    let mut out = vec![Vector3::zeros(); mid];

                    group.bench_with_input(BenchmarkId::new("serial", mid), &mid, |b, _| {
                        b.iter(|| {
                            points.iter().zip(out.iter_mut()).for_each(|(p, o)| {
                                *o = $func_name(*p, $($args),*);
                            });
                            criterion::black_box(&out);
                        });
                    });

                    group.bench_with_input(BenchmarkId::new("batch", mid), &mid, |b, _| {
                        b.iter(|| {
                            points.par_iter().zip(out.par_iter_mut()).for_each(|(p, o)| {
                                *o = $func_name(*p, $($args),*);
                            });
                            criterion::black_box(&out);
                        });
                    });

                    let par_time = reproducible::extract_criterion_mean_ns(
                        &par_path.join(mid.to_string()).join("new").join("estimates.json")
                    ).unwrap();
                    let ser_time = reproducible::extract_criterion_mean_ns(
                        &ser_path.join(mid.to_string()).join("new").join("estimates.json")
                    ).unwrap();
                    let ratio = par_time / ser_time;

                    if ratio < 1.0 - ACCEPT_THRESHOLD {
                        high = mid;
                    } else if ratio > 1.0 + ACCEPT_THRESHOLD {
                        low = mid;
                    } else {
                        record.threshold = mid;
                        record.par_time = par_time;
                        record.ser_time = ser_time;
                        record.ratio = ratio;
                        record.exit_cond = ExitCond::Converged;
                        break;
                    }

                    if high - low <= STEP {
                        record.threshold = high;
                        record.par_time = par_time;
                        record.ser_time = ser_time;
                        record.ratio = ratio;
                        record.exit_cond = ExitCond::MinStep;
                        break;
                    }

                    if n == MAX_ITER - 1 {
                        record.threshold = high;
                        record.exit_cond = ExitCond::MaxIter;
                    }

                    last_mid = mid;
                }
                break;
            }

            low = high;
            high *= 2;
            if high > MAX_THRESHOLD {
                record.exit_cond = ExitCond::MaxThres;
                break;
            }
        }
        $results.push(record);
        save_results($record_file, &$results);
        group.finish();
    };
}

fn bench_thresholds(c: &mut Criterion) {
    let pos = Point3::origin();
    let ori = UnitQuaternion::identity();
    let pol = vector![1.0, 0.0, 0.0];
    let dim = vector![0.01, 0.01, 0.01];
    let height = 0.02;
    let diameter = 0.02;
    let moment = vector![0.0, 0.0, 1e-3];
    let current = 1.0;

    let mut results = Vec::new();
    let record_file = Path::new("benches/par_threshold.md");

    find_threshold!(
        c,
        results,
        circular_B,
        (pos, ori, diameter, current),
        500,
        record_file
    );
    find_threshold!(c, results, cuboid_B, (pos, ori, pol, dim), 50, record_file);
    find_threshold!(
        c,
        results,
        cylinder_B,
        (pos, ori, pol, diameter, height),
        150,
        record_file
    );
    find_threshold!(c, results, dipole_B, (pos, ori, moment), 5000, record_file);
    find_threshold!(
        c,
        results,
        sphere_B,
        (pos, ori, pol, diameter),
        2500,
        record_file
    );
}

criterion_group!(benches, bench_thresholds);
criterion_main!(benches);
