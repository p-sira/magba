/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion};
use magba::field::local_cyl_B;
use nalgebra::{Point3, Vector3};
use rayon::prelude::*;

/// ```
/// use russell_lab::*;
///
/// println!("Using Intel MKL  = {}", using_intel_mkl());
/// println!("BLAS num threads = {}", get_num_threads());
/// set_num_threads(2);
/// println!("BLAS num threads = {}", get_num_threads());
/// ```
fn generate_local_cyl_b_test_data(size: usize) -> (Vec<Point3<f64>>, f64, f64, Vector3<f64>) {
    let test_points = [
        Point3::new(1.0, -1.0, 1.5),
        Point3::new(1.0, 1.0, 1.5),
        Point3::new(0.0, 0.0, 0.0),
    ];
    let points = (0..size)
        .map(|n| test_points[n % test_points.len()].clone())
        .collect();
    (points, 1.0, 2.0, Vector3::new(1.0, 1.0, 1.0))
}

/// Found that parallel is better after 20
fn bench_b_cyl_parallel_vs_serial(c: &mut Criterion) {
    let mut group = c.benchmark_group("parallel_overhead");

    for size in [10, 20, 50, 100, 1000].iter() {
        let (points, radius, height, pol) = generate_local_cyl_b_test_data(*size);

        group.bench_with_input(BenchmarkId::new("b_cyl_serial", size), &size, |b, _| {
            b.iter(|| {
                points
                    .iter()
                    .map(|p| local_cyl_B(p, radius, height, &pol).unwrap())
                    .collect::<Vec<_>>()
            });
        });

        group.bench_with_input(BenchmarkId::new("b_cyl_parallel", size), &size, |b, _| {
            b.iter(|| {
                points
                    .par_iter()
                    .map(|p| local_cyl_B(p, radius, height, &pol).unwrap())
                    .collect::<Vec<_>>()
            });
        });
    }
    group.finish();
}

criterion_group!(benches, bench_b_cyl_parallel_vs_serial,);
criterion_main!(benches);
