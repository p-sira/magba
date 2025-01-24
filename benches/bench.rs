/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion};
use magba::{
    fields::field_cylinder::local_cyl_B,
    sources::{CylinderMagnet, Field},
};
use nalgebra::{Matrix3, Point3, Translation3, UnitQuaternion, Vector3};
use rand::prelude::*;
use rayon::prelude::*;

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

fn get_points(n: usize) -> Vec<Point3<f64>> {
    (0..n)
        .map(|i| {
            let i = i as f64;
            Point3::new(i, i + 1.0, i + 2.0)
        })
        .collect()
}

/// Serial is always faster
fn bench_translate_parallel_vs_serial(c: &mut Criterion) {
    let mut group = c.benchmark_group("parallel_overhead");

    for size in [10, 100, 100000].iter() {
        let points = get_points(*size);
        let translation = Translation3::new(3.0, 2.0, 1.0);

        group.bench_with_input(BenchmarkId::new("translate_serial", size), &size, |b, _| {
            b.iter(|| {
                points.iter().for_each(|point| {
                    let _ = &translation.transform_point(&point);
                });
            });
        });

        group.bench_with_input(
            BenchmarkId::new("translate_parallel", size),
            &size,
            |b, _| {
                b.iter(|| {
                    points.par_iter().for_each(|point| {
                        let _ = &translation.transform_point(&point);
                    });
                });
            },
        );
    }
    group.finish();
}

/// Serial is always faster
fn bench_rotate_parallel_vs_serial(c: &mut Criterion) {
    let mut group = c.benchmark_group("parallel_overhead");

    fn rotate_anchor(
        current_position: &Point3<f64>,
        current_orientation: &UnitQuaternion<f64>,
        rotation: &UnitQuaternion<f64>,
        anchor: &Point3<f64>,
    ) {
        let local_position = current_position - anchor;
        let _ = Point3::from(rotation * local_position + Vector3::from(anchor.coords));
        let _ = rotation * current_orientation;
    }

    for size in [10, 100, 100000].iter() {
        let points = get_points(*size);
        let current_orientation = UnitQuaternion::identity();
        let rotation = UnitQuaternion::from_scaled_axis(Vector3::new(1.0, 2.0, 3.0));
        let anchor = Point3::new(3.0, 2.0, 1.0);

        group.bench_with_input(BenchmarkId::new("rotate_serial", size), &size, |b, _| {
            b.iter(|| {
                points.iter().for_each(|point| {
                    rotate_anchor(&point, &current_orientation, &rotation, &anchor);
                });
            });
        });

        group.bench_with_input(BenchmarkId::new("rotate_parallel", size), &size, |b, _| {
            b.iter(|| {
                points.par_iter().for_each(|point| {
                    rotate_anchor(&point, &current_orientation, &rotation, &anchor);
                });
            });
        });
    }
    group.finish();
}

fn get_cylinder_collection(n: usize) -> Vec<CylinderMagnet> {
    let mut rng = rand::thread_rng();
    (0..n)
        .map(|_| {
            CylinderMagnet::new(
                Point3::new(rng.gen(), rng.gen(), rng.gen()),
                UnitQuaternion::from_matrix(&Matrix3::new_random()),
                Vector3::new_random(),
                rng.gen(),
                rng.gen(),
            )
        })
        .collect()
}

/// Parallel is always faster
fn bench_collection_b_parallel_vs_serial(c: &mut Criterion) {
    let mut group = c.benchmark_group("parallel_overhead");

    for size in [1, 2, 5, 10].iter() {
        let points = get_points(100);
        let collection = get_cylinder_collection(*size);

        group.bench_with_input(
            BenchmarkId::new("collection_b_serial", size),
            &size,
            |b, _| {
                b.iter(|| {
                    let results: Vec<_> = collection
                        .iter()
                        .map(|magnet| magnet.get_B(&points))
                        .collect();
                    black_box(results);
                });
            },
        );

        group.bench_with_input(
            BenchmarkId::new("collection_b_parallel", size),
            &size,
            |b, _| {
                b.iter(|| {
                    let results: Vec<_> = collection
                        .par_iter()
                        .map(|magnet| magnet.get_B(&points))
                        .collect();
                    black_box(results);
                });
            },
        );
    }
    group.finish();
}

criterion_group!(
    benches,
    bench_b_cyl_parallel_vs_serial,
    bench_translate_parallel_vs_serial,
    bench_rotate_parallel_vs_serial,
    bench_collection_b_parallel_vs_serial,
);
criterion_main!(benches);
