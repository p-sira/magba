/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion};
use magba::{
    fields::field_cylinder::local_cyl_B,
    sources::{CylinderMagnet, Field},
};
use nalgebra::{Point3, Translation3, UnitQuaternion, Vector3};
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
                let field = points
                    .iter()
                    .map(|p| local_cyl_B(p, radius, height, &pol).unwrap())
                    .collect::<Vec<_>>();
                assert!(!field.is_empty())
            });
        });

        group.bench_with_input(BenchmarkId::new("b_cyl_parallel", size), &size, |b, _| {
            b.iter(|| {
                let field = points
                    .par_iter()
                    .map(|p| local_cyl_B(p, radius, height, &pol).unwrap())
                    .collect::<Vec<_>>();
                assert!(!field.is_empty())
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
                let results = points
                    .iter()
                    .map(|point| translation.transform_point(&point))
                    .collect::<Vec<_>>();
                assert!(!results.is_empty())
            });
        });

        group.bench_with_input(
            BenchmarkId::new("translate_parallel", size),
            &size,
            |b, _| {
                b.iter(|| {
                    let results = points
                        .par_iter()
                        .map(|point| translation.transform_point(&point))
                        .collect::<Vec<_>>();
                    assert!(!results.is_empty())
                });
            },
        );
    }
    group.finish();
}

/// Serial is mostly faster, except at very large collection size
fn bench_rotate_parallel_vs_serial(c: &mut Criterion) {
    let mut group = c.benchmark_group("parallel_overhead");

    fn rotate_anchor(
        current_position: &Point3<f64>,
        current_orientation: &UnitQuaternion<f64>,
        rotation: &UnitQuaternion<f64>,
        anchor: &Point3<f64>,
    ) -> (Point3<f64>, UnitQuaternion<f64>) {
        let local_position = current_position - anchor;
        let new_position = Point3::from(rotation * local_position + Vector3::from(anchor.coords));
        let new_orientation = rotation * current_orientation;
        (new_position, new_orientation)
    }

    for size in [10, 100, 100000].iter() {
        let points = get_points(*size);
        let current_orientation = UnitQuaternion::identity();
        let rotation = UnitQuaternion::from_scaled_axis(Vector3::new(1.0, 2.0, 3.0));
        let anchor = Point3::new(3.0, 2.0, 1.0);

        group.bench_with_input(BenchmarkId::new("rotate_serial", size), &size, |b, _| {
            b.iter(|| {
                let results = points
                    .iter()
                    .map(|point| {
                        rotate_anchor(&point, &current_orientation, &rotation, &anchor);
                    })
                    .collect::<Vec<_>>();
                assert!(!results.is_empty())
            });
        });

        group.bench_with_input(BenchmarkId::new("rotate_parallel", size), &size, |b, _| {
            b.iter(|| {
                let results = points
                    .par_iter()
                    .map(|point| {
                        rotate_anchor(&point, &current_orientation, &rotation, &anchor);
                    })
                    .collect::<Vec<_>>();
                assert!(!results.is_empty())
            });
        });
    }
    group.finish();
}

fn get_cylinder_collection(n: usize) -> Vec<CylinderMagnet> {
    (0..n)
        .map(|_| {
            CylinderMagnet::new(
                Point3::new(-0.004694999999999998, 0.008131978541535878, -0.006),
                UnitQuaternion::from_scaled_axis(Vector3::new(
                    1.5315599088338596,
                    0.41038024073191587,
                    0.4103802407319159,
                )),
                Vector3::new(0.4, 0.5, 0.6),
                2e-3,
                5e-3,
            )
        })
        .collect()
}

/// Parallel is always faster, except with 1 source.
/// However, I don't think it is worth branching.
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
                    let field = collection
                        .iter()
                        .map(|magnet| magnet.get_B(&points))
                        .collect::<Vec<_>>();
                    assert!(!field.is_empty())
                });
            },
        );

        group.bench_with_input(
            BenchmarkId::new("collection_b_parallel", size),
            &size,
            |b, _| {
                b.iter(|| {
                    let field = collection
                        .par_iter()
                        .map(|magnet| magnet.get_B(&points))
                        .collect::<Vec<_>>();
                    assert!(!field.is_empty())
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
