/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use criterion::{BenchmarkId, Criterion, criterion_group, criterion_main};
use magba::fields::*;
use nalgebra::{Point3, UnitQuaternion, Vector3, point, vector};

#[cfg(feature = "rayon")]
use rayon::prelude::*;

fn get_points(n: usize) -> Vec<Point3<f64>> {
    (0..n)
        .map(|i| {
            let i = i as f64;
            point![i * 0.01, i * 0.01, i * 0.01]
        })
        .collect()
}

macro_rules! bench_field {
    ($c:ident, $group_name:expr, $func_name:ident, $input_sizes:expr, ($($args:expr),*)) => {
        let mut group = $c.benchmark_group($group_name);
        for size in $input_sizes {
            let points = get_points(*size);
            let mut out = vec![Vector3::zeros(); *size];

            group.bench_with_input(
                BenchmarkId::new("serial", size),
                size,
                |b, _| {
                    b.iter(|| {
                        points.iter().zip(out.iter_mut()).for_each(|(p, o)| {
                            *o = $func_name(*p, $($args),*);
                        });
                        criterion::black_box(&out);
                    });
                },
            );

            #[cfg(feature = "rayon")]
            group.bench_with_input(
                BenchmarkId::new("batch", size),
                size,
                |b, _| {
                    b.iter(|| {
                        points.par_iter()
                            .zip(out.par_iter_mut())
                            .for_each(|(p, o)| {
                                *o = $func_name(*p, $($args),*);
                            });
                        criterion::black_box(&out);
                    });
                },
            );
        }
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

    bench_field!(
        c,
        "threshold/circular_B",
        circular_B,
        &[150, 300, 500, 700, 1000],
        (pos, ori, diameter, current)
    );
    bench_field!(
        c,
        "threshold/cuboid_B",
        cuboid_B,
        &[20, 30, 50, 100, 150],
        (pos, ori, pol, dim)
    );
    bench_field!(
        c,
        "threshold/cylinder_B",
        cylinder_B,
        &[75, 100, 150, 200, 250],
        (pos, ori, pol, diameter, height)
    );
    bench_field!(
        c,
        "threshold/dipole_B",
        dipole_B,
        &[3000, 4000, 5000, 6000, 7000],
        (pos, ori, moment)
    );
    bench_field!(
        c,
        "threshold/sphere_B",
        sphere_B,
        &[1500, 2000, 2500, 3000, 3500],
        (pos, ori, pol, diameter)
    );
}

criterion_group!(benches, bench_thresholds);
criterion_main!(benches);
