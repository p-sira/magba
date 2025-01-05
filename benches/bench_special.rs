/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use criterion::{black_box, criterion_group, criterion_main, Criterion};
use magba::special::cel::*;

fn benchmark_cel(crit: &mut Criterion) {
    let kc = black_box(5.43);
    let p = black_box(1.0);
    let c = black_box(1.0);
    let s = black_box(-1.0);

    crit.bench_function("cel", |b| {
        b.iter(|| cel(kc, p, c, s))
    });
}

criterion_group!(benches, benchmark_cel);
criterion_main!(benches);