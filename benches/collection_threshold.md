# Source collection Rayon calibration

## Relative complexity

Calibrated on 2026-11-07 from the perf PR (`d5e1d3b`), using an AMD Ryzen 5
4600H (6 cores / 12 threads), Rust 1.99.1, the bench profile, and locked
dependencies (`ellip` 1.2.2). Primitive execution stays automatic; only outer
collection execution is forced serial.

Run the nine representative two-source workloads individually:

```sh
for source in sphere/two dipole/two circular/two cylinder/two triangle-magnet/two triangle-current/two cuboid/two tetrahedron/two path/2-segments; do
  MAGBA_RELATIVE_COMPLEXITY=1 cargo bench --locked --bench collection_threshold --features threshold-calibration,mesh -- \
    "collection/$source/serial/points=5000" --sample-size 10 --warm-up-time 1 --measurement-time 2
done
```

Criterion writes medians to `target/criterion/collection_*/serial/points=5000,score=*/new/estimates.json`.
For fixed sources, round the median/dipole ratio (minimum 1); for paths,
divide by the two segments and round up. Mesh and sheet weights scale from
the corresponding triangle weights.

| Source | Median time (two passes) | Weight |
| --- | ---: | ---: |
| Sphere | 116.6 µs | 1 |
| Dipole | 137.8 µs | 1 |
| Path (2 segments) | 238.0 µs | 1 / segment |
| Circular current | 259.8 µs | 2 |
| Cylinder | 343.8 µs | 2 |
| Triangle magnet | 406.9 µs | 3 |
| Triangle current | 537.8 µs | 4 |
| Cuboid | 650.0 µs | 5 |
| Tetrahedron | 968.2 µs | 7 |
| Mesh | geometry-dependent | 3 / triangle |
| Sheet current | geometry-dependent | 4 / triangle |

These are deliberately coarse work estimates, not primitive Rayon thresholds.
The earlier weights were measured at commit `f6a9fb3` with its `Cargo.lock`
and Rust 1.98.1.

## Outer parallel threshold

`collection_threshold.rs` adaptively benchmarks forced outer-serial and
outer-parallel execution, then automatic execution at the selected size:

```sh
cargo bench --locked --bench collection_threshold --features threshold-calibration,mesh
```

Benchmark results are saved to `benches/collection_threshold_results.md`.
The current **40,000 work-unit** threshold (strict `>`) was calibrated on
2026-10-05 with the older weights; it has **not** yet been refitted to the
new relative complexities. Singleton collections stay outer-serial.
