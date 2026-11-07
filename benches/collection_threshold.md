# Source collection Rayon calibration

## Relative complexity

Measure each geometry's cost with outer collection execution forced serial,
while primitive execution remains automatic as in production:

```sh
cargo bench --locked --bench relative_complexity --features threshold-calibration,mesh
```

The benchmark covers 11 source geometries, 5,000 observers, 10 samples,
1-second warmup and 2-second measurement. Criterion writes medians under
`target/criterion/relative_complexity_outer_serial/`. Normalize to dipole and
round (minimum 1); path, mesh and sheet weights are per segment/triangle.

The existing weights below were calibrated in two passes on 2026-11-07 from
the perf PR (`d5e1d3b`), using a Ryzen 5 4600H, Rust 1.99.1, the bench
profile and locked dependencies (`ellip` 1.2.2). Mesh and sheet weights were
estimated from their corresponding triangle kernels, not independently fitted.

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

The search starts at 40,000 estimated work units, uses approximately 10,000-
unit steps and searches up to 160,000 units. Each candidate uses 10 Criterion
samples, 1-second warmup and 2-second measurement. All collection shapes are
retained, with primitive execution automatic. The resulting crossover is a
coarse estimate; confirm any selected production threshold with longer runs
around its boundary.

Benchmark results are saved to `benches/collection_threshold_results.md`.
The current **40,000 work-unit** threshold (strict `>`) was calibrated on
2026-10-05 with the older weights; it has **not** yet been refitted to the
new relative complexities. Singleton collections stay outer-serial.
