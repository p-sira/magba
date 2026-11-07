# Source collection Rayon calibration

See [performance calibration](perf_calibration/README.md) for the workflow.

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

The search starts at 50,000 estimated work units, uses approximately 10,000-
unit steps and searches up to 160,000 units. Each candidate uses 10 Criterion
samples, 1-second warmup and 2-second measurement. All collection shapes are
retained, with primitive execution automatic. The resulting crossover is a
coarse estimate; confirm any selected production threshold with longer runs
around its boundary.

The full 2026-11-07 calibration is recorded in
`benches/collection_threshold_results.md`. Most sources favored outer
parallelism below 20,000 units, but the mesh cases were more conservative:
4-triangle mesh was 20.7% slower at 39,936 units and 7.0% faster at 49,920;
32-triangle mesh was approximately tied at 39,936.

A separate fixed-size check at 40k/50k/60k/80k work units (outer serial versus
parallel, primitives automatic) confirmed these parallel/serial ratios:

| Collection | ~40k | ~50k | ~60k | ~80k |
| --- | ---: | ---: | ---: | ---: |
| Mesh, 4 triangles | 1.003 | 0.849 | 0.888 | 0.905 |
| Mesh, 32 triangles | 0.945 | 0.843 | 0.820 | 0.829 |
| Mixed | 0.760 | 0.788 | 0.741 | 0.763 |

The selected **50,000 work-unit** production threshold uses strict `>`;
singleton collections remain outer-serial. Primitive Rayon cutovers make
timings non-monotonic, so the search's `Min step size` and `Converged` labels
are not proof of an exact crossover. This conservative threshold avoids the
mesh regressions around 40k without delaying other sources excessively.
