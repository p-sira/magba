# Source collection Rayon calibration

Calibration date: 2026-10-05

## Host

- CPU: AMD Ryzen 5 4600H, 6 cores / 12 hardware threads
- Rayon worker availability: 12 logical CPUs
- Rust: `rustc 1.98.1 (48a229cea 2026-09-01)`, LLVM 22.1.8
- Target: `x86_64-unknown-linux-gnu`
- Profile: Cargo `bench` (optimized), default Rayon pool

## Method

`collection_threshold.rs` benchmarks forced outer-serial, forced
outer-parallel, and automatic execution independently. Primitive policies stay
automatic so the measurement isolates the additional collection iterator.
Benchmark IDs record source shape, observer count, and complexity score;
Criterion retains the raw samples under `target/criterion`.

The matrix includes every built-in source category, two path sizes, two
mesh/sheet sizes, empty/singleton/two/many direct children, mixed sources, and
flat/nested compositions. Inputs use non-zero field strengths and varied
observer positions.

## Fitted weights

Weights were rounded from paired, forced-serial timings for two-source batches
at 5,000 observers, normalized to the dipole. They are deliberately small and
conservative.

| Source | Median time | Weight |
| --- | ---: | ---: |
| Sphere | 60.6 us | 1 |
| Dipole | 82.6 us | 1 |
| Path | 229.5 us (2 segments) | 2 / segment |
| Circular current | 235.1 us | 3 |
| Cylinder | 273.7 us | 3 |
| Triangle magnet | 315.7 us | 4 |
| Triangle current | 537.0 us | 6 |
| Cuboid | 551.9 us | 7 |
| Tetrahedron | 890.1 us | 11 |
| Mesh | geometry-dependent | 4 / triangle |
| Sheet current | geometry-dependent | 6 / triangle |

The fixed-cost weights come from measured wall time, not from the standalone
kernel point thresholds. Mesh and sheet weights preserve the ordering of their
corresponding triangle kernels and scale with current topology.

## Threshold

The selected threshold is **40,000 work units**, with a strict `>` comparison.
At 12,500 units, fixed-cost sources other than Sphere were already 16-34%
faster in the outer-parallel branch, but Sphere and a two-way nested assembly
still regressed. At 20,000 units Sphere was faster while the nested case was
not yet stable. At 40,000 units the held-out two-way nested case improved from
338.7 us serial to 254.5 us automatic (median), so 40,000 was retained as the
slowest stable crossover.

Automatic execution remains serial at the threshold itself. Singleton
collections remain outer-serial at every score, leaving their child free to
select its own point-parallel path.
