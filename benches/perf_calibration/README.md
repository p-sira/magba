# Performance optimization and calibration

Magba optimizes field kernels and selects between serial and Rayon execution. These are separate decisions: a faster kernel can change the point at which parallel execution pays off. The thresholds are machine-dependent.

This guide covers the review workflow. The detailed measurements can be found in
[`collection_threshold.md`](../collection_threshold.md),
[`collection_threshold_results.md`](../collection_threshold_results.md), and
[`par_threshold.md`](../par_threshold.md).

## Benchmark roles

| Benchmark | Measures | Use |
| --- | --- | --- |
| [`bench.rs`](../bench.rs) | End-to-end `compute_B_batch` for `f32` and `f64` sources | Kernel regressions and optimizations |
| [`threshold.rs`](../threshold.rs) | Scalar-loop versus Rayon point-loop timings | Primitive Rayon threshold candidates |
| [`relative_complexity.rs`](../relative_complexity.rs) | Collection cost by source geometry | Relative scheduling weights |
| [`collection_threshold.rs`](../collection_threshold.rs) | Outer-serial versus outer-parallel collections | Collection Rayon crossover |
| [CodSpeed](../../.github/workflows/codspeed.yml) | Serial CPU simulation of `bench.rs` | Instruction-cost comparison and flamegraphs |

CodSpeed deliberately builds **without Rayon**. Thus, its simulation measurements cannot establish multithreaded wall-clock speedups or validate Rayon cutoffs. Use local Criterion timings for those decisions.

## Reproduce calibration

From the Magba repository root, with the `testing/` submodule initialized:

```sh
# Source field throughput; also exercised by CodSpeed in serial simulation.
cargo bench --locked --bench bench

# Primitive point-parallel crossovers (comparative scalar/parallel loops).
cargo bench --locked --bench threshold --features unstable,io-stl

# Relative weights: force outer collection serial, keep primitive Rayon auto.
cargo bench --locked --bench relative_complexity --features threshold-calibration,mesh

# Outer crossover: force collection serial/parallel; primitive Rayon auto.
cargo bench --locked --bench collection_threshold --features threshold-calibration,mesh
```

Use a fixed `RAYON_NUM_THREADS` for comparisons and record the value. `threshold` and `collection_threshold` can take considerably longer than `bench` because they measure multiple candidate sizes. Criterion writes estimates to `target/criterion/`; threshold benchmarks additionally update the Markdown reports in `benches/`. Review those generated changes before retaining them.

The collection search starts at 50,000 estimated work units, advances at about 10,000-unit resolution up to 160,000, and uses 10 samples, 1-second warmup and 2-second measurement per candidate. This is a **screening pass**; use repeated, longer measurements to confirm close calls or thresholds near a discontinuity.

## Interpret the scheduling rules

### Primitive Rayon

Each `*_B_batch` field kernel has a point threshold; Path, Mesh and Sheet scale the point cutoff by segments or faces. See the implementation in `src/fields/` and the measured [primitive candidates](../par_threshold.md). 

### Source collection

For `SourceAssembly` and other source collections, estimated work is

`observer_count * sum(child.relative_complexity())`.

The direct-child count must exceed one, and work must be **strictly greater** than the selected collection threshold to use outer Rayon. Nested children contribute recursively to work, but parallelization is across direct children. Singleton collections remain outer-serial, even when their primitive may run point-parallel.

Calibrate relative weights with **outer serial, primitive auto**: this models the cost of the children under their real dispatch policy. For the collection crossover, compare **outer serial versus outer parallel**, with primitive auto in both arms. Forcing all primitive loops serial would change the competition and distort the weights used by production scheduling.

The search's `Converged` means the measured parallel/serial ratio is within 5% of 1. Primitive Rayon cutovers can make results non-monotonic. Choose a conservative crossover from measured homogeneous, mixed, nested and variable-geometry collections, not the smallest candidate returned by the search.

## Review and adoption

Record the baseline and candidate commits, CPU, Rust version, dependency lock, thread settings, workloads, timing variability, accuracy checks, and the reason for the selected threshold. Reject or investigate regressions outside the intended tradeoff before modifying production constants. Use CodSpeed as a separate instruction-level signal, then verify user-visible parallel speedups with wall-clock measurements.
