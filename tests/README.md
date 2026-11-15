# Testing

## Generating MagpyLib Test Data

The `testing/python/generate_test_magpy/magnets` module contains the `generate_test` function that will create CSVs of test data, consisting of *magnet.csv*, *magnet-small.csv*, *magnet-translate.csv*, *magnet-rotate.csv*, *magnet-rotate-translate.csv*, substituting *magnet* with the name of the class. Each file contains magnetic field vectors of a test magnet at 1,000 (10x10x10) observer points. The magnets' rotation is `[π/7, π/6, π/5]` in rotation vector form, and parameters are in sync with the Rust side. For magnet position, `(0.03, 0.02, 0.01)` is used for *magnet-small.csv*, otherwise, `(0.1, 0.2, 0.3)` is used. The other magnet parameters are similarly scaled down by 10, e.g., a magnet with polarization vector magnitude of 1 Tesla will correspond to a small magnet of 0.1 Tesla.

The observer points are generated using `get_points` and `get_points_small` functions located in `testing/python/test_generation_util` module. The observer points are spaced evenly, spanning from -0.05 to +0.05 in each axis for *small magnets* and from -0.5 to +0.5 for *non-small magnets*. As mentioned above, each axis consists of 10 data points, forming a grid of 1,000 observer points.

*magnet.csv* and *magnet-small.csv* tests the field function without translating or rotating the magnet. *magnet-translate.csv* translates the magnet by `[-0.1, -0.2, -0.3]`, and *magnet-rotate.csv* rotates the magnet by the inverse of `[π/7, π/6, π/5]` to cancel out the magnet's orientation, while *magnet-rotate-translate.csv* performs both operations. This approach allows for a consistent setup for testing the functionality of magnet position and orientation, translation, rotation, and field function.

## Accuracy Report

This report is generated on AMD Ryzen 5 4600H with Radeon Graphics @1.4 GHz RAM 16 GB running x86_64-unknown-linux-gnu rustc 1.99.1 using magba v0.8.0. The performance is benchmarked using Criterion, and the average compute times are divided by the number of test cases (1,000) to get the approximate time to compute the field function for one observer point.

The performance column uses the latest local Criterion results; `—` indicates a missing benchmark. Benchmark geometry differs from the accuracy grid.

Regenerate timings with `cargo bench --bench bench --no-default-features --features std,mesh`, then regenerate this report with `cargo run --release --example generate_error_report`.

Relative error is `‖B − B_ref‖₂ / ‖B_ref‖₂`, computed in `f64` without an absolute cutoff. Equal zero vectors have zero error; a nonzero result against a zero reference has infinite relative error. The `f32` results include input rounding and pose transformations.

These grids measure agreement with MagPyLib. The grid tests complement the independent Wolfram fixtures in `tests/high_precision.rs`, which check both precisions at four poses. Cylinder-segment MagPyLib references can lose accuracy in the far field; independent integral and dipole-limit regressions adjudicate those cases.

### Relative Error: f64

| Function              | Median    | Mean      | P95       | Max       | Performance |
|-----------------------|-----------|-----------|-----------|-----------|-------------|
| CircularCurrent       | 3.607e-16 | 5.664e-16 | 1.455e-15 | 4.162e-14 | 63.5 ns     |
| PathCurrent           | 9.029e-16 | 1.193e-15 | 2.899e-15 | 5.019e-14 | 51.9 ns     |
| SheetCurrent          | 7.096e-15 | 5.714e-14 | 1.399e-13 | 4.770e-12 | 345.6 ns    |
| TriangleCurrent       | 6.691e-15 | 9.978e-14 | 1.220e-13 | 2.876e-11 | 205.9 ns    |
| CylinderMagnet        | 2.851e-13 | 7.409e-13 | 2.044e-12 | 7.113e-11 | 174.7 ns    |
| CylinderSegmentMagnet | 1.099e-7  | 4.251e-7  | 1.370e-6  | 1.822e-5  | —           |
| CuboidMagnet          | 1.468e-14 | 2.188e-14 | 6.847e-14 | 2.061e-13 | 419.5 ns    |
| Dipole                | 3.697e-16 | 4.288e-16 | 9.060e-16 | 1.701e-15 | 8.0 ns      |
| SphereMagnet          | 3.824e-16 | 4.341e-16 | 9.009e-16 | 1.756e-15 | 5.8 ns      |
| TetrahedronMagnet     | 6.640e-14 | 2.881e-13 | 6.224e-13 | 5.301e-11 | 397.3 ns    |
| TriangleMagnet        | 3.253e-15 | 1.620e-14 | 4.089e-14 | 4.337e-12 | 156.6 ns    |
| MeshMagnet            | 6.640e-14 | 2.881e-13 | 6.224e-13 | 5.301e-11 | 352.6 ns    |

### Relative Error: f32

> [!NOTE]
> Reference solutions are computed in 64-bit double precision (`f64`). For single precision (`f32`), the machine epsilon is $\epsilon \approx 1.192 \times 10^{-7}$ ($2^{-23}$). Errors below are scaled by `f32::EPSILON` and expressed in units of machine epsilon ($\epsilon$).
> Cylinder fields compute in native `f32` arithmetic, which can introduce cancellation, as evidenced by the results below.

| Function              | Median (ε) | Mean (ε) | P95 (ε)  | Max (ε) | Performance |
|-----------------------|------------|----------|----------|---------|-------------|
| CircularCurrent       | 1.773      | 2.688    | 6.244    | 159.370 | 58.1 ns     |
| PathCurrent           | 2.308      | 2.867    | 6.212    | 48.206  | 49.8 ns     |
| SheetCurrent          | 3.563      | 5.239    | 14.210   | 174.457 | 260.4 ns    |
| TriangleCurrent       | 3.596      | 5.224    | 13.769   | 128.081 | 133.9 ns    |
| CylinderMagnet        | 219.031    | 1632.625 | 2549.053 | 2.665e5 | 99.3 ns     |
| CylinderSegmentMagnet | 2.221      | 4.558    | 11.930   | 153.607 | —           |
| CuboidMagnet          | 50.929     | 75.300   | 226.043  | 786.855 | 192.8 ns    |
| Dipole                | 1.034      | 1.183    | 2.562    | 4.430   | 7.7 ns      |
| SphereMagnet          | 0.967      | 1.117    | 2.369    | 4.265   | 3.0 ns      |
| TetrahedronMagnet     | 30.729     | 37.439   | 93.598   | 190.986 | 177.6 ns    |
| TriangleMagnet        | 4.005      | 4.560    | 9.929    | 16.162  | 82.1 ns     |
| MeshMagnet            | 30.729     | 37.439   | 93.598   | 190.986 | 197.1 ns    |
