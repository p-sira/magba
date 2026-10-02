# Testing

## Generating MagpyLib Test Data

The `testing/python/generate_test_magpy/magnets` module contains the `generate_test` function that will create CSVs of test data, consisting of *magnet.csv*, *magnet-small.csv*, *magnet-translate.csv*, *magnet-rotate.csv*, *magnet-rotate-translate.csv*, substituting *magnet* with the name of the class. Each file contains magnetic field vectors of a test magnet at 1,000 (10x10x10) observer points. The magnets' rotation is `[π/7, π/6, π/5]` in rotation vector form, and parameters are in sync with the Rust side. For magnet position, `(0.03, 0.02, 0.01)` is used for *magnet-small.csv*, otherwise, `(0.1, 0.2, 0.3)` is used. The other magnet parameters are similarly scaled down by 10, e.g., a magnet with polarization vector magnitude of 1 Tesla will correspond to a small magnet of 0.1 Tesla.

The observer points are generated using `get_points` and `get_points_small` functions located in `testing/python/test_generation_util` module. The observer points are spaced evenly, spanning from -0.05 to +0.05 in each axis for *small magnets* and from -0.5 to +0.5 for *non-small magnets*. As mentioned above, each axis consists of 10 data points, forming a grid of 1,000 observer points.

*magnet.csv* and *magnet-small.csv* tests the field function without translating or rotating the magnet. *magnet-translate.csv* translates the magnet by `[-0.1, -0.2, -0.3]`, and *magnet-rotate.csv* rotates the magnet by the inverse of `[π/7, π/6, π/5]` to cancel out the magnet's orientation, while *magnet-rotate-translate.csv* performs both operations. This approach allows for a consistent setup for testing the functionality of magnet position and orientation, translation, rotation, and field function.

## Accuracy Report

This report is generated on AMD Ryzen 5 4600H with Radeon Graphics @4.0 GHz RAM 16 GB running x86_64-unknown-linux-gnu rustc 1.98.1 using magba v0.7.1. The performance is benchmarked using Criterion, and the average compute times are divided by the number of test cases (1,000) to get the approximate time to compute the field function for one observer point.

### Relative Error: f64

| Function          | Median    | Mean      | P95       | Max       | Performance |
|-------------------|-----------|-----------|-----------|-----------|-------------|
| CircularCurrent   | 0.000     | 0.000     | 0.000     | 0.000     | 38.0 ns     |
| PathCurrent       | 0.000     | 0.000     | 0.000     | 0.000     | 44.4 ns     |
| SheetCurrent      | 0.000     | 0.000     | 0.000     | 0.000     | 163.7 ns    |
| TriangleCurrent   | 0.000     | 0.000     | 0.000     | 0.000     | 62.7 ns     |
| CylinderMagnet    | 2.921e-13 | 8.067e-13 | 2.049e-12 | 1.609e-10 | 55.0 ns     |
| CuboidMagnet      | 0.000     | 6.477e-15 | 5.234e-14 | 2.119e-13 | 68.4 ns     |
| Dipole            | 0.000     | 0.000     | 0.000     | 0.000     | 7.2 ns      |
| SphereMagnet      | 0.000     | 1.071e-18 | 0.000     | 1.070e-15 | 5.6 ns      |
| TetrahedronMagnet | 0.000     | 3.086e-13 | 6.514e-13 | 1.020e-10 | 97.7 ns     |
| TriangleMagnet    | 0.000     | 6.008e-15 | 1.393e-14 | 2.405e-12 | 48.4 ns     |
| MeshMagnet        | 0.000     | 3.086e-13 | 6.514e-13 | 1.020e-10 | 86.4 ns     |

### Relative Error: f32

> [!NOTE]
> Reference solutions are computed in 64-bit double precision (`f64`). For single precision (`f32`), the machine epsilon is $\epsilon \approx 1.192 \times 10^{-7}$ ($2^{-23}$). Errors below are scaled by `f32::EPSILON` and expressed in units of machine epsilon ($\epsilon$).

| Function          | Median (ε) | Mean (ε) | P95 (ε) | Max (ε) | Performance |
|-------------------|------------|----------|---------|---------|-------------|
| CircularCurrent   | 1.781      | 2.705    | 6.251   | 159.856 | 37.4 ns     |
| PathCurrent       | 2.919      | 4.312    | 11.544  | 100.721 | 38.2 ns     |
| SheetCurrent      | 30.494     | 350.535  | 742.726 | 9.861e4 | 96.7 ns     |
| TriangleCurrent   | 28.448     | 476.754  | 506.403 | 1.251e5 | 49.0 ns     |
| CylinderMagnet    | 210.260    | 1.908e3  | 2.443e3 | 5.603e5 | 48.0 ns     |
| CuboidMagnet      | 50.810     | 75.263   | 226.049 | 786.855 | 52.6 ns     |
| Dipole            | 1.035      | 1.183    | 2.562   | 4.430   | 3.4 ns      |
| SphereMagnet      | 0.967      | 1.118    | 2.369   | 4.265   | 2.8 ns      |
| TetrahedronMagnet | 248.989    | 986.615  | 2.405e3 | 1.981e5 | 80.2 ns     |
| TriangleMagnet    | 13.454     | 62.923   | 171.095 | 3.826e3 | 42.8 ns     |
| MeshMagnet        | 248.989    | 986.615  | 2.405e3 | 1.981e5 | 75.3 ns     |
