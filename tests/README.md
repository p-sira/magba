# Testing

## Generating MagpyLib Test Data

The `python/generate_test_magpy/magnets` module contains the `generate_test` function that will create CSVs of test data, consisting of *magnet.csv*, *magnet-small.csv*, *magnet-translate.csv*, *magnet-rotate.csv*, *magnet-rotate-translate.csv*, substituting *magnet* with the name of the class. Each file contains magnetic field vectors of a test magnet at 1,000 (10x10x10) observer points. The magnets' rotation is `[π/7, π/6, π/5]` in rotation vector form, and parameters are in sync with the Rust side. For magnet position, `(0.03, 0.02, 0.01)` is used for *magnet-small.csv*, otherwise, `(0.1, 0.2, 0.3)` is used. The other magnet parameters are similarly scaled down by 10, e.g., a magnet with polarization vector magnitude of 1 Tesla will correspond to a small magnet of 0.1 Tesla.

The observer points are generated using `get_points` and `get_points_small` functions located in `python/test_generation_util` module. The observer points are spaced evenly, spanning from -0.05 to +0.05 in each axis for *small magnets* and from -0.5 to +0.5 for *non-small magnets*. As mentioned above, each axis consists of 10 data points, forming a grid of 1,000 observer points.

*magnet.csv* and *magnet-small.csv* tests the field function without translating or rotating the magnet. *magnet-translate.csv* translates the magnet by `[-0.1, -0.2, -0.3]`, and *magnet-rotate.csv* rotates the magnet by the inverse of `[π/7, π/6, π/5]` to cancel out the magnet's orientation, while *magnet-rotate-translate.csv* performs both operations. This approach allows for a consistent setup for testing the functionality of magnet position and orientation, translation, rotation, and field function.

## Accuracy Report

This report is generated on AMD Ryzen 5 4600H with Radeon Graphics @4.0 GHz RAM 16 GB running x86_64-unknown-linux-gnu rustc 1.90.0 using magba v0.6.1. The performance is benchmarked using Criterion, and the results are divided by the number of test cases (1,000) to get the approximate time to compute the field function for one observer point.

### Relative Error: f64

| Function          | Median    | Mean      | P95       | Max       | Performance |
|-------------------|-----------|-----------|-----------|-----------|-------------|
| CircularCurrent   | 0.000     | 0.000     | 0.000     | 0.000     | 46.7 ns     |
| PathCurrent       | 0.000     | 0.000     | 0.000     | 0.000     | 52.5 ns     |
| SheetCurrent      | 0.000     | 0.000     | 0.000     | 0.000     | 197.3 ns    |
| TriangleCurrent   | 0.000     | 0.000     | 0.000     | 0.000     | 78.5 ns     |
| CylinderMagnet    | 2.947e-13 | 3.511e-12 | 2.543e-12 | 2.501e-10 | 62.4 ns     |
| CuboidMagnet      | 0.000     | 6.930e-15 | 5.568e-14 | 2.103e-13 | 133.0 ns    |
| Dipole            | 0.000     | 0.000     | 0.000     | 0.000     | 31.6 ns     |
| SphereMagnet      | 0.000     | 8.086e-19 | 0.000     | 8.078e-16 | 26.6 ns     |
| TetrahedronMagnet | 0.000     | 3.450e-13 | 7.618e-13 | 1.020e-10 | 109.1 ns    |
| TriangleMagnet    | 0.000     | 1.066e-14 | 3.274e-14 | 2.405e-12 | 55.3 ns     |
| MeshMagnet        | 0.000     | 3.450e-13 | 7.618e-13 | 1.020e-10 | 109.7 ns    |

### Relative Error: f32

| Function          | Median   | Mean     | P95      | Max      | Performance |
|-------------------|----------|----------|----------|----------|-------------|
| CircularCurrent   | 2.094e-7 | 3.216e-7 | 7.188e-7 | 1.907e-5 | 45.6 ns     |
| PathCurrent       | 3.389e-7 | 5.152e-7 | 1.376e-6 | 1.197e-5 | 49.4 ns     |
| SheetCurrent      | 3.661e-6 | 4.207e-5 | 8.781e-5 | 0.012    | 117.5 ns    |
| TriangleCurrent   | 3.362e-6 | 5.663e-5 | 5.845e-5 | 0.015    | 56.2 ns     |
| CylinderMagnet    | 2.511e-5 | 1.736e-4 | 2.912e-4 | 0.031    | 52.5 ns     |
| CuboidMagnet      | 6.130e-6 | 9.213e-6 | 2.790e-5 | 9.530e-5 | 98.9 ns     |
| Dipole            | 1.897e-7 | 2.186e-7 | 4.850e-7 | 8.763e-7 | 28.2 ns     |
| SphereMagnet      | 1.766e-7 | 2.070e-7 | 4.614e-7 | 9.171e-7 | 26.8 ns     |
| TetrahedronMagnet | 2.305e-5 | 1.174e-4 | 2.135e-4 | 0.031    | 105.0 ns    |
| TriangleMagnet    | 1.285e-6 | 5.890e-6 | 1.609e-5 | 4.272e-4 | 52.0 ns     |
| MeshMagnet        | 2.305e-5 | 1.174e-4 | 2.135e-4 | 0.031    | 100.6 ns    |
