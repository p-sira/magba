# Testing

## Generating MagpyLib Test Data

The `python/generate_test_magpy/magnets` module contains the `generate_test` function that will create CSVs of test data, consisting of *magnet.csv*, *magnet-small.csv*, *magnet-translate.csv*, *magnet-rotate.csv*, *magnet-rotate-translate.csv*, substituting *magnet* with the name of the class. Each file contains magnetic field vectors of a test magnet at 1,000 (10x10x10) observer points. The magnets' rotation is `[π/7, π/6, π/5]` in rotation vector form, and parameters are in sync with the Rust side. For magnet position, `(0.03, 0.02, 0.01)` is used for *magnet-small.csv*, otherwise, `(0.1, 0.2, 0.3)` is used. The other magnet parameters are similarly scaled down by 10, e.g., a magnet with polarization vector magnitude of 1 Tesla will correspond to a small magnet of 0.1 Tesla.

The observer points are generated using `get_points` and `get_points_small` functions located in `python/test_generation_util` module. The observer points are spaced evenly, spanning from -0.05 to +0.05 in each axis for *small magnets* and from -0.5 to +0.05 *non-small magnets* As mentioned above, each axis consists of 10 data points, forming a grid of 1,000 observer points.

*magnet.csv* and *magnet-small.csv* tests the field function without translating or rotating the magnet. *magnet-translate.csv* translates the magnet by `[-0.1, -0.2, -0.3]`, and *magnet-rotate.csv* rotates the magnet by the inverse of `[π/7, π/6, π/5]` to cancel out the magnet's orientation, while *magnet-rotate-translate.csv* performs both operations. This approach allows for a consistent setup for testing the functionality of magnet position and orientation, translation, rotation, and field function.

## Accuracy Report

### Relative Error: f64

| Function          | Median    | Mean      | P95       | Max       |
|-------------------|-----------|-----------|-----------|-----------|
| CylinderMagnet    | 2.947e-13 | 3.511e-12 | 2.543e-12 | 2.501e-10 |
| CuboidMagnet      | 0.000     | 6.930e-15 | 5.568e-14 | 2.103e-13 |
| Dipole            | 1.320e-10 | 7.586e-11 | 1.320e-10 | 1.320e-10 |
| SphereMagnet      | 0.000     | 8.086e-19 | 0.000     | 8.078e-16 |
| TetrahedronMagnet | 0.000     | 3.450e-13 | 7.618e-13 | 1.020e-10 |
| TriangleMagnet    | 0.000     | 1.066e-14 | 3.274e-14 | 2.405e-12 |
| MeshMagnet        | 0.000     | 3.450e-13 | 7.618e-13 | 1.020e-10 |

### Relative Error: f32

| Function          | Median   | Mean     | P95      | Max      |
|-------------------|----------|----------|----------|----------|
| CylinderMagnet    | 2.511e-5 | 1.736e-4 | 2.912e-4 | 0.031    |
| CuboidMagnet      | 6.130e-6 | 9.213e-6 | 2.790e-5 | 9.530e-5 |
| Dipole            | 1.897e-7 | 2.186e-7 | 4.850e-7 | 8.763e-7 |
| SphereMagnet      | 1.766e-7 | 2.070e-7 | 4.614e-7 | 9.171e-7 |
| TetrahedronMagnet | 2.305e-5 | 1.174e-4 | 2.135e-4 | 0.031    |
| TriangleMagnet    | 1.285e-6 | 5.890e-6 | 1.609e-5 | 4.272e-4 |
| MeshMagnet        | 2.305e-5 | 1.174e-4 | 2.135e-4 | 0.031    |
