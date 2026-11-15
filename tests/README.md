# Testing

## Generating MagpyLib Test Data

The `testing/python/generate_test_magpy/magnets` module contains the `generate_test` function that will create CSVs of test data, consisting of *magnet.csv*, *magnet-small.csv*, *magnet-translate.csv*, *magnet-rotate.csv*, *magnet-rotate-translate.csv*, substituting *magnet* with the name of the class. Each file contains magnetic field vectors of a test magnet at 1,000 (10x10x10) observer points. The magnets' rotation is `[π/7, π/6, π/5]` in rotation vector form, and parameters are in sync with the Rust side. For magnet position, `(0.03, 0.02, 0.01)` is used for *magnet-small.csv*, otherwise, `(0.1, 0.2, 0.3)` is used. The other magnet parameters are similarly scaled down by 10, e.g., a magnet with polarization vector magnitude of 1 Tesla will correspond to a small magnet of 0.1 Tesla.

The observer points are generated using `get_points` and `get_points_small` functions located in `testing/python/test_generation_util` module. The observer points are spaced evenly, spanning from -0.05 to +0.05 in each axis for *small magnets* and from -0.5 to +0.5 for *non-small magnets*. As mentioned above, each axis consists of 10 data points, forming a grid of 1,000 observer points.

*magnet.csv* and *magnet-small.csv* tests the field function without translating or rotating the magnet. *magnet-translate.csv* translates the magnet by `[-0.1, -0.2, -0.3]`, and *magnet-rotate.csv* rotates the magnet by the inverse of `[π/7, π/6, π/5]` to cancel out the magnet's orientation, while *magnet-rotate-translate.csv* performs both operations. This approach allows for a consistent setup for testing the functionality of magnet position and orientation, translation, rotation, and field function.

## Accuracy Report

This report is generated on magba 0.8.0 (x86_64, linux) using the committed MagPyLib 1,000-point grids. Regenerate it with `cargo run --release --example generate_error_report`.

Relative error is `‖B − B_ref‖₂ / ‖B_ref‖₂`; absolute error is `‖B − B_ref‖₂` in teslas. Errors are computed in `f64` without an absolute cutoff or normalization by machine epsilon. Equal zero vectors have zero error; a nonzero result against a zero reference has infinite relative error. The maximum absolute error and worst relative observer can occur at different points. The `f32` results include input rounding and pose transformations.

These grids measure agreement with MagPyLib. They complement the independent Wolfram fixtures in `tests/high_precision.rs`, which check both precisions at four poses. Cylinder-segment MagPyLib references can lose accuracy in the far field; independent integral and dipole-limit regressions adjudicate those cases. Run `cargo bench --bench bench` separately for performance; benchmark geometry differs from the accuracy grid.

### Relative Error: f64

| Source | Median relative | P95 relative | Max relative | Max absolute (T) | Worst relative observer (m) |
|---|---:|---:|---:|---:|---|
| CircularCurrent | 3.608e-16 | 1.461e-15 | 4.162e-14 | 1.710e-18 | (-0.055556, -0.277778, 0.277778) |
| PathCurrent | 9.036e-16 | 2.944e-15 | 5.019e-14 | 3.964e-20 | (0.166667, 0.166667, 0.388889) |
| SheetCurrent | 7.119e-15 | 1.431e-13 | 4.770e-12 | 1.060e-19 | (-0.500000, -0.277778, -0.500000) |
| TriangleCurrent | 6.697e-15 | 1.264e-13 | 2.876e-11 | 6.942e-20 | (0.055556, -0.277778, 0.055556) |
| CylinderMagnet | 2.852e-13 | 2.049e-12 | 7.113e-11 | 9.064e-14 | (-0.388889, 0.388889, -0.388889) |
| CylinderSegmentMagnet | 1.101e-7 | 1.390e-6 | 1.822e-5 | 9.181e-10 | (-0.277778, 0.055556, -0.500000) |
| CuboidMagnet | 1.471e-14 | 7.067e-14 | 2.061e-13 | 4.684e-15 | (0.500000, -0.388889, -0.500000) |
| Dipole | 3.699e-16 | 9.114e-16 | 1.701e-15 | 2.989e-18 | (0.055556, -0.055556, 0.166667) |
| SphereMagnet | 3.827e-16 | 9.076e-16 | 1.756e-15 | 1.360e-15 | (0.055556, -0.055556, 0.166667) |
| TetrahedronMagnet | 6.649e-14 | 6.427e-13 | 5.301e-11 | 7.905e-14 | (-0.388889, -0.277778, 0.388889) |
| TriangleMagnet | 3.274e-15 | 4.117e-14 | 4.337e-12 | 2.532e-14 | (0.055556, -0.500000, -0.277778) |
| MeshMagnet | 6.649e-14 | 6.427e-13 | 5.301e-11 | 7.905e-14 | (-0.388889, -0.277778, 0.388889) |

### Relative Error: f32

| Source | Median relative | P95 relative | Max relative | Max absolute (T) | Worst relative observer (m) |
|---|---:|---:|---:|---:|---|
| CircularCurrent | 2.117e-7 | 7.572e-7 | 1.900e-5 | 7.806e-10 | (-0.055556, -0.277778, 0.277778) |
| PathCurrent | 2.752e-7 | 7.406e-7 | 5.747e-6 | 4.538e-12 | (0.166667, 0.166667, 0.388889) |
| SheetCurrent | 4.250e-7 | 1.715e-6 | 2.080e-5 | 6.920e-13 | (-0.277778, -0.166667, -0.500000) |
| TriangleCurrent | 4.290e-7 | 1.659e-6 | 1.527e-5 | 1.193e-13 | (0.166667, -0.500000, -0.055556) |
| CylinderMagnet | 1.557e-7 | 3.741e-7 | 7.284e-7 | 2.777e-7 | (0.166667, 0.277778, 0.166667) |
| CylinderSegmentMagnet | 2.651e-7 | 1.422e-6 | 1.831e-5 | 5.590e-9 | (-0.277778, 0.055556, -0.500000) |
| CuboidMagnet | 6.085e-6 | 2.696e-5 | 9.380e-5 | 5.554e-7 | (-0.500000, 0.500000, -0.500000) |
| Dipole | 1.234e-7 | 3.062e-7 | 5.281e-7 | 5.378e-10 | (0.166667, 0.277778, 0.277778) |
| SphereMagnet | 1.153e-7 | 2.834e-7 | 5.084e-7 | 2.638e-7 | (0.055556, 0.166667, 0.055556) |
| TetrahedronMagnet | 3.667e-6 | 1.119e-5 | 2.277e-5 | 2.775e-7 | (0.500000, -0.388889, -0.277778) |
| TriangleMagnet | 4.777e-7 | 1.187e-6 | 1.927e-6 | 2.251e-7 | (0.388889, 0.166667, -0.500000) |
| MeshMagnet | 3.667e-6 | 1.119e-5 | 2.277e-5 | 2.775e-7 | (0.500000, -0.388889, -0.277778) |
