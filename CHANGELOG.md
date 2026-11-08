# Changelog

## 0.8

### 0.8.0

**Breaking Changes**

- Remove all `sum_multiple_*` field functions; use source collections to superpose fields from multiple sources ([#48](https://github.com/p-sira/magba/pull/48)).

**New Features**

- Add optional graphics support with masked field sampling, adaptive field-line tracing, configurable views and pole shading, custom collection geometry, and Plotly visualizations with offline HTML export ([#42](https://github.com/p-sira/magba/pull/42)).

**Release Improvements**

- Declare Rust 1.88 as the minimum supported version and verify it in CI ([#42](https://github.com/p-sira/magba/pull/42)).
- Correct `alloc`, `std`, and `no_std` feature guidance ([#42](https://github.com/p-sira/magba/pull/42)).
- Reduce the routine feature matrix while retaining comprehensive manual CI coverage ([#45](https://github.com/p-sira/magba/pull/45)).
- Run Ubuntu pairwise feature tests across four parallel CI runners, with broad optional-feature coverage in a separate job ([#46](https://github.com/p-sira/magba/pull/46)).
- Reduce test build artifact size with line-table-only debug information.
- Upgrade `ellip` to 1.2.4 and `codspeed-criterion-compat` to 6 ([#49](https://github.com/p-sira/magba/pull/49)).

**Performance Improvements**

- Fuse Bulirsch CEL iterations for circular currents and avoid per-point angular conversions for cylinder fields ([#46](https://github.com/p-sira/magba/pull/46)).
- Reuse the reciprocal across fused circular-current CEL updates ([#50](https://github.com/p-sira/magba/pull/50)).
- Reduce cuboid `atan2` evaluations using the diagonal-kernel trace identity for mixed polarization in `f64`, retaining the more accurate direct `f32` path ([#46](https://github.com/p-sira/magba/pull/46)).
- Precompute face normals, surface charges, and edge geometry for triangle, tetrahedron, and mesh batch field evaluations ([#46](https://github.com/p-sira/magba/pull/46)).

**Testing**

- Raise the line-coverage gate to 99.1% and correct the axial CEL fallback test ([#44](https://github.com/p-sira/magba/pull/44)).
- Add independent cuboid and polyhedral surface-integral regressions, plus analytical cylinder and circular-current invariants ([#46](https://github.com/p-sira/magba/pull/46)).
- Validate all 11 field-source primitives against 169 high-precision Mathematica/Wolfram references in both `f32` and `f64`, including alternate geometries, boundaries, translations, and rotations ([#47](https://github.com/p-sira/magba/pull/47)).

**Documentation**

- Add organized graphics examples, rendered previews, runnable field-line demonstrations, and configuration guidance ([#42](https://github.com/p-sira/magba/pull/42)).

**Bug Fixes**

- Improve cuboid field accuracy near edges of slender magnets by rationalizing cancellation-prone logarithmic terms, including `f32` results ([#47](https://github.com/p-sira/magba/pull/47)).
- Use relative face tolerances for STL import, accepting valid small and multiscale meshes consistently with `TriMesh::new` ([#43](https://github.com/p-sira/magba/pull/43)).
- Require exactly one current density per sheet-current mesh face in constructors, setters, builders, and field functions; reject invalid updates before mutation ([#43](https://github.com/p-sira/magba/pull/43)).
- Reject nonfinite and nonpositive linear Hall supply voltages in constructors, setters, and builders, preserving configuration when validation fails ([#43](https://github.com/p-sira/magba/pull/43)).
- Reject zero or nonfinite sensitive-axis norms and nonfinite thresholds in Hall switch and latch constructors, setters, and builders, preserving sensor state when validation fails ([#43](https://github.com/p-sira/magba/pull/43)).

**API Improvements**

- Add `SheetCurrent::set_mesh_and_current_densities` to replace geometry and per-face densities together while preserving their count invariant ([#43](https://github.com/p-sira/magba/pull/43)).

**Dependencies**

- Replace `num-lazy` and `numeric_literals` with `num-lazy-proc` 0.3.0 for generic float literals and constants ([#52](https://github.com/p-sira/magba/pull/52)).

## 0.7

### 0.7.3

**Performance Improvements**

- Scale the Rayon parallelization thresholds for path-current, mesh, and sheet-current batch field computations by the number of segments or triangles evaluated per observer ([#38](https://github.com/p-sira/magba/pull/38)).
- Select serial or parallel source-collection evaluation from observer count, recursive source complexity, and direct child count, avoiding Rayon overhead for small collections ([#39](https://github.com/p-sira/magba/pull/39)).

**API Improvements**

- Add `Source::relative_complexity()` as an overridable scheduling hint, including geometry-aware and recursively aggregated estimates for built-in sources ([#39](https://github.com/p-sira/magba/pull/39)).

**Testing**

- Add calibration-only execution controls and separate primitive/collection branch instrumentation for repeatable Rayon crossover benchmarks ([#39](https://github.com/p-sira/magba/pull/39)).

### 0.7.2

**Performance Improvements**

- Use specialized math routines for cylinder field function ([#36](https://github.com/p-sira/magba/pull/36)).
- Retune parallelization threshold.

**API Improvements**

- Loosen bound for `magba::Float` trait by removing `ellip::bulirsch::BulirschConst` bound constraint.

### 0.7.1

**Performance Improvements**

- Optimize `local_circular_B` by using direct Cartesian projection, eliminating Cartesian-to-cylindrical transformations, inverse trigonometric functions (`atan2`), and trigonometric projections (`cos`, `sin`) ([#32](https://github.com/p-sira/magba/pull/32)).
- Optimize `local_cuboid_B` by eliminating matrix allocation/multiplication overhead in sign determination, combining paired logarithms into single division evaluations ($\ln(A) - \ln(B) = \ln(A/B)$), and conditionally skipping zero-polarization field components ([#32](https://github.com/p-sira/magba/pull/32)).
- Optimize `local_cylinder_B` with an axial-polarization fast path, skipping coordinate transformations when transverse polarization components are zero ([#32](https://github.com/p-sira/magba/pull/32)).
- Optimize `local_dipole_B` and `dipole_B_batch` by pre-rotating dipole moments once in global coordinates, reusing $r^2$ to compute $r$, $1/r^3$, and $1/r^5$ with a single square root, and avoiding per-observer coordinate transformations ([#32](https://github.com/p-sira/magba/pull/32)).
- Optimize `local_sphere_B` and `sphere_B_batch` by checking squared distance to skip square roots inside the sphere, pre-rotating polarization once in global coordinates, and avoiding per-observer coordinate transformations ([#32](https://github.com/p-sira/magba/pull/32)).
- Optimize `local_triangle_B` and `local_mesh_B` by returning solid angle from local triangle evaluations, eliminating duplicate displacement, distance, and solid angle calculations across faces in mesh evaluations ([#32](https://github.com/p-sira/magba/pull/32)).
- Optimize `local_triangle_current_B`, `triangle_current_B_batch`, and `local_sheet_current_B` by precomputing observer-independent local triangle coordinate systems and geometric constants outside observer loops ([#32](https://github.com/p-sira/magba/pull/32)).
- Optimize `path_current_B_batch` by precomputing line segment parameters outside the observer evaluation loop ([#32](https://github.com/p-sira/magba/pull/32)).
- Optimize batch field evaluations (`*_B_batch`) across all field sources by precomputing inverse orientation quaternions once outside observer loops ([#32](https://github.com/p-sira/magba/pull/32)).

### 0.7.0

**Breaking Changes**

- Encapsulate `Node` fields behind `component`, `component_mut`, `into_component`, and `local_offset` methods so mutable component access can be tracked safely ([#23](https://github.com/p-sira/magba/pull/23)).

**Bug Fixes**

- Preserve child edits with per-node dirty tracking while keeping untouched local offsets stable across repeated transformations ([#23](https://github.com/p-sira/magba/pull/23)).
- Correct mesh containment when a test ray crosses shared triangle edges or vertices ([#24](https://github.com/p-sira/magba/pull/24)).
- Make mesh ray-intersection tolerances scale-aware for small `f32` geometry ([#25](https://github.com/p-sira/magba/pull/25)).
- Preserve a linear Hall sensor's private sensitive axis across positive, negative, and zero sensitivity values ([#28](https://github.com/p-sira/magba/pull/28)).
- Propagate collection pose changes through nested source and observer collections ([#22](https://github.com/p-sira/magba/pull/22)).
- Use `openmesh`'s per-face relative tolerance so valid small and multiscale meshes are accepted without global rescaling ([#26](https://github.com/p-sira/magba/pull/26)).

**Testing**

- Reject non-finite accuracy-test values and compare identical zero vectors correctly ([#27](https://github.com/p-sira/magba/pull/27)).
- Increase code coverage to 99% with comprehensive tests across traits, collections, sensors, and field boundary conditions.
- Verify physical constants in `base::math` with numerical values.

## 0.6

### 0.6.2

**Bug Fixes**

- Fix missing `From` trait implementation for `MeshMagnet`, `TriangleMagnet`, and `TetrahedronMagnet` into `SourceComponent`.

**Testing**

- Test data is migrated from `tests/test-data` to `testing/data`, which is included as a submodule from the git repository `https://github.com/p-sira/magba-testing`.
- Add `MAGBA_REQUIRE_TEST_DATA` environment variable to prevent silent failure in the CI if the test data is missing.

### 0.6.1

- Update the magnetic permeability in vacuum constant to follow the Committee on Data for Science and Technology (CODATA) 2022 recommendation. The value is changed from $1.2566370614359173e-6$ (derived from 4π × 1e-7) to $1.25663706127e-6$ (based on empirical evidence).

### 0.6.0

**New Features**

- `PathCurrent`, `TriangleCurrent`, and `SheetCurrent` structs and their field functions.

**Improvements**

- Remove redundant batch calculation for `TetrahedronMagnet`.
- Retune the parallelization thresholds for field computation functions.

**Testing**

- Improve accuracy report.
- Add documentation on how to automatically tune the parallelization threshold in the module `fields`.

## 0.5

### 0.5.0

**New Features**

- Add `TriangleMagnet`, `TetrahedronMagnet`, and `MeshMagnet` structs and their field functions.

**Testing**

- Add accuracy report.

## 0.4

### 0.4.4

**Improvements**

- Add `from_iter` method to `SourceArray` and `ObserverArray`.

### 0.4.3

**Breaking Changes**

- Add Ellip's `BulirschConst<Self>` bound constraint to Magba's `Float` trait to allow true support of generic float type for cylinder field functions.

**Bug Fixes**

- Fix cylinder magnet field function failing to converge for f32.

### 0.4.2

**Bug Fixes**

- Add missing input validation for sensors.

**Testing**

- Add input validation tests.

### 0.4.1

**Bug Fixes**

- Implement missing transitive From for `SphereMagnet` -> `Magnet` -> `SourceComponent`.
- Implement missing `Sphere` variant of the `Magnet` enum.

### 0.4.0

**New Features**

- **`SphereMagnet`** Struct.
- **`currents` Module**: Magnetic sources generated by current-carrying conductors, encompassed by the `Current` enum, whose variants include `CircularCurrent`. All of the variants implement `Source` and can be integrated into the `SourceAssembly` and `SourceArray` systems.
- `compute_B_perp` for `LinearHallSensor`.

**Bug Fixes**

- Fix unimplemented Rayon parallelization in `impl_parallel_sum`.
- Fix double division in the diameter argument of `sum_multiple_cylinder_B`.
- Fix incorrect conversion between B and H.

**Performance**

- Optimize parallelization thresholds for field computation functions.

**Testing**

- Implement binary search for the parallelization thresholds.
- Add tests for `sum_multiple_*` functions.

## 0.3

### 0.3.0

#### BREAKING CHANGES

**Core Architecture**
- **Assembly Types:** Assembly holds a `Vec` of `Node` and handles both heap and stack-allocated items internally. For `SourceAssembly`, each node contains a `SourceComponent`. Likewise, the nodes of `SensorAssembly` hold `SensorComponent`. See the New Features section for more information about `Node` and the Component structs.
- **MultiSourceCollection Removed:** Use `SourceAssembly` which now supports nested and mixed types.
- **Field Trait Removed:** The `Field` trait is merged into the `Source` trait because the trait is redundant, and its name collides with nalgebra's `Field`.

**API Renames & Standardization**
- `compute_B` and `compute_B_batch`: The word "compute" suggests numerical evaluation rather than returning pre-computed values.
- `CylinderMagnet` accepts `diameter` instead of `radius`.
- `SourceAssembly`: Rename the methods `add` to `push` and `add_sources` to `extend`.

**Codebase Restructure**
- Remove the `util` module.
- `Transform` trait now indicates the ability to return `Pose` object.

**Feature Flag Reorganization**
- Available feature flags are `std`, `rayon`, `libm`, `unstable`. To install for `no_std` environments, you must also enable `libm`.

#### New Features

**Components Design**
- **Node Struct:** Assemblies now store `Node`, which holds the component's local position and the relative offset with respect to the collection's local coordinate, mitigating error accumulation during repeated transformations.
- **SourceComponent Enum:** The variants can be `Magnet`, `Assembly`, or `Custom`. Components can be grouped into `SourceAssembly`.
- **ObserverComponent Enum:** The variants can be `Sensor` or `Custom`. Likewise, they can be grouped into `ObserverAssembly`.
- **Magnet and Sensor Enums:** The variants are all structs defined in `magba::magnets` and `magba::sensors`, respectively.

**Stack Allocation**
- `SourceArray` and `ObserverArray`: A fixed-size, stack-allocated, homogeneous collection of `Source` and `Observer`, respectively.

**New Feature Flags**
- Add `no_std`, `libm`, and `alloc` feature flags.

**Sensors**
- `SensorOutput` for unified sensor output typing.
- **`magba::sensors::hall_effect`:** `LinearHallSensor`, `HallSwitch`, `HallLatch` and their corresponding read function in `magba::measurement`.

**Pose Struct**
- Introduce `Pose` struct to handle position/orientation logic centrally. `Source` delegates transformation logic here.

**Developer Experience**
- `sources!` and `observers!` macros for convenient declaration of composite Arrays and Assemblies.
- Most arguments now implement Into for automatic conversion from `std` types to `nalgebra` types.
- Add builder methods (`with_*`) to all magnet and collection structs.
- Add input validation for all constructors and setters.

#### Improvements

- **Field Computation:** Use Rust's fold idiom for LLVM auto-vectorization and Rayon's fold and reduce for better parallelization.
- **Dependency Upgrade:** Update `ellip`, the internal math backend, to v1.1.0, improving performance, removing BulirschConst constraint from `magba::Float`, and supporting `no_std`.
- **Dependency Upgrade:** Update `nalgebra` to v0.34.1.

**Documentations**
- Add `CODE_OF_CONDUCT.md` and `CONTRIBUTING.md`.
- Add testing documentation in `tests/`.
- Improve documentations.

**Testing**
- Add static tests and corresponding testing suite.
- Add `assert_close_vec!` macro for doctest.

## 0.2
### 0.2.0
**Breaking Changes**
- Functions will return bare values instead of `Result`.
- Removed `local_cyl_B_vec` as the parallelization is done at the level of global frame calculation.
- Change the function names in `field_cylinder` to cylinder instead of cyl and the argument name from `pol` to `polarization`.
- `magba::fields::conversion` submodule moved to `magba::conversion`. However, this will not affect legacy codes importing from
  `magba::conversion` as the submodule was re-exported there before.
- Core field computation functions in `fields::*` submodules are gated behind the `unstable` feature flag.

**New Features**
- Support both `f32` and `f64`.
- `fields::field_cuboid`: Computes magnetic field for cuboid magnets.
- `fields::field_dipole`: Computes magnetic field for magnetic dipole moments.
- `CuboidMagnet`: Struct for cuboid magnet.
- `Dipole`: Struct for magnetic dipole moment.
- Add `from_sources` method for `SourceCollection` and `MultiSourceCollection`.

**Improvements**
- Improve documentation.
- Update tests to use parameters that better reflect real-world scales (e.g, 1-cm magnet instead of 10-m magnet).
- Change the parallelization of `fields` calculation to the global frame step, increasing efficiency.
- Implement reasonable default value for sources instead of relying on deriving `Default` trait.

**Dependencies**
- Add `num-traits` and `numeric_literals` to support generic floats.
- Add `itertools` to assist development.
- Update dependencies.

## 0.1
### 0.1.1
**Improvements**
- Optimize performance and memory usage for non-parallel cylindrical magnetic field calculation.
- Improve the visual and performance of object display and debug formatting.
- Complete the documentation.
- Reduce crate size.

**Minor Changes**
- Add `util` module for library testing.
- Change doctests to relative error instead of exact equality.
- Use csv instead of sparse mtx for test data to reduce test data size and test time.
- Increase threshold for parallelization, likely will increase efficiency. 

**Dependencies**
- Use `getset` crate.
- Update dependencies.

### 0.1.0
**New Features**
- `fields::field_cylinder`: Computes magnetic field for cylindrical magnets.
- `CylinderMagnet`: Struct for cylindrical magnet.
- `SourceCollection`: Struct for homogeneous source collection.
- `MultiSourceCollection`: Struct for heterogeneous source collection.
