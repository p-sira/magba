# Changelog
## 0.3
### 0.3.0

#### BREAKING CHANGES

**Core Architecture**
- **Unified Collection:** `Collection` holds a `Vec<Component>` and handles both heap and stack-allocated items internally.
* **MultiSourceCollection Removed:** Use `Collection` which now supports nested and mixed types.

**API Renames & Standardization**
- `CylinderMagnet` accepts `diameter` instead of `radius`.
- `Collection`: Rename the methods `add` to `push` and `add_sources` to `extend`.

**Codebase Restructure**
- Remove the `util` module.
- Remove the `transform` feature flag. Transformation capabilities are shipped with `sources`.
- `Transform` trait now indicates the ability to return `Pose` object.

#### New Features

**Magnets and Components**

**Stack Allocation & No-Std**
- `SourceArray`: A fixed-size, stack-allocated, homogeneous collection of Sources.
- `ZeroMagnet` and `zero_field` to serve as a placeholder in `SourceArray`.
- Add `no_std` and `libm` feature flag. 

**Pose Struct**
- Introduce `Pose` struct to handle position/orientation logic centrally. `Source` delegates transformation logic here.

**Developer Experience**
- Most arguments now implement Into for automatic conversion from `std` types to `nalgebra` types.
- Add builder methods (`with_*`) to all magnet and collection structs.
- Add input validation for all constructors and setters.

#### Improvements

- **Relative Positioning:** Collections now store its children's positions relative to the collection's local coordinate, mitigating error accumulation during repeated transformations.
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
- `fields::field_cuboid`: Compute magnetic field for cuboid magnets.
- `fields::field_dipole`: Compute magnetic field for magnetic dipole moments.
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
- `fields::field_cylinder`: Compute magnetic field for cylindrical magnets.
- `CylinderMagnet`: Struct for cylindrical magnet.
- `SourceCollection`: Struct for homogeneous source collection.
- `MultiSourceCollection`: Struct for heterogeneous source collection.