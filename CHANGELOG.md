# Changelog
## 0.3
### 0.3.0
**Breaking Changes**
- Change the argument `radius` to `diameter` in `CylinderMagnet`.
- Rename `SourceCollection` to `Collection` and `MultiSourceCollection` to `BoxedCollection`.
- Remove `util` module.
- Remove `transform` feature flag. Transformation capabilities are shipped with `sources`.
- Remove `Transform` trait.

**New Features**
- `SCollection`: Stack-allocated collection for `no_std` environment.
- Add `no_std` feature flag.
- Add builder methods (`with_*`) to all magnet structs.
- `Pose` struct for holding data for `Source` and provide transformation capabilities. `Source` delegates their transformation methods to `Pose`.
- `ZeroMagnet` and `zero_field` to serve as a placeholder in static `Collection`.

**Improvements**
- Unify `SourceCollection` and `MultiSourceCollection`. `MultiSourceCollection` is now an alias for collection of boxed sources.
- Add input validation for constructors and setters.
- `SourceCollection` holds its children's relative position in the local coordinate, mitigating error accumulation when transforming the collection.

**Documentations**
- Add code of conduct and contributing documentations.
- Add testing documentation in `tests/`.
- Improve documentations.

**Testing**
- Add static tests and corresponding testing suite.
- Add `assert_close_vec!` macro for doctest.

**Dependencies**
- Update `ellip` to v1.0.4.
- Update `nalgebra` to v0.34.1.
- Remove `getset`.
- Add `concat_idents`.
- Add `delegate`.
- Add `regex` as dev-dependency.

**Others**
- Upgrade Python dependencies.

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