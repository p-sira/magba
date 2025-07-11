# Changelog
## 0.2
### 0.2.0
**Breaking Changes**
- Functions will return bare values instead of `Result`.

**New Features**
- `fields::field_cuboid`: Compute magnetic field for cuboid magnets.
- `CuboidMagnet`: Struct for cuboid magnet.

**Improvements**
- Update tests to use parameters that better reflect real-world scales (e.g, 1-cm magnet instead of 10-m magnet).

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
- Updated dependencies.

### 0.1.0
**New Features**
- `fields::field_cylinder`: Compute magnetic field for cylindrical magnets.
- `CylinderMagnet`: Struct for cylindrical magnet.
- `SourceCollection`: Struct for homogeneous source collection.
- `MultiSourceCollection`: Struct for heterogeneous source collection.