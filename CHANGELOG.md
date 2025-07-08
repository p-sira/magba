# Changelog
## 0.1
### 0.1.1
**Improvements**
- Complete the documentation.
- Reduce crate size.

**Minor Changes**
- Add `util` module for library testing.
- Change doctests to relative error instead of exact equality.
- Use csv instead of sparse mtx for test data to reduce test data size and test time.
- Updated dependencies.

### 0.1.0
**New Features**
- `fields::field_cylinder`: Compute magnetic field for cylindrical magnets
- `CylinderMagnet`: Struct for cylindrical magnet
- `SourceCollection`: Struct for homogeneous source collection
- `MultiSourceCollection`: Struct for heterogeneous source collection