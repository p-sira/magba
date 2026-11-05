# Magnetic-field optimization regression datasets

The existing `testing/` submodule provides 1,000-point MagPyLib grids for
each source family. Those broad grids are useful but do not deliberately sample
the branch boundaries, polarizations, and degeneracies affected by the
optimizations in this PR. The additional tests below use **fixed, independent
reference values**, not another call to the optimized scalar implementation,
where analytical accuracy is being asserted.

| Dataset | Independent reference | Why these cases were added |
| --- | --- | --- |
| `cuboid_trace_reference.csv` (76 cases) | Six-face magnetic surface-charge integral; one tangential coordinate integrated analytically, the other with adaptive Gauss–Legendre quadrature | The new `f64` trace identity replaces eight `atan2` evaluations when polarization has three nonzero components. Mixed-sign polarization, interior/exterior, near-face/edge/corner and far-field cases check that it does not change the demagnetizing field. Single-axis and zero-polarization cases exercise fallback and zero branches. Translated/rotated cases protect world-to-local transforms. Six exact-face cases exercise the fallback and check the physically unambiguous *normal* component; tangential values are only required to be finite because existing face conventions are one-sided and axis-dependent. |
| `cylinder_projection_magpylib.csv` (52 cases) | MagPyLib 6.1.0 `magpylib.magnet.Cylinder.getB` | The Cartesian projection optimization removes per-point azimuth/trigonometric conversions. Tests cover arbitrary transverse-plus-axial polarization, pure axial/transverse control cases, off-axis azimuths, the centerline, the small-radius Taylor/general-formula transition, interior/exterior, finite near-rim offsets, and rotated/translating poses. These cases check both angular signs and branch selection. |
| `circular_fused_magpylib.csv` (32 cases) | MagPyLib 6.1.0 `magpylib.current.Circle.getB` | Two Bulirsch CEL evaluations now share iteration state. Centerline, tiny-radius, off-axis, finite near-wire, far-field, positive/negative current and rotated/translated poses detect coupling, sign, termination and coordinate mistakes that a regular grid might miss. |
| `field_regression_polyhedral.rs` (deterministic in-test point sets) | Independent Coulomb integral of surface magnetic charge, evaluated using 96×96 Gauss–Legendre quadrature and a Duffy triangle mapping | The face-normal, surface-charge and edge precomputation is shared by triangles, tetrahedra and mesh magnets. Rotated anisotropic shapes, three physical scales, interior/exterior tetrahedron probes, reversed winding and zero-area faces test that cached geometry has the correct sign and remains well defined. Larger deterministic observer sets additionally check that batch/parallel dispatch returns the same results as scalar evaluation. |

## Interpretation and precision

- Fixture fields are in **tesla**, positions and dimensions in **metres**,
  polarizations in **tesla**, and rotations are **rotation vectors in radians**.
  The expected values are computed in double precision and frozen as decimal
  CSV values so Rust tests have no Python or MagPyLib runtime dependency.
- Scalar and batch outputs are both compared with independent values for
  reference cases. Large additional batch point sets check scalar/batch
  agreement rather than claiming to be independent reference solutions.
- The tests distinguish normal finite offsets from singular physical
  locations. The circular-current wire and cylindrical rim are not assigned
  arbitrary finite reference fields. Single precision uses independently
  justified, less stringent error budgets near ill-conditioned branches.
- The separate cuboid quadrature reference was cross-checked against the
  existing MagPyLib reference dataset. The polyhedral quadrature is generated
  directly inside the Rust integration test, and does not use Magba's
  analytical solid-angle or edge-logarithm expressions.

## Running the tests

```sh
cargo test --test field_regression_cuboid
cargo test --test field_regression_cylinder_circular
cargo test --test field_regression_polyhedral
cargo test --no-default-features --features std,mesh --test field_regression_polyhedral
```

The `testing/` submodule is unchanged. Reference fixture generators in this
directory are for offline regeneration or verification only.

## Reproducing the independent CSV references

The two generators below are deliberately excluded from Rust tests and CI:

```sh
# Cuboid surface-charge quadrature: Python + NumPy 2.5.4.
python tests/data/cuboid_trace_generator.py --verify
python tests/data/cuboid_trace_generator.py --write

# Cylinder and circular current: use an isolated Python environment.
python -m pip install 'magpylib==6.1.0' 'numpy==2.5.4' 'scipy==1.17.1'
python tests/data/generate_cylinder_circular_reference.py --verify
python tests/data/generate_cylinder_circular_reference.py
```

The cuboid script defines all 76 cases and regenerates the entire CSV from
the physical surface-charge integral, including six exact-face symmetric
limits. The cylinder/current script keeps the fixed input columns in the
52- and 32-case CSVs and independently recomputes only their expected field
columns through MagPyLib. Neither script uses the previous expected values
as inputs to the field calculations; `--verify` compares the results to
the committed fixture without changing it.
