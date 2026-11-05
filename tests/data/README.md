# Magnetic-field regression references

These targeted cases supplement the existing 1,000-point MagPyLib grids with
independent checks of branches affected by the field optimizations.

| Dataset | Independent reference | Coverage rationale |
| --- | --- | --- |
| `cuboid_trace_reference.csv` (76 cases) | Six-face surface-charge integral, with analytic integration in one direction and adaptive Gauss–Legendre quadrature in the other | Mixed-sign/axis-aligned/zero polarization, `f64` trace shortcut, `f32` fallback, inside/outside, near faces/edges/corners, far field, and rotated/translated cases. |
| `cylinder_projection_magpylib.csv` (52 cases) | MagPyLib 6.1.0 `Cylinder.getB` | Mixed and axial/transverse polarization; axis, azimuth, Taylor cutoff, near-rim offsets, inside/outside, and rotated/translated cases. |
| `circular_fused_magpylib.csv` (32 cases) | MagPyLib 6.1.0 `Circle.getB` | Fused CEL correctness across near-axis/wire, far-field, current-sign, and rotated/translated cases. |
| `field_regression_polyhedral.rs` (deterministic cases) | Direct surface-charge integral using 96×96 Gauss–Legendre/Duffy quadrature | Triangle/tetrahedron/mesh geometry caching; anisotropy, rotation, scale, inside/outside, reversed winding, and degenerate faces. |

CSV fields and polarizations are in tesla, lengths in metres, and rotation
vectors in radians. Expected values are frozen in double precision; Rust tests
do not require Python. Larger batches verify scalar/batch consistency, not
independent physical accuracy.

Six exact cuboid-face cases compare only the normal field component because
the existing tangential convention is axis-dependent. Physical wire and
cylinder-rim singularities are excluded from finite-value reference checks;
`f32` tolerances account for roundoff near ill-conditioned branches.

## Reference generation

- `cuboid_trace_generator.py` (NumPy 2.5.4) generates all 76 cuboid cases from
  surface-charge integration, cross-checked against existing MagPyLib data;
  use `--verify` to check or `--write` to regenerate.
- `generate_cylinder_circular_reference.py` (MagPyLib 6.1.0, NumPy 2.5.4,
  SciPy 1.17.1) recomputes expected fields from the CSV input columns without
  using stored expected values. `--verify` checks without modifying files;
  the default mode updates expected fields.

Both generators run offline; the `testing/` submodule is unchanged.
