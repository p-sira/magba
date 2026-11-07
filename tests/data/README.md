# Magnetic-field regression references

These targeted cases supplement the existing 1,000-point MagPyLib grids with
independent checks of branches affected by the field optimizations.

| Dataset | Independent reference | Coverage rationale |
| --- | --- | --- |
| `cuboid_trace_reference.csv` (76 cases) | Six-face surface-charge integral, with analytic integration in one direction and adaptive Gauss–Legendre quadrature in the other | Mixed-sign/axis-aligned/zero polarization, `f64` trace shortcut, `f32` fallback, inside/outside, near faces/edges/corners, far field, and rotated/translated cases. |
| `field_regression_polyhedral.rs` (deterministic cases) | Direct surface-charge integral using 96×96 Gauss–Legendre/Duffy quadrature | Triangle/tetrahedron/mesh geometry caching; anisotropy, rotation, scale, inside/outside, reversed winding, and degenerate faces. |

CSV fields and polarizations are in tesla, lengths in metres, and rotation
vectors in radians. Rust tests do not require Python. Other regressions test
parallel batches against analytical on-axis fields.

Six exact cuboid-face cases compare only the normal field component because
the existing tangential convention is axis-dependent. Physical wire and cylinder-rim singularities are excluded from finite-value reference checks.
Cylinder and circular-current tests use analytic on-axis identities; the stronger
independent Wolfram fixtures in `testing/data/high-precision/` cover off-axis
fields, rotated sources, near-wire points, and cylinder rims.

## Reference generation

- `cuboid_trace_generator.py` (NumPy 2.5.4) generates all 76 cuboid cases from
  surface-charge integration, cross-checked against existing MagPyLib data;
  use `--verify` to check or `--write` to regenerate.

The cuboid generator runs offline. The separate `testing/` submodule holds
reproducible independent Mathematica/Wolfram reference generators and CSVs.
