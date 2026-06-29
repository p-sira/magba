| Function                  | Threshold | Parallel Time | Serial Time | Ratio | Exit Condition |
|---------------------------|-----------|---------------|-------------|-------|----------------|
| circular_B                | 350       | 27.138 µs     | 28.636 µs   | 0.948 | Converged      |
| path_current_B            | 200       | 21.441 µs     | 25.789 µs   | 0.831 | Min step size  |
| cuboid_B                  | 50        | 18.809 µs     | 22.424 µs   | 0.839 | Min step size  |
| cylinder_B                | 100       | 19.268 µs     | 18.607 µs   | 1.036 | Converged      |
| dipole_B                  | 2500      | 34.228 µs     | 34.617 µs   | 0.989 | Converged      |
| sphere_B                  | 3100      | 36.434 µs     | 40.027 µs   | 0.910 | Converged      |
| triangle_B                | 300       | 26.763 µs     | 29.183 µs   | 0.917 | Converged      |
| tetrahedron_B_precomputed | 100       | 24.224 µs     | 38.996 µs   | 0.621 | Min step size  |
| mesh_B                    | 100       | 23.642 µs     | 40.206 µs   | 0.588 | Min step size  |
| triangle_current_B        | 100       | 19.631 µs     | 19.797 µs   | 0.992 | Converged      |
| sheet_current_B           | 100       | 28.841 µs     | 73.609 µs   | 0.392 | Min step size  |