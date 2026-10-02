| Function                  | Threshold | Parallel Time | Serial Time | Ratio | Exit Condition |
|---------------------------|-----------|---------------|-------------|-------|----------------|
| circular_B                | 350       | 26.155 µs     | 24.775 µs   | 1.056 | Converged      |
| path_current_B            | 200       | 22.855 µs     | 25.646 µs   | 0.891 | Min step size  |
| cuboid_B                  | 150       | 25.881 µs     | 28.034 µs   | 0.923 | Converged      |
| cylinder_B                | 150       | 20.100 µs     | 17.920 µs   | 1.122 | Min step size  |
| dipole_B                  | 7500      | 46.438 µs     | 50.250 µs   | 0.924 | Converged      |
| sphere_B                  | 10000     | 61.466 µs     | 51.572 µs   | 1.192 | Min step size  |
| triangle_B                | 250       | 25.204 µs     | 25.490 µs   | 0.989 | Converged      |
| tetrahedron_B_precomputed | 100       | 25.417 µs     | 43.103 µs   | 0.590 | Min step size  |
| triangle_current_B        | 150       | 25.453 µs     | 30.704 µs   | 0.829 | Min step size  |
| mesh_B                    | 100       | 25.112 µs     | 44.604 µs   | 0.563 | Min step size  |
| sheet_current_B           | 50        | 29.016 µs     | 36.316 µs   | 0.799 | Min step size  |