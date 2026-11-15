| Function             | Threshold | Parallel Time | Serial Time | Ratio | Exit Condition |
|----------------------|-----------|---------------|-------------|-------|----------------|
| dipole/two           | 20000     | 122.094 µs    | 181.444 µs  | 0.673 | Min step size  |
| dipole/many          | 10000     | 36.646 µs     | 80.554 µs   | 0.455 | Min step size  |
| circular/two         | 9996      | 85.223 µs     | 106.557 µs  | 0.800 | Min step size  |
| triangle-current/two | 9996      | 70.281 µs     | 104.496 µs  | 0.673 | Min step size  |
| cuboid/two           | 9996      | 84.526 µs     | 110.205 µs  | 0.767 | Min step size  |
| cylinder/two         | 9996      | 87.210 µs     | 112.321 µs  | 0.776 | Min step size  |
| sphere/two           | 20000     | 114.665 µs    | 173.214 µs  | 0.662 | Min step size  |
| triangle-magnet/two  | 10000     | 77.716 µs     | 114.494 µs  | 0.679 | Min step size  |
| tetrahedron/two      | 9988      | 81.150 µs     | 114.210 µs  | 0.711 | Min step size  |
| path/2-segments      | 10000     | 63.616 µs     | 85.290 µs   | 0.746 | Min step size  |
| path/16-segments     | 9984      | 42.443 µs     | 60.893 µs   | 0.697 | Min step size  |
| mesh/4-triangles     | 49920     | 1.095 ms      | 1.178 ms    | 0.930 | Min step size  |
| mesh/32-triangles    | 39936     | 932.433 µs    | 914.752 µs  | 1.019 | Converged      |
| sheet/4-triangles    | 9984      | 57.779 µs     | 85.920 µs   | 0.672 | Min step size  |
| sheet/32-triangles   | 9984      | 60.464 µs     | 106.074 µs  | 0.570 | Min step size  |
| mixed                | 39360     | 146.330 µs    | 209.333 µs  | 0.699 | Min step size  |
| nested               | 10000     | 48.569 µs     | 84.762 µs   | 0.573 | Min step size  |