"""Regenerate/verify Magba's independent cuboid surface-charge B-field fixture.

Use::
    python tests/data/cuboid_trace_generator.py --verify
    python tests/data/cuboid_trace_generator.py --write

Requires Python with NumPy (original generation used NumPy 2.5.4); Rust tests
consume the fixed CSV and have no Python dependency. No Magba/MagpyLib kernel
is used to calculate the reference. Physical units: lengths in metres; B and
local magnet polarization in Tesla; Rodrigues rotation vectors in radians.

Reference integral (in the local cuboid coordinates):
  B(r) = M * inside + (1/(4*pi)) * integral_faces
                             (M dot n) * (r-s)/|r-s|**3 dS.

One in-plane coordinate is integrated analytically, the remaining dimension
is integrated with adaptively bisected 16/32-node Gauss-Legendre quadrature.
Near-surface points are deliberately not on edges/corners. Six exact-face rows
use the *symmetric* limit of this surface-charge integral (interior/exterior
average, Richardson extrapolation from 1e-5 and 2e-5 m), where the normal B
component is uniquely defined. The Rust test deliberately checks only normal
B for exact faces: Magba's tangential face conventions select differing
one-sided limits. Rotations follow scipy's Rotation.from_rotvec/nalgebra's
UnitQuaternion::from_scaled_axis, with B rotated to world axes.

Independent cross-check performed against the first row of the project submodule
`testing/data/cuboid.csv` (MagPyLib-generated reference): max difference
2.15e-16 T for the reference setup defined in testing/python/generate_test_magpy/magnets.py.
"""

import argparse
import math
from pathlib import Path

import numpy as np

N16, W16 = np.polynomial.legendre.leggauss(16)
N32, W32 = np.polynomial.legendre.leggauss(32)


def _gauss_integral(function, low, high, nodes, weights):
    half = (high - low) / 2.0
    return half * (function((low + high) / 2.0 + half * nodes) * weights[:, None]).sum(axis=0)


def _adaptive_integral(function, low, high, tolerance=2e-12, depth=0):
    if low == high:
        return np.zeros(3)
    a = _gauss_integral(function, low, high, N16, W16)
    b = _gauss_integral(function, low, high, N32, W32)
    if np.max(np.abs(a - b)) < max(tolerance, 2e-14):
        return b
    if depth > 32:
        raise ArithmeticError(f"quadrature failed to converge on {low}, {high}")
    mid = (low + high) / 2.0
    return (_adaptive_integral(function, low, mid, tolerance / 2.0, depth + 1)
            + _adaptive_integral(function, mid, high, tolerance / 2.0, depth + 1))


def surface_charge_b(local_point, dimensions, polarization):
    """Magnetic flux density using independent six-face surface integration."""
    point = np.asarray(local_point, dtype=float)
    dimensions = np.asarray(dimensions, dtype=float)
    polarization = np.asarray(polarization, dtype=float)
    field = polarization.copy() if np.all(np.abs(point) < dimensions / 2.0) else np.zeros(3)
    for normal_axis in range(3):
        if polarization[normal_axis] == 0.0:
            continue
        u, v = (normal_axis + 1) % 3, (normal_axis + 2) % 3
        halfu, halfv = dimensions[[u, v]] / 2.0
        ulo, uhi = point[u] + halfu, point[u] - halfu
        for sign in (-1.0, 1.0):
            distance_normal = point[normal_axis] - sign * dimensions[normal_axis] / 2.0

            def integrand(vs):
                dv = point[v] - vs
                a = distance_normal**2 + dv**2
                rlo, rhi = np.sqrt(a + ulo**2), np.sqrt(a + uhi**2)
                factor = ulo / (a * rlo) - uhi / (a * rhi)
                result = np.empty((len(dv), 3))
                result[:, normal_axis] = distance_normal * factor
                result[:, u] = 1.0 / rhi - 1.0 / rlo
                result[:, v] = dv * factor
                return result

            split = np.clip(point[v], -halfv, halfv)
            integral = (_adaptive_integral(integrand, -halfv, split)
                        + _adaptive_integral(integrand, split, halfv))
            field += sign * polarization[normal_axis] / (4.0 * math.pi) * integral
    return field


def rotation_matrix(rotvec):
    vec = np.asarray(rotvec, dtype=float)
    angle = np.linalg.norm(vec)
    if angle == 0.0:
        return np.eye(3)
    x, y, z = vec / angle
    skew = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + np.sin(angle) * skew + (1 - np.cos(angle)) * (skew @ skew)


DIMENSIONS = np.array([0.12, 0.20, 0.28])
a, b, c = DIMENSIONS / 2.0
# Different octants, interiors/exteriors, near faces/edges/corners, far field.
POINTS = [
    ('center', [0, 0, 0]),
    ('inside1', [.025, -.049, .072]),
    ('inside2', [-.032, .037, -.081]),
    ('inside3', [-.014, -.061, .096]),
    ('far', [.24, -.38, .43]),
    ('far_neg', [-.23, .31, -.54]),
    ('axis_x', [.18, 0, 0]),
    ('axis_y', [0, -.27, 0]),
    ('axis_z', [0, 0, .42]),
    ('face_x_out', [a + .0003, -.033, .061]),
    ('face_x_in', [a - .0003, .033, -.061]),
    ('face_neg_x_out', [-a - .00035, .023, .091]),
    ('face_y_out', [.021, b + .00033, -.059]),
    ('face_neg_y_in', [-.024, -b + .00033, .082]),
    ('face_z_out', [-.025, .036, c + .0004]),
    ('face_neg_z_in', [.019, -.049, -c + .0004]),
    ('edge_xy_out', [a + .0003, -b - .00035, -.042]),
    ('edge_xy_mixed', [-a + .00035, b + .0003, .031]),
    ('edge_xz_out', [a + .0003, -.021, c + .00035]),
    ('edge_yz_out', [.036, b + .00033, -c - .0004]),
    ('corner_out', [a + .00035, b + .00045, c + .0004]),
    ('corner_in', [-a + .00035, b - .0003, -c + .00045]),
    ('corner_mixed', [a - .00035, -b + .00035, c + .00045]),
    ('outside_near', [-a - .004, .001, -.002]),
]
# name, local polarization, world translation, rotation vector, point indices.
SCENARIOS = [
    ('identity_mixed', [.37, -.61, .89], [0, 0, 0], [0, 0, 0], list(range(len(POINTS)))),
    ('rotated_mixed', [.85, .27, -.64], [.33, -.26, .11], [.43, -.31, .71],
     [0, 1, 4, 8, 9, 10, 12, 13, 15, 17, 18, 20, 21, 22]),
    ('rotated_opposite', [-.48, .73, .32], [-.42, .13, -.29], [-.24, .55, -.37],
     [0, 2, 5, 9, 11, 14, 16, 19, 20, 22]),
    ('axis_x', [.73, 0, 0], [.10, -.08, .15], [.61, .16, -.28], [0, 3, 6, 9, 12, 18]),
    ('axis_y', [0, -.67, 0], [-.14, .12, -.05], [.35, .44, -.32], [0, 3, 7, 10, 13, 19]),
    ('axis_z', [0, 0, .91], [.02, .11, -.09], [-.12, .33, .65], [0, 3, 8, 11, 15, 20]),
    ('zero_pol', [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 4, 9, 20]),
]
FACES = [
    ('pos_x', 0, [.06, .022, -.037]),
    ('neg_x', 0, [-.06, -.013, .058]),
    ('pos_y', 1, [.016, .1, -.063]),
    ('neg_y', 1, [-.028, -.1, .072]),
    ('pos_z', 2, [.031, -.043, .14]),
    ('neg_z', 2, [-.019, .054, -.14]),
]
HEADER = ('case,dim_x,dim_y,dim_z,pol_x,pol_y,pol_z,pos_x,pos_y,pos_z,'
          'rot_x,rot_y,rot_z,local_x,local_y,local_z,expected_bx,expected_by,expected_bz')


def _row(name, pol, position, rotvec, local_p, field):
    values = np.concatenate((DIMENSIONS, pol, position, rotvec, local_p, field))
    assert len(values) == 18 and np.all(np.isfinite(values))
    return name + ',' + ','.join(format(float(x), '.17g') for x in values)


def generated_content():
    lines = [HEADER]
    for label, polarization, position, rotvec, indices in SCENARIOS:
        for index in indices:
            point_name, local_p = POINTS[index]
            local_b = surface_charge_b(local_p, DIMENSIONS, polarization)
            world_b = rotation_matrix(rotvec) @ local_b
            lines.append(_row(f'{label}_{point_name}', polarization, position, rotvec,
                              local_p, world_b))
    polarization = np.array([.37, -.61, .89])
    zeros = np.zeros(3)
    for name, axis, point in FACES:
        point = np.asarray(point)
        normal = np.zeros(3)
        normal[axis] = 1.0

        def limit(h):
            return (surface_charge_b(point + h * normal, DIMENSIONS, polarization)
                    + surface_charge_b(point - h * normal, DIMENSIONS, polarization)) / 2.0

        symmetric = (4.0 * limit(1e-5) - limit(2e-5)) / 3.0
        lines.append(_row('identity_mixed_exact_face_' + name, polarization, zeros,
                          zeros, point, symmetric))
    assert len(lines) == 77  # header + 70 regular + 6 face cases
    return '\n'.join(lines) + '\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--verify', action='store_true', help='compare regenerated bytes with CSV')
    mode.add_argument('--write', action='store_true', help='overwrite checked-in CSV')
    args = parser.parse_args()
    target = Path(__file__).with_name('cuboid_trace_reference.csv')
    text = generated_content()
    if args.verify:
        if target.read_text() != text:
            raise SystemExit(f'fixture differs from generator; run {__file__} --write')
        print(f'verified {len(text.splitlines()) - 1} cases in {target}')
    else:
        target.write_text(text)
        print(f'wrote {len(text.splitlines()) - 1} cases in {target}')


if __name__ == '__main__':
    main()
