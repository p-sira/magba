"""Independent quadrature references for the 0.8.0 accuracy regressions.

Run with NumPy and SciPy: python tests/data/field_stability_generator.py --write
or --verify. No Magba or MagPyLib formulas are used. Tensor Gauss-Legendre
quadrature at orders 96 and 144 must agree before a fixture is written.
Lengths are metres, polarization teslas, current amperes, sheet density A/m.
The source pose and geometry match examples/generate_error_report.rs.
"""

import argparse
import math
import struct
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

MU0 = 1.25663706127e-6
POL = np.array([1.0, 2.0, 3.0])
POSITION = np.array([0.1, 0.2, 0.3])
ROTATION = Rotation.from_rotvec([math.pi / 7, math.pi / 6, math.pi / 5])
TETRA = np.array([[-.1, -.1, -.1], [.1, -.1, -.1], [0, .1, -.1], [0, 0, .1]])
FACES = [[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2]]
CASES = [
    ("cylinder", [-.5, .5, -.5]),
    ("cylinder", [-.5, 7 / 18, -.5]),
    ("triangle_current", [1 / 18, -5 / 18, 1 / 18]),
    ("triangle_current", [1 / 6, -.5, -1 / 18]),
    ("sheet_current", [-7 / 18, -1 / 6, -5 / 18]),
    ("tetrahedron", [-7 / 18, -5 / 18, 7 / 18]),
    ("tetrahedron", [.5, -7 / 18, -5 / 18]),
    ("triangle_magnet", [7 / 18, 1 / 6, -.5]),
    ("path_current", [1 / 6, 1 / 6, 7 / 18]),
]


def triangle_integral(point, vertices, nodes, weights):
    """Integral R/|R|³ dS, using a Duffy map to the triangle."""
    a, b = vertices[1:] - vertices[0]
    cross = np.cross(a, b)
    area = np.linalg.norm(cross)
    u, v = np.meshgrid(nodes, nodes, indexing="ij")
    surface = vertices[0] + u[..., None] * a + ((1 - u) * v)[..., None] * b
    delta = point - surface
    density = delta / np.linalg.norm(delta, axis=-1)[..., None] ** 3
    w = weights[:, None] * weights[None, :] * (1 - u) * area
    return (density * w[..., None]).sum(axis=(0, 1)), cross / area


def cylinder_field(point, nodes, weights):
    radius, height = .05, .2
    theta = nodes * (2 * math.pi)
    ct, st = np.cos(theta), np.sin(theta)
    result = np.zeros(3)
    for cap in [-1, 0, 1]:
        if cap == 0:
            surface = np.stack(np.broadcast_arrays(
                radius * ct[:, None], radius * st[:, None],
                (nodes[None, :] - .5) * height), axis=-1)
            charge = (POL[0] * ct + POL[1] * st)[:, None] * radius * height
        else:
            r = nodes * radius
            surface = np.stack(np.broadcast_arrays(
                ct[:, None] * r[None, :], st[:, None] * r[None, :],
                np.full((len(nodes), len(nodes)), cap * height / 2)), axis=-1)
            charge = cap * POL[2] * r[None, :] * radius
        delta = point - surface
        w = 2 * math.pi * weights[:, None] * weights[None, :] * charge
        result += (delta * (w / np.linalg.norm(delta, axis=-1) ** 3)[..., None]).sum(axis=(0, 1))
    return result / (4 * math.pi)


def field(source, observer, order):
    nodes, weights = np.polynomial.legendre.leggauss(order)
    nodes, weights = (nodes + 1) / 2, weights / 2
    point = ROTATION.inv().apply(np.asarray(observer) - POSITION)
    if source == "cylinder":
        result = cylinder_field(point, nodes, weights)
    elif source == "path_current":
        result = np.zeros(3)
        for start, end in zip(TETRA[:-1], TETRA[1:]):
            direction = end - start
            delta = point - (start + nodes[:, None] * direction)
            result += (np.cross(direction, delta) *
                       (weights / np.linalg.norm(delta, axis=1) ** 3)[:, None]).sum(axis=0)
        result *= MU0 / (4 * math.pi)
    else:
        if source == "triangle_current":
            triangles = [TETRA[:3]]
        elif source == "triangle_magnet":
            triangles = [np.array([[-.1, -.1, -.1], [.1, -.1, .1], [0, .2, 0]])]
        else:
            triangles = [TETRA[face] for face in FACES]
        result = np.zeros(3)
        for vertices in triangles:
            integral, normal = triangle_integral(point, vertices, nodes, weights)
            if source in ("triangle_current", "sheet_current"):
                current = POL - np.dot(POL, normal) * normal
                result += MU0 * np.cross(current, integral) / (4 * math.pi)
            else:
                result += np.dot(POL, normal) * integral / (4 * math.pi)
    return ROTATION.apply(result)


def generate():
    lines = ["source,x,y,z,bx,by,bz"]
    for source, observer in CASES:
        low, high = (field(source, observer, order) for order in (96, 144))
        if np.linalg.norm(low - high) > 2e-12 * np.linalg.norm(high):
            raise ArithmeticError(f"quadrature did not converge: {source}, {observer}")
        lines.append(source + "," + ",".join(f"{v:.17e}" for v in [*observer, *high]))
    return "\n".join(lines) + "\n"


def verify_suzanne():
    """Adjudicate the STL grid disagreement with extended-precision integrals.

    This cross-check uses analytic triangle integrals in NumPy longdouble
    (at least 64 significand bits), rather than quadrature near a mesh face.
    It is distinct from the independent quadrature used for the CSV above.
    """
    if np.finfo(np.longdouble).eps >= np.finfo(float).eps:
        raise RuntimeError("Suzanne verification requires extended-precision longdouble")
    data = Path(__file__).resolve().parents[2] / "testing/data"
    raw = (data / "suzanne.stl").read_bytes()
    count = struct.unpack_from("<I", raw, 80)[0]
    vertices = np.array([struct.unpack_from("<9f", raw, 96 + 50 * i)
                         for i in range(count)], dtype=np.longdouble).reshape(-1, 3, 3)
    point = np.loadtxt(data / "points.csv", delimiter=",")[420].astype(np.longdouble)
    edges = np.roll(vertices, -1, axis=1) - vertices
    area = np.cross(edges[:, 0], vertices[:, 2] - vertices[:, 0])
    normals = area / np.linalg.norm(area, axis=1)[:, None]
    r = vertices - point
    distances = np.linalg.norm(r, axis=2)
    numerator = -np.sum(r[:, 0] * area, axis=1)
    denominator = (np.prod(distances, axis=1)
                   + np.sum(r[:, 2] * r[:, 1], axis=1) * distances[:, 0]
                   + np.sum(r[:, 2] * r[:, 0], axis=1) * distances[:, 1]
                   + np.sum(r[:, 1] * r[:, 0], axis=1) * distances[:, 2])
    omega = 2 * np.arctan2(numerator, denominator)
    lengths = np.linalg.norm(edges, axis=2)
    terms = 2 * np.arctanh(lengths / (distances + np.roll(distances, -1, axis=1)))
    edge_integral = np.sum(edges * (terms / lengths)[..., None], axis=1)
    pi = np.arccos(np.longdouble(-1))
    fields = ((normals * omega[:, None] - np.cross(normals, edge_integral))
              * normals[:, 2, None] / (4 * pi))
    result = np.sum(fields, axis=0)
    if abs(np.sum(omega)) > 2 * pi:
        result[2] += 1
    np.testing.assert_allclose(result,
        [0.19765997606671456, 0.029222810867144054, 0.2582644043044949],
        rtol=0, atol=1e-15)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--write", action="store_true")
    mode.add_argument("--verify", action="store_true")
    args = parser.parse_args()
    content = generate()
    verify_suzanne()
    path = Path(__file__).with_name("field_stability_reference.csv")
    if args.write:
        path.write_text(content, encoding="ascii")
    else:
        actual = np.loadtxt(path, delimiter=",", skiprows=1, usecols=range(1, 7))
        expected = np.array([[float(v) for v in row.split(",")[1:]] for row in content.splitlines()[1:]])
        np.testing.assert_allclose(actual, expected, rtol=2e-12, atol=0)
