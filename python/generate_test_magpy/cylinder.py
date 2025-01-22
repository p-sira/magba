# Magba is licensed under The 3-Clause BSD, see LICENSE.
# Copyright 2025 Sira Pornsiriprasert <code@psira.me>

from magpylib.magnet import Cylinder
import numpy as np
from pathlib import Path
import sys

from scipy.sparse import coo_array
from scipy.spatial.transform import Rotation
from scipy.io import mmwrite

sys.path.append(str(Path(__file__).parent.parent))

from test_generation_util import generate_grid, TEST_DATA_DIR


def get_points():
    bounds = np.array([[-5, 5]] * 3)
    N = [20] * 3
    points = generate_grid(bounds, N)
    return points


def test_cylinder(points):
    magnet = Cylinder(
        (0.1, 0.2, 0.3),
        Rotation.from_rotvec([np.pi / 7, np.pi / 6, np.pi / 5]),
        (1, 2),
        (1, 2, 3),
    )
    field = magnet.getB(points)
    mmwrite(TEST_DATA_DIR / "cylinder-result.mtx", coo_array(field))


def test_small_cylinder(points):
    magnet = Cylinder(
        (0.03, 0.02, 0.01),
        Rotation.from_rotvec([np.pi / 8, np.pi / 7, np.pi / 6]),
        (28e-3, 10e-3),
        (0.15, 0.15, 0.3),
    )
    field = magnet.getB(points)
    mmwrite(TEST_DATA_DIR / "cylinder-small-result.mtx", coo_array(field))


def test_translate_cylinder(points):
    magnet = Cylinder(
        (0.1, 0.2, 0.3),
        Rotation.from_rotvec([np.pi / 7, np.pi / 6, np.pi / 5]),
        (1, 2),
        (1, 2, 3),
    )
    magnet.move((-0.1, -0.2, -0.3))
    field = magnet.getB(points)
    mmwrite(TEST_DATA_DIR / "cylinder-translate-result.mtx", coo_array(field))


def test_rotate_cylinder(points):
    rotation = Rotation.from_rotvec([np.pi / 7, np.pi / 6, np.pi / 5])
    magnet = Cylinder(
        (0.1, 0.2, 0.3),
        Rotation.from_rotvec([np.pi / 7, np.pi / 6, np.pi / 5]),
        (1, 2),
        (1, 2, 3),
    )
    magnet.rotate(rotation.inv())
    field = magnet.getB(points)
    mmwrite(TEST_DATA_DIR / "cylinder-rotate-result.mtx", coo_array(field))


def test_rotate_translate_cylinder(points):
    magnet = Cylinder(
        (0.1, 0.2, 0.3),
        Rotation.from_rotvec([np.pi / 7, np.pi / 6, np.pi / 5]),
        (1, 2),
        (1, 2, 3),
    )
    magnet.move((3, 2, 1))
    magnet.rotate(Rotation.from_rotvec((np.pi / 3, np.pi / 2, np.pi)))
    field = magnet.getB(points)
    mmwrite(TEST_DATA_DIR / "cylinder-rotate-translate-result.mtx", coo_array(field))


if __name__ == "__main__":
    points = get_points()
    mmwrite(TEST_DATA_DIR / "cylinder-points.mtx", coo_array(points))

    test_cylinder(points)
    test_small_cylinder(points)
    test_translate_cylinder(points)
    test_rotate_cylinder(points)
    test_rotate_translate_cylinder(points)
