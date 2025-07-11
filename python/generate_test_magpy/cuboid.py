# Magba is licensed under The 3-Clause BSD, see LICENSE.
# Copyright 2025 Sira Pornsiriprasert <code@psira.me>

import sys
from pathlib import Path

import numpy as np
from magpylib.magnet import Cuboid
from scipy.spatial.transform import Rotation

sys.path.append(str(Path(__file__).parent.parent))

from test_generation_util import (
    get_points,
    get_points_small,
    save_test_array,
)


def test_cuboid(points):
    magnet = Cuboid(
        (0.05, 0.1, 0.2),
        Rotation.from_rotvec([np.pi / 7, np.pi / 6, np.pi / 5]),
        (0.1, 0.2, 0.3),
        (1, 2, 3),
    )
    field = magnet.getB(points)
    save_test_array("cuboid.csv", field)


def test_small_cuboid(points):
    magnet = Cuboid(
        (0.03, 0.02, 0.01),
        Rotation.from_rotvec([np.pi / 8, np.pi / 7, np.pi / 6]),
        (5e-3, 3e-3, 1e-3),
        (0.15, 0.15, 0.3),
    )
    field = magnet.getB(points)
    save_test_array("cuboid-small.csv", field)


def test_translate_cuboid(points):
    magnet = Cuboid(
        (0.05, 0.1, 0.2),
        Rotation.from_rotvec([np.pi / 7, np.pi / 6, np.pi / 5]),
        (0.1, 0.2, 0.3),
        (1, 2, 3),
    )
    magnet.move((-0.1, -0.2, -0.3))
    field = magnet.getB(points)
    save_test_array("cuboid-translate.csv", field)


def test_rotate_cuboid(points):
    rotation = Rotation.from_rotvec([np.pi / 7, np.pi / 6, np.pi / 5])
    magnet = Cuboid(
        (0.05, 0.1, 0.2),
        Rotation.from_rotvec([np.pi / 7, np.pi / 6, np.pi / 5]),
        (0.1, 0.2, 0.3),
        (1, 2, 3),
    )
    magnet.rotate(rotation.inv())
    field = magnet.getB(points)
    save_test_array("cuboid-rotate.csv", field)


def test_rotate_translate_cuboid(points):
    magnet = Cuboid(
        (0.05, 0.1, 0.2),
        Rotation.from_rotvec([np.pi / 7, np.pi / 6, np.pi / 5]),
        (0.1, 0.2, 0.3),
        (1, 2, 3),
    )
    magnet.move((0.3, 0.2, 0.1))
    magnet.rotate(Rotation.from_rotvec((np.pi / 3, np.pi / 2, np.pi)))
    field = magnet.getB(points)
    save_test_array("cuboid-rotate-translate.csv", field)


def generate_tests(points, points_small):
    test_cuboid(points)
    test_small_cuboid(points_small)
    test_translate_cuboid(points)
    test_rotate_cuboid(points)
    test_rotate_translate_cuboid(points)


def main():
    points = get_points()
    points_small = get_points_small()

    generate_tests(points, points_small)


if __name__ == "__main__":
    main()
