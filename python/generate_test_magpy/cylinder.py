# Magba is licensed under The 3-Clause BSD, see LICENSE.
# Copyright 2025 Sira Pornsiriprasert <code@psira.me>

import sys
from pathlib import Path

import numpy as np
from magpylib.magnet import Cylinder
from scipy.spatial.transform import Rotation

sys.path.append(str(Path(__file__).parent.parent))

from test_generation_util import (TEST_DATA_DIR, get_points, get_points_small,
                                  save_array_to_file)


def test_cylinder(points):
    magnet = Cylinder(
        (0.1, 0.2, 0.3),
        Rotation.from_rotvec([np.pi / 7, np.pi / 6, np.pi / 5]),
        (1, 2),
        (1, 2, 3),
    )
    field = magnet.getB(points)
    save_array_to_file(TEST_DATA_DIR / "cylinder.csv", field)


def test_small_cylinder(points):
    magnet = Cylinder(
        (0.03, 0.02, 0.01),
        Rotation.from_rotvec([np.pi / 8, np.pi / 7, np.pi / 6]),
        (28e-3, 10e-3),
        (0.15, 0.15, 0.3),
    )
    field = magnet.getB(points)
    save_array_to_file(TEST_DATA_DIR / "cylinder-small.csv", field)


def test_translate_cylinder(points):
    magnet = Cylinder(
        (0.1, 0.2, 0.3),
        Rotation.from_rotvec([np.pi / 7, np.pi / 6, np.pi / 5]),
        (1, 2),
        (1, 2, 3),
    )
    magnet.move((-0.1, -0.2, -0.3))
    field = magnet.getB(points)
    save_array_to_file(TEST_DATA_DIR / "cylinder-translate.csv", field)


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
    save_array_to_file(TEST_DATA_DIR / "cylinder-rotate.csv", field)


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
    save_array_to_file(TEST_DATA_DIR / "cylinder-rotate-translate.csv", field)


def test_axial_cylinder(points):
    magnet = Cylinder(
        (0, 0, 0),
        Rotation.identity(),
        (1, 2),
        (0, 0, 3),
    )

    field = magnet.getB(points)
    save_array_to_file(TEST_DATA_DIR / "cylinder-axial.csv", field)


def test_diametric_cylinder(points):
    magnet = Cylinder(
        (0, 0, 0),
        Rotation.identity(),
        (1, 2),
        (0, 1, 0),
    )

    field = magnet.getB(points)
    save_array_to_file(TEST_DATA_DIR / "cylinder-diametric.csv", field)


def test_diametric_cylinder_2(points):
    magnet = Cylinder(
        (0, 0, 0),
        Rotation.identity(),
        (1, 2),
        (2, 1, 0),
    )

    field = magnet.getB(points)
    save_array_to_file(TEST_DATA_DIR / "cylinder-diametric-2.csv", field)


def generate_tests(points, points_small):
    test_cylinder(points)
    test_small_cylinder(points_small)
    test_translate_cylinder(points)
    test_rotate_cylinder(points)
    test_rotate_translate_cylinder(points)
    test_axial_cylinder(points)
    test_diametric_cylinder(points)
    test_diametric_cylinder_2(points)    

def main():
    points = get_points()
    points_small = get_points_small()

    generate_tests(points, points_small)

if __name__ == "__main__":
    main()
