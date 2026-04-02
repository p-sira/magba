# Magba is licensed under The 3-Clause BSD, see LICENSE.
# Copyright 2025 Sira Pornsiriprasert <code@psira.me>

import sys
from pathlib import Path

import numpy as np
from magpylib.magnet import Cuboid, Cylinder, Sphere, Tetrahedron
from magpylib.misc import Dipole, Triangle
from scipy.spatial.transform import Rotation

sys.path.append(str(Path(__file__).parent.parent))

from test_generation_util import get_points, get_points_small, save_test_array


def generate_test(source_class, points, points_small, *args):
    class_name = source_class.__name__.lower()
    rotation = Rotation.from_rotvec([np.pi / 7, np.pi / 6, np.pi / 5])

    magnet = source_class(
        (0.1, 0.2, 0.3),
        rotation,
        *args,
    )

    small_magnet = source_class(
        (0.03, 0.02, 0.01),
        rotation,
        *[x / 10 for x in args],
    )

    save_test_array(f"{class_name}.csv", magnet.getB(points))
    save_test_array(f"{class_name}-small.csv", small_magnet.getB(points_small))

    magnet.move((-0.1, -0.2, -0.3))
    save_test_array(f"{class_name}-translate.csv", magnet.getB(points))
    magnet.move((0.1, 0.2, 0.3))
    magnet.rotate(rotation.inv())
    save_test_array(f"{class_name}-rotate.csv", magnet.getB(points))
    magnet.move((-0.1, -0.2, -0.3))
    save_test_array(f"{class_name}-rotate-translate.csv", magnet.getB(points))


def generate_tests(points, points_small):

    generate_test(
        Cylinder,
        points,
        points_small,
        np.array((0.1, 0.2)),  # dimensions (d, h)
        np.array((1.0, 2.0, 3.0)),  # polarization
    )

    generate_test(
        Cuboid,
        points,
        points_small,
        np.array((0.1, 0.2, 0.3)),  # dimensions
        np.array((1.0, 2.0, 3.0)),  # polarization
    )

    generate_test(
        Dipole,
        points,
        points_small,
        np.array((1.0, 2.0, 3.0)),  # moment
    )

    generate_test(
        Sphere,
        points,
        points_small,
        0.1,  # diameter
        np.array((1.0, 2.0, 3.0)),  # polarization
    )

    generate_test(
        Triangle,
        points,
        points_small,
        np.array([[-0.1, -0.1, -0.1], [0.1, -0.1, 0.1], [0.0, 0.2, 0.0]]),  # vertices
        np.array((1.0, 2.0, 3.0)),  # polarization
    )

    generate_test(
        Tetrahedron,
        points,
        points_small,
        np.array([[-0.1, -0.1, -0.1], [0.1, -0.1, -0.1], [0.0, 0.1, -0.1], [0.0, 0.0, 0.1]]),  # vertices
        np.array((1.0, 2.0, 3.0)),  # polarization
    )

if __name__ == "__main__":
    points = get_points()
    points_small = get_points_small()
    generate_tests(points, points_small)
