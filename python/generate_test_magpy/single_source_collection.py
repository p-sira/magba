# Magba is licensed under The 3-Clause BSD, see LICENSE.
# Copyright 2025 Sira Pornsiriprasert <code@psira.me>

import sys
from pathlib import Path

import numpy as np
from magpylib import Collection
from magpylib.magnet import Cuboid, Cylinder
from scipy.spatial.transform import Rotation

sys.path.append(str(Path(__file__).parent.parent))

from test_generation_util import get_points_small, save_test_array


def test_cylinder_collection(points):
    magnets = Collection(
        Cylinder(
            (0.009389999999999999, 0.0, -0.006),
            Rotation.from_rotvec(
                (1.2091995761561452, 1.209199576156145, 1.2091995761561452)
            ),
            (3e-3, 4e-3),
            (1.0, 2.0, 3.0),
        ),
        Cylinder(
            (-0.004694999999999998, 0.008131978541535878, -0.006),
            Rotation.from_rotvec(
                (1.5315599088338596, 0.41038024073191587, 0.4103802407319159)
            ),
            (4e-3, 5e-3),
            (0.4, 0.5, 0.6),
        ),
        Cylinder(
            (-0.004695000000000004, -0.008131978541535875, -0.006),
            Rotation.from_rotvec(
                (1.5315599088338594, -0.410380240731917, -0.41038024073191703)
            ),
            (5e-3, 6e-3),
            (0.9, 0.8, 0.6),
        ),
    )

    field = magnets.getB(points)
    save_test_array("cylinder-collection.csv", field)

    magnets.position = (0.01, 0.015, 0.02)
    field = magnets.getB(points)
    save_test_array("cylinder-collection-translate.csv", field)

    magnets.position = (0, 0, 0)
    magnets.orientation = Rotation.from_rotvec((np.pi / 3, np.pi / 4, np.pi / 5))
    field = magnets.getB(points)
    save_test_array("cylinder-collection-rotate.csv", field)

    magnets.position = (0.01, 0.015, 0.02)
    field = magnets.getB(points)
    save_test_array("cylinder-collection-translate-rotate.csv", field)


def test_cuboid_collection(points):
    rot1 = Rotation.identity()
    rot2 = Rotation.from_rotvec([0, np.pi / 3, 0])
    rot3 = Rotation.from_rotvec([0, 0, np.pi / 3])

    magnets = Collection(
        Cuboid(
            (0.005, 0.01, 0.015),
            rot1,
            (0.02, 0.02, 0.03),
            (0.1, 0.2, 0.3),
        ),
        Cuboid(
            (0.015, 0.005, 0.01),
            rot2,
            (0.02, 0.02, 0.03),
            (0.1, 0.2, 0.3),
        ),
        Cuboid(
            (0.01, 0.015, 0.005),
            rot3,
            (0.02, 0.02, 0.03),
            (0.1, 0.2, 0.3),
        ),
    )

    field = magnets.getB(points)
    save_test_array("cuboid-collection.csv", field)

    magnets.position = (0.01, 0.015, 0.02)
    field = magnets.getB(points)
    save_test_array("cuboid-collection-translate.csv", field)

    magnets.position = (0, 0, 0)
    magnets.orientation = Rotation.from_rotvec((np.pi / 3, np.pi / 4, np.pi / 5))
    field = magnets.getB(points)
    save_test_array("cuboid-collection-rotate.csv", field)

    magnets.position = (0.01, 0.015, 0.02)
    field = magnets.getB(points)
    save_test_array("cuboid-collection-translate-rotate.csv", field)


def generate_tests(points_small):
    test_cylinder_collection(points_small)
    test_cuboid_collection(points_small)


def main():
    points_small = get_points_small()
    generate_tests(points_small)


if __name__ == "__main__":
    main()
