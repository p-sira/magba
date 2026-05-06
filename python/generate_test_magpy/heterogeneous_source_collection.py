# Magba is licensed under The 3-Clause BSD, see LICENSE.
# Copyright 2025 Sira Pornsiriprasert <code@psira.me>

import sys
from pathlib import Path

from magpylib.misc import Dipole
import numpy as np
from magpylib import Collection
from magpylib.magnet import Cuboid, Cylinder
from scipy.spatial.transform import Rotation

sys.path.append(str(Path(__file__).parent.parent))

from test_generation_util import get_points_small, save_test_array


def test_multi_collection(points):
    rot1 = Rotation.identity()
    rot2 = Rotation.from_rotvec([0, np.pi / 3, 0])
    rot3 = Rotation.from_rotvec([0, np.pi / 2, np.pi / 2])

    magnets = Collection(
        Cylinder(
            (0.005, 0.01, 0.015),
            rot1,
            (0.04, 0.05),
            (0.1, 0.2, 0.3),
        ),
        Cuboid(
            (0.015, 0.005, 0.01),
            rot2,
            (0.02, 0.02, 0.03),
            (0.1, 0.2, 0.3),
        ),
        Dipole((0, 0, 0), rot3, (0.4, 0.5, 0.6)),
    )

    field = magnets.getB(points)
    save_test_array("multi-sources.csv", field)

    magnets.position = (0.01, 0.015, 0.02)
    field = magnets.getB(points)
    save_test_array("multi-sources-translate.csv", field)

    magnets.position = (0, 0, 0)
    magnets.orientation = Rotation.from_rotvec((np.pi / 3, np.pi / 4, np.pi / 5))
    field = magnets.getB(points)
    save_test_array("multi-sources-rotate.csv", field)

    magnets.position = (0.01, 0.015, 0.02)
    field = magnets.getB(points)
    save_test_array("multi-sources-translate-rotate.csv", field)


def generate_tests(points_small):
    test_multi_collection(points_small)


def main():
    points_small = get_points_small()
    generate_tests(points_small)


if __name__ == "__main__":
    main()
