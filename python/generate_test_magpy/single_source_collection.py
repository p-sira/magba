# Magba is licensed under The 3-Clause BSD, see LICENSE.
# Copyright 2025 Sira Pornsiriprasert <code@psira.me>

import sys
from pathlib import Path

import numpy as np
from magpylib import Collection
from magpylib.magnet import Cylinder
from scipy.io import mmwrite
from scipy.sparse import coo_array
from scipy.spatial.transform import Rotation

sys.path.append(str(Path(__file__).parent.parent))

from test_generation_util import TEST_DATA_DIR, generate_grid


def get_points():
    bounds = np.array([[-1, 1]] * 3)
    N = [10] * 3
    points = generate_grid(bounds, N)
    return points


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
    mmwrite(TEST_DATA_DIR / "cylinder-collection-result.mtx", coo_array(field))
    # print(magnets.position.tolist(), magnets.orientation.as_quat().tolist())
    # [print(magnet.position.tolist(), magnet.orientation.as_quat().tolist()) for magnet in magnets] 
    # print()

    magnets.position = (0.1, 0.15, 0.2)
    field = magnets.getB(points)
    mmwrite(TEST_DATA_DIR / "cylinder-collection-translate-result.mtx", coo_array(field))

    magnets.position = (0, 0, 0)
    # print(magnets.position.tolist(), magnets.orientation.as_quat().tolist())
    # [print(magnet.position.tolist(), magnet.orientation.as_quat().tolist()) for magnet in magnets] 
    magnets.orientation = Rotation.from_rotvec((np.pi / 3, np.pi / 4, np.pi / 5))
    field = magnets.getB(points)
    mmwrite(TEST_DATA_DIR / "cylinder-collection-rotate-result.mtx", coo_array(field))

    magnets.position = (0.1, 0.15, 0.2)
    field = magnets.getB(points)
    mmwrite(TEST_DATA_DIR / "cylinder-collection-translate-rotate-result.mtx", coo_array(field))

if __name__ == "__main__":
    points = get_points()
    mmwrite(TEST_DATA_DIR / "single-collection-points.mtx", coo_array(points))

    test_cylinder_collection(points)
