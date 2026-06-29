# Magba is licensed under The 3-Clause BSD, see LICENSE.
# Copyright 2025 Sira Pornsiriprasert <code@psira.me>

import sys
from pathlib import Path

import numpy as np
from magpylib.magnet import Cuboid, Cylinder, Sphere, Tetrahedron, TriangularMesh
from magpylib.misc import Dipole, Triangle
from scipy.spatial.transform import Rotation

sys.path.append(str(Path(__file__).parent.parent))

from test_generation_util import get_points, get_points_small, save_test_array


def generate_test(source_class, points, points_small, **kwargs):
    class_name = source_class.__name__.lower()
    rotation = Rotation.from_rotvec([np.pi / 7, np.pi / 6, np.pi / 5])

    magnet = source_class(
        position=(0.1, 0.2, 0.3),
        orientation=rotation,
        **kwargs,
    )

    small_kwargs = {}
    for k, v in kwargs.items():
        if isinstance(v, np.ndarray) and v.dtype.kind == 'f':
            small_kwargs[k] = v / 10.0
        elif isinstance(v, float):
            small_kwargs[k] = v / 10.0
        else:
            small_kwargs[k] = v

    small_magnet = source_class(
        position=(0.03, 0.02, 0.01),
        orientation=rotation,
        **small_kwargs,
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
        dimension=np.array((0.1, 0.2)),  # dimensions (d, h)
        polarization=np.array((1.0, 2.0, 3.0)),  # polarization
    )

    generate_test(
        Cuboid,
        points,
        points_small,
        dimension=np.array((0.1, 0.2, 0.3)),  # dimensions
        polarization=np.array((1.0, 2.0, 3.0)),  # polarization
    )

    generate_test(
        Dipole,
        points,
        points_small,
        moment=np.array((1.0, 2.0, 3.0)),  # moment
    )

    generate_test(
        Sphere,
        points,
        points_small,
        diameter=0.1,  # diameter
        polarization=np.array((1.0, 2.0, 3.0)),  # polarization
    )

    generate_test(
        Triangle,
        points,
        points_small,
        vertices=np.array([[-0.1, -0.1, -0.1], [0.1, -0.1, 0.1], [0.0, 0.2, 0.0]]),  # vertices
        polarization=np.array((1.0, 2.0, 3.0)),  # polarization
    )

    generate_test(
        Tetrahedron,
        points,
        points_small,
        vertices=np.array([[-0.1, -0.1, -0.1], [0.1, -0.1, -0.1], [0.0, 0.1, -0.1], [0.0, 0.0, 0.1]]),  # vertices
        polarization=np.array((1.0, 2.0, 3.0)),  # polarization
    )

    generate_test(
        TriangularMesh,
        points,
        points_small,
        vertices=np.array([[-0.1, -0.1, -0.1], [0.1, -0.1, -0.1], [0.0, 0.1, -0.1], [0.0, 0.0, 0.1]]),  # vertices
        faces=np.array([[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2]]), # faces
        polarization=np.array((1.0, 2.0, 3.0)),  # polarization
    )



if __name__ == "__main__":
    points = get_points()
    points_small = get_points_small()
    generate_tests(points, points_small)
