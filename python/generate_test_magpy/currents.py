# Magba is licensed under The 3-Clause BSD, see LICENSE.
# Copyright 2025 Sira Pornsiriprasert <code@psira.me>

import sys
from pathlib import Path

import numpy as np
from magpylib.current import Circle, TriangleSheet, Polyline
from scipy.spatial.transform import Rotation

sys.path.append(str(Path(__file__).parent.parent))

from test_generation_util import get_points, get_points_small, save_test_array


class TriangleCurrent:
    pass


def generate_test(source_class, points, points_small, **kwargs):
    class_name = source_class.__name__.lower()
    if class_name == "circle":
        class_name = "circularcurrent"
    if class_name == "trianglesheet":
        class_name = "sheetcurrent"
    if class_name == "trianglecurrent":
        source_class = TriangleSheet
        kwargs["faces"] = [[0, 1, 2]]

    rotation = Rotation.from_rotvec([np.pi / 7, np.pi / 6, np.pi / 5])

    source = source_class(
        position=(0.1, 0.2, 0.3),
        orientation=rotation,
        **kwargs,
    )

    small_kwargs = {}
    for k, v in kwargs.items():
        if isinstance(v, np.ndarray) and v.dtype.kind in "fc":
            small_kwargs[k] = v / 10.0
        elif isinstance(v, (float, int)):
            small_kwargs[k] = v / 10.0
        elif isinstance(v, (list, tuple)):
            if k == "faces":
                small_kwargs[k] = v
            else:
                small_kwargs[k] = (np.array(v) / 10.0).tolist()
        else:
            small_kwargs[k] = v

    small_source = source_class(
        position=(0.03, 0.02, 0.01),
        orientation=rotation,
        **small_kwargs,
    )

    save_test_array(f"{class_name}.csv", source.getB(points))
    save_test_array(f"{class_name}-small.csv", small_source.getB(points_small))

    source.move((-0.1, -0.2, -0.3))
    save_test_array(f"{class_name}-translate.csv", source.getB(points))
    source.move((0.1, 0.2, 0.3))
    source.rotate(rotation.inv())
    save_test_array(f"{class_name}-rotate.csv", source.getB(points))
    source.move((-0.1, -0.2, -0.3))
    save_test_array(f"{class_name}-rotate-translate.csv", source.getB(points))


def generate_tests(points, points_small):

    generate_test(
        Circle,
        points,
        points_small,
        diameter=1.0,
        current=1.0,
    )

    generate_test(
        TriangleCurrent,
        points,
        points_small,
        vertices=[
            (-0.1, -0.1, -0.1),
            (0.1, -0.1, -0.1),
            (0.0, 0.1, -0.1),
        ],
        current_densities=[(1.0, 2.0, 3.0)],
    )

    generate_test(
        TriangleSheet,
        points,
        points_small,
        vertices=[
            (-0.1, -0.1, -0.1),
            (0.1, -0.1, -0.1),
            (0.0, 0.1, -0.1),
            (0.0, 0.0, 0.1),
        ],
        faces=[(0, 2, 1), (0, 1, 3), (1, 2, 3), (0, 3, 2)],
        current_densities=[
            (1.0, 2.0, 3.0),
            (1.0, 2.0, 3.0),
            (1.0, 2.0, 3.0),
            (1.0, 2.0, 3.0),
        ],
    )

    generate_test(
        Polyline,
        points,
        points_small,
        current=100.0,
        vertices=np.array([[-0.1, -0.1, -0.1], [0.1, -0.1, -0.1], [0.0, 0.1, -0.1], [0.0, 0.0, 0.1]]),
    )


if __name__ == "__main__":
    points = get_points()
    points_small = get_points_small()
    generate_tests(points, points_small)
