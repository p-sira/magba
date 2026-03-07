# Magba is licensed under The 3-Clause BSD, see LICENSE.
# Copyright 2025 Sira Pornsiriprasert <code@psira.me>

import sys
from pathlib import Path

import numpy as np
from magpylib.current import Circle
from scipy.spatial.transform import Rotation

sys.path.append(str(Path(__file__).parent.parent))

from test_generation_util import get_points, get_points_small, save_test_array


def generate_test(source_class, points, points_small, **kwargs):
    class_name = source_class.__name__.lower()
    if class_name == "circle":
        class_name = "circularcurrent"

    rotation = Rotation.from_rotvec([np.pi / 7, np.pi / 6, np.pi / 5])

    source = source_class(
        current=1.0,
        position=(0.1, 0.2, 0.3),
        orientation=rotation,
        **kwargs,
    )

    small_kwargs = {
        k: v / 10 if isinstance(v, (int, float, np.ndarray, list, tuple)) else v
        for k, v in kwargs.items()
    }

    small_source = source_class(
        current=0.1,
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
    )


if __name__ == "__main__":
    points = get_points()
    points_small = get_points_small()
    generate_tests(points, points_small)
