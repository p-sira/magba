# Magba is licensed under The 3-Clause BSD, see LICENSE.
# Copyright 2025 Sira Pornsiriprasert <code@psira.me>

from collections.abc import Iterable
from pathlib import Path
import numpy as np
from numpy.typing import NDArray

TEST_DATA_DIR = Path(__file__).parent.parent / "tests" / "test-data"


def generate_grid(bounds: NDArray, N: Iterable) -> NDArray:
    linsp = [np.linspace(bounds[i, 0], bounds[i, 1], n) for i, n in enumerate(N)]
    mesh = np.meshgrid(*linsp)
    return np.column_stack([m.flatten() for m in mesh])


def save_array_to_file(file: str | Path, a: NDArray) -> None:
    np.savetxt(file, a, delimiter=",")


def get_points_large():
    """Generate a 5x5x5-m workspace"""
    bounds = np.array([[-2.5, 2.5]] * 3)
    N = [15] * 3
    points = generate_grid(bounds, N)
    return points


def get_points():
    """Generate a 2x2x2-m workspace"""
    bounds = np.array([[-1, 1]] * 3)
    N = [15] * 3
    points = generate_grid(bounds, N)
    return points


def get_points_small():
    """Generate a 10x10x10-cm workspace"""
    bounds = np.array([[-0.05, 0.05]] * 3)
    N = [15] * 3
    points = generate_grid(bounds, N)
    return points


def save_test_array(filename, a):
    save_array_to_file(TEST_DATA_DIR / filename, a)


def generate_points():
    points = get_points()
    save_test_array("points.csv", points)
    points_small = get_points_small()
    save_test_array("points-small.csv", points_small)
    points_large = get_points_large()
    save_test_array("points-large.csv", points_large)

    return points, points_small, points_large


def main():
    generate_points()


if __name__ == "__main__":
    main()
