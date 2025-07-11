# Magba is licensed under The 3-Clause BSD, see LICENSE.
# Copyright 2025 Sira Pornsiriprasert <code@psira.me>

from generate_test_magpy import (
    cuboid,
    cylinder,
    multi_source_collection,
    single_source_collection,
)
from test_generation_util import generate_points

if __name__ == "__main__":
    points, small_points, large_points = generate_points()
    cylinder.generate_tests(points, large_points)
    cuboid.generate_tests(points, small_points)
    single_source_collection.generate_tests(small_points)
    multi_source_collection.generate_tests(small_points)
