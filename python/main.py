# Magba is licensed under The 3-Clause BSD, see LICENSE.
# Copyright 2025 Sira Pornsiriprasert <code@psira.me>

from generate_test_magpy import (
    magnets,
    multi_source_collection,
)
from generate_test_magpy import homogeneous_source_collection
from test_generation_util import generate_points

if __name__ == "__main__":
    points, small_points = generate_points()
    magnets.generate_tests(points, small_points)
    homogeneous_source_collection.generate_tests(small_points)
    multi_source_collection.generate_tests(small_points)
