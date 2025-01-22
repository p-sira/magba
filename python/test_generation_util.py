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

