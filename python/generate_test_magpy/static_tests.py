# Magba is licensed under The 3-Clause BSD, see LICENSE.
# Copyright 2025 Sira Pornsiriprasert <code@psira.me>

from dataclasses import dataclass
from collections.abc import Callable
import numpy as np
from magpylib.func import cuboid_field, cylinder_field, dipole_field
from scipy.spatial.transform import Rotation

# Not meant for copy and paste. The parameters must be manually adjusted.


@dataclass
class TestCase:
    observer: list
    dimension: list
    polarization: list
    position: list
    orientation: list
    rtol: float


def str_list(l: list) -> str:
    sanitize = lambda x: repr(float(x))
    return ", ".join(map(sanitize, l))


def generate_test_cases(
    magba_func: str, magpy_func: Callable, test_cases: list[TestCase]
):
    for test_case in test_cases:
        if len(test_case.dimension) == 0:
            ans = magpy_func(
                "B",
                test_case.observer,
                test_case.polarization,
                test_case.position,
                Rotation.from_rotvec(test_case.orientation),
            )
        else:
            ans = magpy_func(
                "B",
                test_case.observer,
                test_case.dimension,
                test_case.polarization,
                test_case.position,
                Rotation.from_rotvec(test_case.orientation),
            )

        print(
            f"assert_close_vec!({magba_func}(&[point![{str_list(test_case.observer)}]], &point![{str_list(test_case.position)}], &quat_from_rotvec({str_list(test_case.orientation)}), &vector![{str_list(test_case.polarization)}], {str_list(test_case.dimension)})[0], vector![{str_list(ans)}], {test_case.rtol});"
        )


observers = np.array([1, 2, 5])

generate_test_cases(
    "cylinder_B",
    cylinder_field,
    [
        TestCase(
            [5, 6, 7],
            [1, 2],
            [0.45, 0.30, 0.15],
            [1, 2, 3],
            [np.pi / 3, np.pi / 5, np.pi / 7],
            1e-15,
        )
    ],
)

generate_test_cases(
    "cuboid_B",
    cuboid_field,
    [
        TestCase(
            [5, 6, 7],
            [1, 2, 3],
            [0.45, 0.30, 0.15],
            [1, 2, 3],
            [np.pi / 3, np.pi / 5, np.pi / 7],
            1e-15,
        )
    ],
)

generate_test_cases(
    "dipole_B",
    dipole_field,
    [
        TestCase(
            [5, 6, 7],
            [],
            [0.45, 0.30, 0.15],
            [1, 2, 3],
            [np.pi / 3, np.pi / 5, np.pi / 7],
            1e-15,
        )
    ],
)

from magpylib.func import sphere_field

generate_test_cases(
    "sphere_B",
    sphere_field,
    [
        TestCase(
            [5, 6, 7],
            [1.0],
            [0.45, 0.30, 0.15],
            [0, 0, 0],
            [0, 0, 0],
            1e-15,
        ),
        TestCase(
            [5, 6, 7],
            [1.0],
            [0.45, 0.30, 0.15],
            [1, 2, 3],
            [np.pi / 3, np.pi / 5, np.pi / 7],
            1e-15,
        ),
    ],
)
