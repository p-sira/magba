#!/usr/bin/env python3
"""Regenerate the independent cylinder and circular-current reference fixtures.

Install pinned reference dependencies in a separate Python environment (not Rust CI)::

    python -m pip install 'magpylib==6.1.0' 'numpy==2.5.4' 'scipy==1.17.1'

From the repository root, verify without changing either CSV::

    python tests/data/generate_cylinder_circular_reference.py --verify

Regenerate the expected B columns in both CSVs::

    python tests/data/generate_cylinder_circular_reference.py

Each CSV owns immutable, deterministic observer inputs: scenario, case, x, y,
z (global-frame metres). Only these input columns and the hard-coded source
parameters below are passed to MagPyLib. Existing bx/by/bz are *never* used to
calculate a field; they are read only in --verify mode for comparison.

Reference provenance: MagPyLib 6.1.0 ``magpylib.magnet.Cylinder.getB`` and
``magpylib.current.Circle.getB`` (2026-11-05); source rotation was constructed
using SciPy ``Rotation.from_rotvec((0.24, -0.15, 0.37))`` in radians. Fields
are in tesla; positions/dimensions are in metres. Rust tests use only the
checked-in CSVs, so Python, NumPy, SciPy and MagPyLib are not needed in CI.
"""

from __future__ import annotations

import argparse
import csv
import io
import math
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path

MAGPYLIB_VERSION = "6.1.0"
COLUMNS = ("scenario", "case", "x", "y", "z", "bx", "by", "bz")
HERE = Path(__file__).resolve().parent
FIXTURES = (("cylinder_projection_magpylib.csv", 52), ("circular_fused_magpylib.csv", 32))


def make_sources():
    """Independent MagPyLib sources; no Magba or Rust field evaluation."""
    from magpylib.current import Circle
    from magpylib.magnet import Cylinder
    from scipy.spatial.transform import Rotation

    position = (0.003, -0.004, 0.002)
    orientation = Rotation.from_rotvec((0.24, -0.15, 0.37))
    cylinder = {
        "mixed_local": Cylinder(
            polarization=(0.8, -0.6, 1.2), dimension=(0.02, 0.02)
        ),
        "axial_local": Cylinder(
            polarization=(0.0, 0.0, 1.2), dimension=(0.02, 0.02)
        ),
        "transverse_local": Cylinder(
            polarization=(0.8, -0.6, 0.0), dimension=(0.02, 0.02)
        ),
        "mixed_rotated": Cylinder(
            polarization=(0.8, -0.6, 1.2),
            dimension=(0.02, 0.02),
            position=position,
            orientation=orientation,
        ),
    }
    circular = {
        "positive_local": Circle(current=2.5, diameter=0.02),
        "negative_local": Circle(current=-1.25, diameter=0.02),
        "positive_rotated": Circle(
            current=2.5,
            diameter=0.02,
            position=position,
            orientation=orientation,
        ),
    }
    return cylinder, circular


def read_inputs(path: Path, expected_count: int):
    with path.open(encoding="utf-8", newline="") as file:
        lines = file.readlines()
    comments = [line.rstrip("\r\n") for line in lines if line.startswith("#")]
    reader = csv.reader(
        line for line in lines if line.strip() and not line.startswith("#")
    )
    if tuple(next(reader)) != COLUMNS:
        raise ValueError(f"{path}: expected CSV columns {COLUMNS!r}")
    rows = list(reader)
    if len(rows) != expected_count:
        raise ValueError(f"{path}: expected {expected_count} cases, found {len(rows)}")
    seen = set()
    for row in rows:
        if len(row) != len(COLUMNS):
            raise ValueError(f"{path}: unexpected CSV row {row!r}")
        case_id = tuple(row[:2])
        if case_id in seen:
            raise ValueError(f"{path}: duplicate scenario/case {case_id}")
        seen.add(case_id)
        if not all(math.isfinite(float(value)) for value in row[2:5]):
            raise ValueError(f"{path}: invalid observer {case_id}")
    return comments, rows


def process(path: Path, expected_count: int, models, verify: bool) -> bool:
    comments, rows = read_inputs(path, expected_count)
    regenerated = []
    mismatches = []
    largest_error = 0.0

    for row in rows:
        scenario, case = row[:2]
        if scenario not in models:
            raise ValueError(f"{path}: unsupported scenario {scenario!r}")
        observer_xyz = tuple(float(value) for value in row[2:5])
        # Only stored input coordinates and fixed model parameters feed getB.
        recomputed = tuple(float(v) for v in models[scenario].getB(observer_xyz))
        if len(recomputed) != 3 or not all(math.isfinite(v) for v in recomputed):
            raise ValueError(f"{path}: nonfinite model B at {scenario}/{case}")

        if verify:
            atol = 2e-13 if path.name.startswith("cylinder") else 5e-17
            for axis, (previous, value) in enumerate(zip(row[5:8], recomputed)):
                reference = float(previous)
                error = abs(value - reference)
                largest_error = max(largest_error, error)
                if not math.isfinite(reference) or not math.isclose(
                    reference, value, rel_tol=1e-11, abs_tol=atol
                ):
                    mismatches.append(
                        f"{scenario}/{case} B[{axis}]: stored={reference:.17g}, "
                        f"new={value:.17g}, abs_error={error:.3g} T"
                    )
        else:
            # Preserve source input columns exactly; replace only bx/by/bz.
            regenerated.append(row[:5] + [format(v, ".17g") for v in recomputed])

    if verify:
        print(f"{path.name}: checked {len(rows)} cases, max abs error={largest_error:.3g} T")
        if mismatches:
            for detail in mismatches[:12]:
                print(f"  {detail}")
            print(f"  {len(mismatches)} component mismatches total")
            return False
    else:
        buffer = io.StringIO(newline="")
        writer = csv.writer(buffer, lineterminator="\n")
        writer.writerow(COLUMNS)
        writer.writerows(regenerated)
        # Preserve comment text and use consistent LF line endings so Git's
        # whitespace checks do not mistake carriage returns for trailing space.
        contents = "".join(f"{comment}\n" for comment in comments) + buffer.getvalue()
        path.write_bytes(contents.encode("utf-8"))
        print(f"{path.name}: regenerated {len(rows)} cases")
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--verify",
        action="store_true",
        help="compare fixture values with independent MagPyLib without writing files",
    )
    args = parser.parse_args()

    try:
        installed = version("magpylib")
    except PackageNotFoundError:
        parser.error("requires magpylib==6.1.0 (see installation instructions)")
    if installed != MAGPYLIB_VERSION:
        parser.error(f"requires magpylib=={MAGPYLIB_VERSION}, found {installed}")

    cylinder, circular = make_sources()
    ok = True
    for filename, count in FIXTURES:
        models = cylinder if filename.startswith("cylinder") else circular
        ok = process(HERE / filename, count, models, args.verify) and ok
    print(f"Reference implementation: MagPyLib {installed}, 84 total cases")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
