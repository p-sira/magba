# Magba is licensed under The 3-Clause BSD, see LICENSE.
# Copyright 2025 Sira Pornsiriprasert <code@psira.me>

import sys
from pathlib import Path

import numpy as np
from stl import mesh as stl_mesh
from magpylib.magnet import TriangularMesh

sys.path.append(str(Path(__file__).parent.parent))

from test_generation_util import get_points, save_test_array

STL_FILE = Path(__file__).parent.parent.parent / "tests" / "test-data" / "suzanne.stl"
POLARIZATION = np.array([0.0, 0.0, 1.0])


def load_stl(path: Path):
    """Load an STL file and return unique vertices and face indices.

    Uses insertion-order deduplication to match the behavior of the Rust `stl_io` crate.
    """
    stl = stl_mesh.Mesh.from_file(str(path))

    # stl.vectors has shape (n_faces, 3, 3) — each face has 3 vertices with 3 coords
    all_vertices = stl.vectors.reshape(-1, 3).astype(np.float32)
    n_faces = len(stl.vectors)

    # Deduplicate vertices preserving insertion order (matching stl_io behavior)
    seen = {}
    unique_verts = []
    face_indices = []
    for tri_idx in range(n_faces):
        face = []
        for vert_idx in range(3):
            v = tuple(all_vertices[tri_idx * 3 + vert_idx])
            if v not in seen:
                seen[v] = len(unique_verts)
                unique_verts.append(v)
            face.append(seen[v])
        face_indices.append(face)

    return np.array(unique_verts, dtype=np.float64), np.array(face_indices)


def generate_stl_test():
    vertices, faces = load_stl(STL_FILE)
    points = get_points()

    magnet = TriangularMesh(
        (0.0, 0.0, 0.0),
        None,
        vertices,
        faces,
        POLARIZATION,
    )

    b_fields = magnet.getB(points)
    save_test_array("suzanne-stl.csv", b_fields)
    print(
        f"Generated suzanne-stl.csv: {len(points)} points, {len(vertices)} vertices, {len(faces)} faces"
    )


if __name__ == "__main__":
    generate_stl_test()
