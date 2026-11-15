#![cfg(feature = "io-stl")]

use magba::{base::mesh::TriMesh, currents::SheetCurrent, magnets::MeshMagnet};
use nalgebra::{Vector3, vector};
use std::io::{Cursor, ErrorKind};

fn tetrahedron_stl(
    scale: f64,
    include_large: bool,
    open: bool,
    degenerate: bool,
) -> Cursor<Vec<u8>> {
    let mut stl = String::from("solid tetrahedra\n");
    let scales = if include_large {
        vec![scale, 1.0]
    } else {
        vec![scale]
    };
    for (index, size) in scales.into_iter().enumerate() {
        let offset = index as f64 * 10.0;
        let vertices = [
            [offset, 0.0, 0.0],
            [offset + size, 0.0, 0.0],
            [offset, size, 0.0],
            [offset, 0.0, size],
        ];
        let mut faces = vec![[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2]];
        if open {
            faces.pop();
        }
        if degenerate {
            faces[0] = [0, 0, 1];
        }
        for face in faces {
            stl.push_str("facet normal 0 0 0\nouter loop\n");
            for i in face {
                let [x, y, z] = vertices[i];
                stl.push_str(&format!("vertex {x} {y} {z}\n"));
            }
            stl.push_str("endloop\nendfacet\n");
        }
    }
    stl.push_str("endsolid tetrahedra\n");
    Cursor::new(stl.into_bytes())
}

macro_rules! validation_tests {
    ($module:ident, $float:ty) => {
        mod $module {
            use super::*;
            type T = $float;

            #[test]
            fn small_and_multiscale_stl_matches_constructor_validation() {
                for scale in [0.001, 1.0] {
                    for include_large in [false, true] {
                        let mesh = TriMesh::<T>::from_stl(&mut tetrahedron_stl(
                            scale,
                            include_large,
                            false,
                            false,
                        ))
                        .unwrap();
                        assert_eq!(mesh.triangles().len(), if include_large { 8 } else { 4 });
                        let mut vertices = Vec::new();
                        let mut faces = Vec::new();
                        for triangle in mesh.triangles() {
                            let face = triangle.vertices().map(|vertex| {
                                if let Some(index) = vertices.iter().position(|v| *v == vertex) {
                                    index
                                } else {
                                    vertices.push(vertex);
                                    vertices.len() - 1
                                }
                            });
                            faces.push(face);
                        }
                        assert_eq!(TriMesh::new(vertices, faces).unwrap(), mesh);
                        assert!(
                            MeshMagnet::<T>::from_stl(
                                &mut tetrahedron_stl(scale, include_large, false, false),
                                vector![0.0, 0.0, 1.0]
                            )
                            .is_ok()
                        );
                        assert!(
                            SheetCurrent::<T>::from_stl(
                                &mut tetrahedron_stl(scale, include_large, false, false),
                                vec![Vector3::x(); mesh.triangles().len()]
                            )
                            .is_ok()
                        );
                    }
                }
            }

            #[test]
            fn invalid_stl_and_density_counts_return_errors() {
                for (open, degenerate) in [(true, false), (false, true)] {
                    let error = TriMesh::<T>::from_stl(&mut tetrahedron_stl(
                        0.001, false, open, degenerate,
                    ))
                    .unwrap_err();
                    assert_eq!(error.kind(), ErrorKind::InvalidData);
                }
                for count in [0, 3, 5] {
                    let error = SheetCurrent::<T>::from_stl(
                        &mut tetrahedron_stl(0.001, false, false, false),
                        vec![Vector3::x(); count],
                    )
                    .unwrap_err();
                    assert_eq!(error.kind(), ErrorKind::InvalidInput);
                }
            }
        }
    };
}

validation_tests!(f32_tests, f32);
validation_tests!(f64_tests, f64);
