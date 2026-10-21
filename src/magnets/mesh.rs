/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use nalgebra::Vector3;
#[cfg(feature = "mesh")]
use openmesh::MeshError;

use crate::base::Float;
use crate::base::mesh::TriMesh;
use crate::crate_utils::define_source;
use alloc::vec::Vec;

define_source! {
    /// Triangular mesh with homogeneous magnetic surface charge.
    #[cfg_attr(feature = "graphics-plotly", doc = include_str!("graphics/mesh.md"))]
    MeshMagnet
    field_fn: mesh_B
    relative_complexity: |source| source.mesh.triangles().len().saturating_mul(4);
    args: {
        polarization: Vector3<T> = Vector3::z(),
        mesh: @ref TriMesh<T> = TriMesh::new_unchecked(Vec::new(), Vec::new()),
    }
    arg_display: "pol={}, triangles count: {}";
    arg_fmt: [format_vector3, format_trimesh_count]
    docs: {
        new: {
            /// Construct a [MeshMagnet].
            ///
            /// This constructor does not validate the mesh. The field computation guarantees
            /// correct results only if the mesh is closed, connected, not self-intersecting
            /// and all faces are oriented outwards. It is recommended to validate the mesh
            /// first or use [MeshMagnet::from_vertices_and_faces] constructor that validates the mesh.
            ///
            /// # Examples
            ///
            /// ```
            /// # use magba::magnets::MeshMagnet;
            /// # use magba::base::mesh::TriMesh;
            /// # use nalgebra::{UnitQuaternion, vector, Vector3};
            /// let vertices = vec![vector![-0.1, -0.1, -0.1], vector![0.1, -0.1, -0.1], vector![0.0, 0.1, -0.1], vector![0.0, 0.0, 0.1]];
            /// let faces = vec![[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2]];
            /// let mesh = TriMesh::new(vertices, faces)
            ///     .expect("Invalid mesh: The mesh has holes or zero-faces.");
            ///
            /// let magnet = MeshMagnet::new(
            ///     [0.0, 0.0, 0.0],              // position (m)
            ///     UnitQuaternion::identity(),   // orientation as unit quaternion
            ///     [0.0, 0.0, 1.0],              // polarization (T)
            ///     mesh,
            /// );
            /// ```
        }
    }
}

impl<T: Float + core::iter::Sum> MeshMagnet<T> {
    /// Validates and constructs a [`MeshMagnet`] from a [TriMesh].
    ///
    /// # Returns
    ///
    /// Returns a [MeshError] if the mesh has holes or zero-faces.
    ///
    /// # Notes
    ///
    /// To construct a [MeshMagnet] without validation, use [MeshMagnet::new].
    #[cfg(feature = "mesh")]
    #[inline]
    pub fn from_vertices_and_faces(
        vertices: Vec<Vector3<T>>,
        faces: Vec<[usize; 3]>,
        polarization: impl Into<Vector3<T>>,
    ) -> Result<Self, MeshError> {
        let trimesh = TriMesh::new(vertices, faces)?;

        Ok(Self::new(
            [T::zero(); 3],
            nalgebra::UnitQuaternion::identity(),
            polarization,
            trimesh,
        ))
    }

    /// Validates and constructs a [`MeshMagnet`] from an STL reader.
    #[cfg(feature = "io-stl")]
    #[inline]
    pub fn from_stl<R>(reader: &mut R, polarization: impl Into<Vector3<T>>) -> std::io::Result<Self>
    where
        R: std::io::Read + std::io::Seek,
    {
        let trimesh = TriMesh::from_stl(reader)?;

        Ok(Self::new(
            [T::zero(); 3],
            nalgebra::UnitQuaternion::identity(),
            polarization,
            trimesh,
        ))
    }
}

#[cfg(all(test, feature = "std"))]
crate::testing_util::generate_tests! {
    MeshMagnet
    filename: triangularmesh
    params: {
        polarization: vector![1.0, 2.0, 3.0],
        mesh: TriMesh::new_unchecked(
            vec![vector![-0.1, -0.1, -0.1], vector![0.1, -0.1, -0.1], vector![0.0, 0.1, -0.1], vector![0.0, 0.0, 0.1]],
            vec![[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2]],
        ),
    }
    rtols: {
        static: 2e-10,
        static_small: 1e-9,
        translate: 1e-9,
        rotate: 1e-10,
    }
    p95_rtols: {
        static: 2e-10,
        static_small: 1e-9,
        translate: 1e-9,
        rotate: 1e-10,
    }
    f32_rtols: {
        static: 5e-2,
        static_small: 0.2,
        translate: 5e-2,
        rotate: 1e-2,
    }
    f32_p95_rtols: {
        static: 1e-3,
        static_small: 1e-3,
        translate: 1e-3,
        rotate: 1e-3,
    }
}

#[cfg(all(test, feature = "io-stl"))]
mod stl_tests {
    use super::*;
    use crate::testing_util::*;

    #[test]
    fn test_suzanne() {
        let base_path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("testing/data");
        let stl_path = base_path.join("suzanne.stl");

        assert!(stl_path.is_file(), "Test data {:?} not found.", stl_path);

        let mut file = std::fs::File::open(&stl_path).expect("Cannot open suzanne.stl");
        let mesh: MeshMagnet<f64> =
            MeshMagnet::from_stl(&mut file, [0.0, 0.0, 1.0]).expect("Failed to read STL");
        compare_B_with_file(
            &mesh,
            &base_path.join("points.csv").to_string_lossy(),
            &base_path.join("suzanne-stl.csv").to_string_lossy(),
            1e-11,
            1e-11,
        );
    }
}

#[cfg(all(test, feature = "std"))]
mod validation_tests {
    use super::*;
    use crate::{base::Source, magnets::TetrahedronMagnet};
    use nalgebra::{Point3, vector};

    #[test]
    fn validated_mesh_matches_tetrahedron_and_rejects_holes() {
        let vertices = [
            Vector3::zeros(),
            vector![0.001, 0.0, 0.0],
            vector![0.0, 0.001, 0.0],
            vector![0.0, 0.0, 0.001],
        ];
        let faces = vec![[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2]];
        let polarization = vector![0.2, -0.3, 1.0];
        let magnet =
            MeshMagnet::from_vertices_and_faces(vertices.to_vec(), faces.clone(), polarization)
                .unwrap();
        assert_eq!(magnet.polarization(), polarization);
        assert_eq!(magnet.mesh().triangles().len(), 4);
        let tetrahedron = TetrahedronMagnet::default()
            .with_vertices(vertices)
            .with_polarization(polarization);
        for point in [
            Point3::new(0.0001, 0.0002, 0.0003),
            Point3::new(0.002, 0.003, 0.004),
        ] {
            approx::assert_relative_eq!(
                magnet.compute_B(point),
                tetrahedron.compute_B(point),
                epsilon = 1e-14
            );
        }
        assert!(
            MeshMagnet::from_vertices_and_faces(
                vertices.to_vec(),
                faces[..3].to_vec(),
                polarization
            )
            .is_err()
        );
    }

    #[cfg(feature = "io-stl")]
    #[test]
    fn invalid_stl_returns_an_error() {
        let mut input = std::io::Cursor::new(b"invalid stl");
        assert!(MeshMagnet::<f64>::from_stl(&mut input, Vector3::z()).is_err());
    }
}
