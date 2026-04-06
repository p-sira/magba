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

define_source! {
    /// Triangular mesh with homogeneous magnetic surface charge.
    MeshMagnet
    field_fn: mesh_B
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
    f32_rtols: {
        static: 5e-2,
        static_small: 0.2,
        translate: 5e-2,
        rotate: 1e-2,
    }
}

#[cfg(all(test, feature = "io-stl"))]
mod stl_tests {
    use super::*;
    use crate::testing_util::*;

    #[test]
    fn test_suzanne() {
        let mut file =
            std::fs::File::open("./tests/test-data/suzanne.stl").expect("Cannot open suzanne.stl");
        let mesh: MeshMagnet<f64> =
            MeshMagnet::from_stl(&mut file, [0.0, 0.0, 1.0]).expect("Failed to read STL");
        compare_B_with_file(
            &mesh,
            "./tests/test-data/points.csv",
            "./tests/test-data/suzanne-stl.csv",
            1e-11,
        );
    }
}
