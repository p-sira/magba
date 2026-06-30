/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use alloc::vec::Vec;
use nalgebra::Vector3;

#[cfg(feature = "mesh")]
use openmesh::MeshError;

use crate::base::Float;
use crate::base::mesh::TriMesh;
use crate::crate_utils::define_source;

define_source! {
    /// A meshed current sheet.
    SheetCurrent
    field_fn: sheet_current_B
    args: {
        current_densities: @ref Vec<Vector3<T>> = Vec::new(),
        mesh: @ref TriMesh<T> = TriMesh::new_unchecked(Vec::new(), Vec::new()),
    }
    arg_display: "current_densities_count: {}, triangles count: {}";
    arg_fmt: [format_vertices_count, format_trimesh_count]
    docs: {
        new: {
            /// Construct a [SheetCurrent].
            ///
            /// # Examples
            ///
            /// ```
            /// # use magba::currents::SheetCurrent;
            /// # use magba::base::mesh::TriMesh;
            /// # use nalgebra::{UnitQuaternion, vector, Vector3};
            /// let vertices = vec![vector![-0.1, -0.1, -0.1], vector![0.1, -0.1, -0.1], vector![0.0, 0.1, -0.1], vector![0.0, 0.0, 0.1]];
            /// let faces = vec![[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2]];
            /// let mesh = TriMesh::new(vertices, faces)
            ///     .expect("Invalid mesh.");
            ///
            /// let sheet = SheetCurrent::new(
            ///     [0.0, 0.0, 0.0],              // position (m)
            ///     UnitQuaternion::identity(),   // orientation as unit quaternion
            ///     vec![vector![1.0, 2.0, 3.0]; 4], // current densities (A/m)
            ///     mesh,
            /// );
            /// ```
        }
    }
}

impl<T: Float + core::iter::Sum> SheetCurrent<T> {
    /// Validates and constructs a [`SheetCurrent`] from a [TriMesh].
    ///
    /// # Returns
    ///
    /// Returns a [MeshError] if the mesh has holes or zero-faces.
    ///
    /// # Notes
    ///
    /// To construct a [SheetCurrent] without validation, use [SheetCurrent::new].
    #[cfg(feature = "mesh")]
    #[inline]
    pub fn from_vertices_and_faces(
        vertices: Vec<Vector3<T>>,
        faces: Vec<[usize; 3]>,
        current_densities: Vec<Vector3<T>>,
    ) -> Result<Self, MeshError> {
        let trimesh = TriMesh::new(vertices, faces)?;

        Ok(Self::new(
            [T::zero(); 3],
            nalgebra::UnitQuaternion::identity(),
            current_densities,
            trimesh,
        ))
    }

    /// Validates and constructs a [`SheetCurrent`] from an STL reader.
    #[cfg(feature = "io-stl")]
    #[inline]
    pub fn from_stl<R>(reader: &mut R, current_densities: Vec<Vector3<T>>) -> std::io::Result<Self>
    where
        R: std::io::Read + std::io::Seek,
    {
        let trimesh = TriMesh::from_stl(reader)?;

        Ok(Self::new(
            [T::zero(); 3],
            nalgebra::UnitQuaternion::identity(),
            current_densities,
            trimesh,
        ))
    }
}

#[cfg(all(test, feature = "std"))]
crate::testing_util::generate_tests! {
    SheetCurrent
    filename: sheetcurrent
    params: {
        current_densities: vec![vector![1.0, 2.0, 3.0]; 4],
        mesh: TriMesh::new_unchecked(
            vec![vector![-0.1, -0.1, -0.1], vector![0.1, -0.1, -0.1], vector![0.0, 0.1, -0.1], vector![0.0, 0.0, 0.1]],
            vec![[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2]],
        ),
    }
    rtols: {
        static: 1e-10,
        static_small: 1e-10,
        translate: 1e-10,
        rotate: 1e-10,
    }
    p95_rtols: {
        static: 1e-10,
        static_small: 1e-10,
        translate: 1e-10,
        rotate: 1e-10,
    }
    f32_rtols: {
        static: 5e-2,
        static_small: 5e-2,
        translate: 5e-2,
        rotate: 2e-2,
    }
    f32_p95_rtols: {
        static: 1e-2,
        static_small: 1e-2,
        translate: 1e-2,
        rotate: 1e-2,
    }
}
