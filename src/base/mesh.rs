/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use nalgebra::{Vector3, vector};
use openmesh::MeshError;
use rayx::{Hit, Ray, Triangle as RayxTriangle};

use crate::base::Float;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Triangle<T: Float> {
    pub vertices: [Vector3<T>; 3],
    m: RayxTriangle<T>,
}

impl<T: Float> Triangle<T> {
    pub fn new(v1: Vector3<T>, v2: Vector3<T>, v3: Vector3<T>) -> Self {
        Self {
            vertices: [v1, v2, v3],
            m: RayxTriangle::new(v1, v2, v3).unwrap(),
        }
    }

    pub fn intersect(&self, ray: Ray<T>, t_min: T, t_max: T) -> Option<Hit<T>> {
        self.m.intersect(ray, t_min, t_max)
    }
}

/// Triangular mesh data structure with IO and validation handling.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TriMesh<T: Float> {
    pub triangles: Vec<Triangle<T>>,
}

impl<T: Float + core::iter::Sum> TriMesh<T> {
    /// Construct a [TriMesh] from vertices and faces.
    ///
    /// # Panics
    ///
    /// Panics if the mesh has holes or zero-faces.
    ///
    /// # Notes
    ///
    /// To construct a [TriMesh] without validation, use [TriMesh::new_unchecked].
    pub fn new<V, F>(vertices: V, faces: F) -> Result<Self, MeshError>
    where
        V: IntoIterator<Item = Vector3<T>>,
        F: IntoIterator<Item = [usize; 3]>,
    {
        let vertices: Vec<_> = vertices.into_iter().map(Into::into).collect();
        let faces: Vec<_> = faces.into_iter().map(Into::into).collect();
        openmesh::core::validate_mesh(&vertices, &faces, T::from(1e-4).unwrap())?;

        let triangles = faces
            .into_iter()
            .map(|face| {
                let v1 = vertices[face.0].clone();
                let v2 = vertices[face.1].clone();
                let v3 = vertices[face.2].clone();
                Triangle::new(
                    vector![v1.0, v1.1, v1.2],
                    vector![v2.0, v2.1, v2.2],
                    vector![v3.0, v3.1, v3.2],
                )
            })
            .collect();

        Ok(Self { triangles })
    }

    #[cfg(feature = "io-stl")]
    pub fn from_stl<R>(reader: &mut R) -> std::io::Result<Self>
    where
        R: std::io::Read + std::io::Seek,
    {
        let mesh: openmesh::Mesh<T> = openmesh::Mesh::from_stl(reader)
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::Other, e.to_string()))?;
        let vertices: Vec<Vector3<T>> = mesh
            .vertices
            .into_iter()
            .map(|v| Vector3::new(v.0, v.1, v.2))
            .collect();
        let faces: Vec<[usize; 3]> = mesh.faces.into_iter().map(|f| [f.0, f.1, f.2]).collect();
        Ok(Self::new_unchecked(vertices, faces))
    }
}

impl<T: Float> TriMesh<T> {
    /// Construct a [TriMesh] from vertices and faces without validation.
    pub fn new_unchecked<V, F>(vertices: V, faces: F) -> Self
    where
        V: IntoIterator<Item = Vector3<T>>,
        F: IntoIterator<Item = [usize; 3]>,
    {
        let vertices: Vec<_> = vertices.into_iter().collect();
        let triangles = faces
            .into_iter()
            .map(|face| Triangle::new(vertices[face[0]], vertices[face[1]], vertices[face[2]]))
            .collect();

        Self { triangles }
    }

    /// Construct a [TriMesh] from triangles without validation.
    pub fn from_triangles(triangles: Vec<Triangle<T>>) -> Self {
        Self { triangles }
    }
}
