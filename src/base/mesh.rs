/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use nalgebra::{Vector3, vector};
use openmesh::MeshError;

use crate::base::Float;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Triangle<T: Float>(pub Vector3<T>, pub Vector3<T>, pub Vector3<T>);

impl<T: Float> Triangle<T> {
    #[inline]
    pub fn vertices(&self) -> [Vector3<T>; 3] {
        [self.0, self.1, self.2]
    }
}

/// Möller–Trumbore ray–triangle intersection algorithm.
#[inline]
pub fn is_ray_hit<T: Float>(
    triangle: Triangle<T>,
    ray_origin: Vector3<T>,
    ray_dir: Vector3<T>,
    t_min: T,
    t_max: T,
) -> bool {
    let eps = T::epsilon() * T::from(16.0).unwrap();
    let e1 = triangle.1 - triangle.0;
    let e2 = triangle.2 - triangle.0;
    let p = ray_dir.cross(&e2);

    let det = e1.dot(&p);
    if num_traits::Float::abs(det) < eps {
        return false;
    }
    let inv_det = T::one() / det;
    let tvec = ray_origin - triangle.0;

    let u = tvec.dot(&p) * inv_det;
    if u < T::zero() || u > T::one() {
        return false;
    }

    let q = tvec.cross(&e1);
    let v = ray_dir.dot(&q) * inv_det;
    if v < T::zero() || u + v > T::one() {
        return false;
    }

    let t = e2.dot(&q) * inv_det;
    if !(t_min..=t_max).contains(&t) {
        return false;
    }

    true
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
                Triangle(
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
            .map(|face| Triangle(vertices[face[0]], vertices[face[1]], vertices[face[2]]))
            .collect();

        Self { triangles }
    }

    /// Construct a [TriMesh] from triangles without validation.
    pub fn from_triangles(triangles: Vec<Triangle<T>>) -> Self {
        Self { triangles }
    }
}
