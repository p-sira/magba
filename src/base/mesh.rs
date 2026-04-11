/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use nalgebra::Vector3;
use openmesh::MeshError;

use crate::base::Float;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Triangle<T: Float> {
    v1: Vector3<T>,
    v2: Vector3<T>,
    v3: Vector3<T>,
}

impl<T: Float> Triangle<T> {
    #[inline]
    pub fn new(v1: Vector3<T>, v2: Vector3<T>, v3: Vector3<T>) -> Self {
        Self { v1, v2, v3 }
    }

    #[inline]
    pub fn vertices(&self) -> [Vector3<T>; 3] {
        [self.v1, self.v2, self.v3]
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
    let e1 = triangle.v2 - triangle.v1;
    let e2 = triangle.v3 - triangle.v1;
    let p = ray_dir.cross(&e2);

    let det = e1.dot(&p);
    if num_traits::Float::abs(det) < eps {
        return false;
    }
    let inv_det = T::one() / det;
    let tvec = ray_origin - triangle.v1;

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
    triangles: Vec<Triangle<T>>,
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
    /// To construct a [TriMesh] without validation, use [TriMesh::new_unchecked]
    pub fn new<V, F>(vertices: V, faces: F) -> Result<Self, MeshError>
    where
        V: IntoIterator<Item = Vector3<T>>,
        F: IntoIterator<Item = [usize; 3]>,
    {
        let vertices: Vec<Vector3<T>> = vertices.into_iter().collect();
        let faces: Vec<[usize; 3]> = faces.into_iter().collect();

        let v_val: Vec<openmesh::Vertex<T>> = vertices.iter().map(|&v| v.into()).collect();
        let f_val: Vec<openmesh::Face> = faces.iter().map(|&f| f.into()).collect();
        openmesh::core::validate_mesh(&v_val, &f_val, T::from(1e-4).unwrap())?;

        Ok(Self::new_unchecked(vertices, faces))
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
    #[inline]
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

    #[inline]
    pub fn triangles(&self) -> &[Triangle<T>] {
        &self.triangles
    }

    /// Construct a [TriMesh] from triangles without validation.
    #[inline]
    pub fn from_triangles(triangles: Vec<Triangle<T>>) -> Self {
        Self { triangles }
    }
}
