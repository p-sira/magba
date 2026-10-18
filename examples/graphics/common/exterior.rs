//! Exclude magnet interiors using the public sampling and tracing mask.
use magba::{graphics::FieldMask, prelude::*};
use nalgebra::{Matrix3, Point3};

pub trait Interior {
    fn contains(&self, point: Point3<f64>) -> bool;
}

/// Mark excluded samples as gaps and stop streamlines at magnet surfaces.
pub fn exterior<S: Interior>(source: S) -> FieldMask<S, impl Fn(&S, Point3<f64>) -> bool + Clone> {
    FieldMask::new(source, |source: &S, point| source.contains(point))
}

impl Interior for SphereMagnet<f64> {
    fn contains(&self, point: Point3<f64>) -> bool {
        let local = self.pose().as_isometry().inverse() * point;
        local.coords.norm_squared() <= (self.diameter() / 2.0).powi(2)
    }
}
impl Interior for CuboidMagnet<f64> {
    fn contains(&self, point: Point3<f64>) -> bool {
        let local = self.pose().as_isometry().inverse() * point;
        (0..3).all(|i| local[i].abs() <= self.dimensions()[i] / 2.0)
    }
}
impl Interior for CylinderMagnet<f64> {
    fn contains(&self, point: Point3<f64>) -> bool {
        let local = self.pose().as_isometry().inverse() * point;
        local.z.abs() <= self.height() / 2.0
            && local.x.powi(2) + local.y.powi(2) <= (self.diameter() / 2.0).powi(2)
    }
}
impl Interior for TetrahedronMagnet<f64> {
    fn contains(&self, point: Point3<f64>) -> bool {
        let local = self.pose().as_isometry().inverse() * point;
        let v = self.vertices();
        Matrix3::from_columns(&[v[1] - v[0], v[2] - v[0], v[3] - v[0]])
            .try_inverse()
            .is_some_and(|inverse| {
                let barycentric = inverse * (local.coords - v[0]);
                barycentric.iter().all(|&w| w >= 0.0) && barycentric.sum() <= 1.0
            })
    }
}
impl Interior for Magnet<f64> {
    fn contains(&self, point: Point3<f64>) -> bool {
        match self {
            Self::Sphere(m) => m.contains(point),
            Self::Cuboid(m) => m.contains(point),
            Self::Cylinder(m) => m.contains(point),
            Self::Tetrahedron(m) => m.contains(point),
            // These examples use no mesh magnets; dipoles and triangles enclose no volume.
            _ => false,
        }
    }
}
impl Interior for SourceComponent<f64> {
    fn contains(&self, point: Point3<f64>) -> bool {
        match self {
            Self::Magnet(m) => m.contains(point),
            Self::Assembly(a) => a.contains(point),
            _ => false,
        }
    }
}
impl Interior for SourceAssembly<f64> {
    fn contains(&self, point: Point3<f64>) -> bool {
        self.components().any(|source| source.contains(point))
    }
}
