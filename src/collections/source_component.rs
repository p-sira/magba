/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use enum_dispatch::enum_dispatch;

use crate::{
    base::{Float, Pose, Source, Transform},
    collections::{SourceArray, SourceAssembly},
    currents::{CircularCurrent, Current, PathCurrent, TriangleCurrent},
    magnets::{
        CuboidMagnet, CylinderMagnet, Dipole, Magnet, SphereMagnet, TetrahedronMagnet,
        TriangleMagnet,
    },
};

#[cfg(feature = "mesh")]
use crate::{currents::SheetCurrent, magnets::MeshMagnet};
use nalgebra::{Point3, Vector3};

#[derive(Debug, Clone)]
#[enum_dispatch(Source<T>, Transform<T>,)]
/// [Source] components that can be grouped into collections.
///
/// ```
/// # use magba::sources;
/// # use magba::prelude::*;
/// let magnet: SourceComponent = CylinderMagnet::default().into();
/// let sources = sources!(magnet);
/// ```
pub enum SourceComponent<T: Float = f64> {
    Magnet(Magnet<T>),
    Current(Current<T>),
    Assembly(SourceAssembly<T>),
    Custom(Box<dyn Source<T>>),
}

impl<T: Float> PartialEq for SourceComponent<T> {
    /// ```
    /// # use magba::prelude::*;
    /// # use magba::sources;
    /// let cylinder = CylinderMagnet::default();
    /// let cylinder2 = CylinderMagnet::default().with_polarization([1.0, 2.0, 3.0]);
    /// assert_eq!(cylinder, cylinder.clone());
    /// assert_ne!(cylinder, cylinder2);
    ///
    /// let current = CircularCurrent::default();
    /// let current2 = CircularCurrent::<f64>::default().with_current(2.0);
    /// assert_eq!(current, current.clone());
    /// assert_ne!(current, current2);
    ///
    /// let assembly = sources!(cylinder, current);
    /// let assembly2 = sources!(cylinder2, current2);
    /// assert_eq!(assembly, assembly.clone());
    /// assert_ne!(assembly, assembly2);
    ///
    /// // Custom types always return unequal.
    /// let custom = SourceComponent::<f64>::Custom(Box::new(CuboidMagnet::default()));
    /// assert_ne!(custom, custom.clone());
    /// ```
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (Self::Magnet(l0), Self::Magnet(r0)) => l0 == r0,
            (Self::Current(l0), Self::Current(r0)) => l0 == r0,
            (Self::Assembly(l0), Self::Assembly(r0)) => l0 == r0,
            (Self::Custom(_), Self::Custom(_)) => false,
            _ => false,
        }
    }
}

macro_rules! impl_transitive_from_magnet {
    ($($primitive:ident),*) => {
        $(
            impl<T: Float> From<$primitive<T>> for SourceComponent<T> {
                fn from(p: $primitive<T>) -> Self {
                    let intermediate: Magnet<T> = p.into();
                    intermediate.into()
                }
            }
        )*
    };
}

macro_rules! impl_transitive_from_current {
    ($($primitive:ident),*) => {
        $(
            impl<T: Float> From<$primitive<T>> for SourceComponent<T> {
                fn from(p: $primitive<T>) -> Self {
                    let intermediate: Current<T> = p.into();
                    intermediate.into()
                }
            }
        )*
    };
}

impl_transitive_from_magnet!(
    CylinderMagnet,
    CuboidMagnet,
    Dipole,
    SphereMagnet,
    TriangleMagnet,
    TetrahedronMagnet
);

#[cfg(feature = "mesh")]
impl_transitive_from_magnet!(MeshMagnet);

impl_transitive_from_current!(CircularCurrent, PathCurrent, TriangleCurrent);

#[cfg(feature = "mesh")]
impl_transitive_from_current!(SheetCurrent);

impl<T: Float> Eq for SourceComponent<T> {}

impl<S: Source<T>, const N: usize, T: Float> From<SourceArray<S, N, T>> for SourceComponent<T>
where
    SourceComponent<T>: From<S>,
{
    fn from(value: SourceArray<S, N, T>) -> Self {
        SourceAssembly::from(value).into()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::currents::CircularCurrent;
    use crate::magnets::Dipole;

    #[test]
    fn test_source_component_eq_and_from() {
        let current: SourceComponent = CircularCurrent::default().into();
        let current2: SourceComponent = CircularCurrent::default().into();
        assert_eq!(current, current2);

        let magnet: SourceComponent = Dipole::<f64>::default().into();
        assert_ne!(current, magnet);

        let custom1: SourceComponent = SourceComponent::Custom(Box::new(Dipole::<f64>::default()));
        let custom2: SourceComponent = SourceComponent::Custom(Box::new(Dipole::<f64>::default()));
        assert_ne!(custom1, custom2);
        assert_ne!(magnet, custom1);

        let arr_comp: SourceComponent = SourceArray::from([Dipole::<f64>::default()]).into();
        assert_eq!(arr_comp, arr_comp.clone());
    }

    #[test]
    fn relative_complexity_dispatches_through_enums() {
        let path = PathCurrent::default().with_vertices(vec![
            Vector3::zeros(),
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
        ]);
        let current: Current = path.into();
        assert_eq!(current.relative_complexity(), 6);

        let component: SourceComponent = current.into();
        assert_eq!(component.relative_complexity(), 6);

        let magnet: Magnet = Dipole::<f64>::default().into();
        assert_eq!(magnet.relative_complexity(), 1);
    }

    #[cfg(feature = "mesh")]
    #[test]
    fn mesh_complexity_tracks_current_topology() {
        use crate::base::mesh::TriMesh;

        let vertices: Vec<Vector3<f64>> = vec![
            Vector3::zeros(),
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
        ];
        let faces = vec![[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2]];
        let mesh = TriMesh::new_unchecked(vertices, faces);

        let magnet = MeshMagnet::default().with_mesh(mesh.clone());
        assert_eq!(magnet.relative_complexity(), 16);

        let sheet = SheetCurrent::default().with_mesh(mesh);
        assert_eq!(sheet.relative_complexity(), 24);
    }
}
