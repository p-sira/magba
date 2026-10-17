//! Skip magnet interiors before evaluating fields in the plotting examples.
use magba::{
    base::Pose,
    graphics::{Graphics, MagnetColorScheme, plotly::Plot},
    prelude::*,
};
use nalgebra::{Matrix3, Point3, Vector3};

pub trait Interior {
    fn contains(&self, point: Point3<f64>) -> bool;
}

/// Mark excluded samples as gaps and stop streamlines at magnet surfaces.
#[derive(Clone)]
pub struct Exterior<S>(pub S);

impl<S: Transform<f64>> Transform<f64> for Exterior<S> {
    fn pose(&self) -> &Pose {
        self.0.pose()
    }
    fn pose_mut(&mut self) -> &mut Pose {
        self.0.pose_mut()
    }
    fn set_pose(&mut self, pose: Pose) {
        self.0.set_pose(pose);
    }
}

impl<S: Source<f64> + Interior + Clone> Source<f64> for Exterior<S> {
    fn relative_complexity(&self) -> usize {
        self.0.relative_complexity()
    }

    fn compute_B(&self, point: Point3<f64>) -> Vector3<f64> {
        if self.0.contains(point) {
            Vector3::repeat(f64::NAN)
        } else {
            self.0.compute_B(point)
        }
    }

    fn compute_B_batch(&self, points: &[Point3<f64>]) -> Vec<Vector3<f64>> {
        let exterior: Vec<_> = points
            .iter()
            .enumerate()
            .filter(|(_, p)| !self.0.contains(**p))
            .collect();
        let samples: Vec<_> = exterior.iter().map(|(_, p)| **p).collect();
        let mut fields = vec![Vector3::repeat(f64::NAN); points.len()];
        if !samples.is_empty() {
            for ((index, _), field) in exterior.into_iter().zip(self.0.compute_B_batch(&samples)) {
                fields[index] = field;
            }
        }
        fields
    }
}

impl<S: Graphics> Graphics for Exterior<S> {
    fn add_to_plot(&self, plot: &mut Plot) {
        self.0.add_to_plot(plot);
    }
    fn add_to_plot_with_color_scheme(&self, plot: &mut Plot, scheme: MagnetColorScheme) {
        self.0.add_to_plot_with_color_scheme(plot, scheme);
    }
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
