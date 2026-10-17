/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2026 Sira Pornsiriprasert <code@psira.me>
 */

//! Interactive 3D visualization, enabled by the `graphics-plotly` feature.
//!
//! Scenes use SI units and preserve the relative scale of all three axes.
//! HTML output embeds Plotly.js and can be viewed offline. Objects are snapshots;
//! regenerate the plot after changing their poses or geometry.
//!
//! ```no_run
//! use magba::prelude::*;
//!
//! let magnet = CuboidMagnet::<f64>::default();
//! let mut plot = magnet.plot();
//! CircularCurrent::<f64>::default()
//!     .with_position([0.0, 0.0, 2.0])
//!     .add_to_plot(&mut plot);
//! std::fs::write("scene.html", plot.to_html())?;
//! # Ok::<(), std::io::Error>(())
//! ```
//!
//! The re-exported Plotly API allows further layout and trace customization.
//! Sensors, dipoles, and custom collection components appear as point markers,
//! since they have no physical dimensions. Implement [`Graphics`] for custom
//! types to supply their own geometry when adding them directly to a plot.
//!
//! # Field plots
//!
//! Every source implements [`FieldGraphics`], providing 2D heatmaps, 2D/3D quivers,
//! and field lines. Sample a [`FieldGrid`] once to reuse its [`FieldSlice`]:
//!
//! ```no_run
//! use magba::prelude::*;
//! use magba::graphics::{FieldComponent, FieldGrid, FieldLineOptions, Plane};
//!
//! let magnet = SphereMagnet::<f64>::default().with_diameter(0.02);
//! let grid = FieldGrid::new(Plane::XZ, 0.0, [-0.04, 0.04], [-0.04, 0.04], [101, 101])?;
//! let mut plot = magnet.plot_field_2d(&grid, FieldComponent::Magnitude)?;
//! magnet.add_field_lines_2d(&mut plot, &grid, &[[0.015, 0.0]], FieldLineOptions {
//!     step_size: 0.0003, max_steps: 800, ..Default::default()
//! })?;
//! std::fs::write("field.html", plot.to_html())?;
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```

pub use super::{
    FieldComponent, FieldGrid, FieldLineOptions, FieldSampler, FieldSlice, GraphicsError,
    MagnetColorScheme, Plane, QuiverOptions,
};
pub use ::plotly::*;

mod field;
pub use field::FieldGraphics;

use ::plotly::{
    common::Mode,
    layout::{AspectMode, Axis, LayoutScene},
};
use nalgebra::{Point3, Vector3};

use crate::{
    base::{Float, Observer, Source, Transform},
    collections::{
        ObserverArray, ObserverAssembly, ObserverComponent, SourceArray, SourceAssembly,
        SourceComponent,
    },
    currents::{CircularCurrent, Current, PathCurrent, TriangleCurrent},
    magnets::{
        CuboidMagnet, CylinderMagnet, Dipole, Magnet, SphereMagnet, TetrahedronMagnet,
        TriangleMagnet,
    },
    sensors::{
        Sensor,
        hall_effect::{HallLatch, HallSwitch, LinearHallSensor},
    },
};

const SEGMENTS: usize = 48;

/// Geometry that can be added to an interactive Plotly scene.
///
/// Implementations convert local geometry to world coordinates before adding it.
/// Collections already store components in world coordinates, so their parent
/// transform is not applied a second time.
pub trait Graphics<T: Float = f64> {
    /// Append this object's traces without changing the plot's layout.
    fn add_to_plot(&self, plot: &mut Plot);

    /// Append traces using the chosen magnet pole palette.
    ///
    /// Custom implementations can override this to support palette selection.
    fn add_to_plot_with_color_scheme(&self, plot: &mut Plot, _scheme: MagnetColorScheme) {
        self.add_to_plot(plot);
    }

    /// Create a scene using the chosen magnet pole palette, including collections.
    fn plot_with_color_scheme(&self, scheme: MagnetColorScheme) -> Plot {
        let mut plot = scene();
        self.add_to_plot_with_color_scheme(&mut plot, scheme);
        plot
    }

    /// Create an interactive scene with equal axis scales and axes in meters.
    fn plot(&self) -> Plot {
        let mut plot = scene();
        self.add_to_plot(&mut plot);
        plot
    }
}

fn scene() -> Plot {
    let mut plot = Plot::new();
    plot.set_layout(
        Layout::new().scene(
            LayoutScene::new()
                .aspect_mode(AspectMode::Data)
                .x_axis(Axis::new().title("x (m)"))
                .y_axis(Axis::new().title("y (m)"))
                .z_axis(Axis::new().title("z (m)")),
        ),
    );
    plot
}

fn coordinates<T: Float>(object: &impl Transform<T>, vertices: &[Vector3<T>]) -> [Vec<f64>; 3] {
    let mut xyz = [Vec::new(), Vec::new(), Vec::new()];
    for vertex in vertices {
        let p = object.pose().as_isometry() * Point3::from(*vertex);
        for axis in 0..3 {
            xyz[axis].push(p[axis].to_f64().unwrap_or(f64::NAN));
        }
    }
    xyz
}

fn surface<T: Float>(
    plot: &mut Plot,
    object: &impl Transform<T>,
    vertices: &[Vector3<T>],
    faces: &[[usize; 3]],
    name: &str,
    poles: Option<(Vector3<T>, MagnetColorScheme)>,
) {
    if faces.is_empty() {
        return;
    }
    let [x, y, z] = coordinates(object, vertices);
    let mut trace = Mesh3D::new(
        x,
        y,
        z,
        Some(faces.iter().map(|f| f[0]).collect()),
        Some(faces.iter().map(|f| f[1]).collect()),
        Some(faces.iter().map(|f| f[2]).collect()),
    )
    .name(name)
    .opacity(0.65)
    .show_legend(true);
    if let Some((polarization, scheme)) = poles {
        let (north, south) = scheme.poles();
        // Surface charge is proportional to the outward normal dotted with polarization.
        // Neutral faces and the equatorial belt remain grey. Work in local coordinates
        // so rotation and translation cannot change a face's pole color.
        let direction = polarization.try_normalize(T::zero());
        let charges: Vec<_> = faces
            .iter()
            .map(|&[a, b, c]| {
                let normal = (vertices[b] - vertices[a]).cross(&(vertices[c] - vertices[a]));
                direction
                    .and_then(|p| normal.try_normalize(T::zero()).map(|n| n.dot(&p)))
                    .and_then(|q| q.to_f64())
                    .unwrap_or(0.0)
            })
            .collect();
        // Scale each pole separately so sloped polyhedra also show both poles.
        let north_charge = charges.iter().copied().fold(0.0_f64, f64::max);
        let south_charge = charges.iter().copied().fold(0.0_f64, f64::min);
        let colors: Vec<_> = charges
            .into_iter()
            .map(|charge| {
                if charge > north_charge * 0.5 {
                    north
                } else if charge < south_charge * 0.5 {
                    south
                } else {
                    "grey"
                }
            })
            .collect();
        trace = trace.color("grey").face_color(colors);
    }
    plot.add_trace(trace);
}

fn line<T: Float>(
    plot: &mut Plot,
    object: &impl Transform<T>,
    vertices: &[Vector3<T>],
    name: &str,
) {
    if vertices.is_empty() {
        return;
    }
    let [x, y, z] = coordinates(object, vertices);
    plot.add_trace(Scatter3D::new(x, y, z).mode(Mode::Lines).name(name));
}

fn marker<T: Float>(plot: &mut Plot, object: &(impl Transform<T> + ?Sized), name: &str) {
    let p = object.pose().position();
    plot.add_trace(
        Scatter3D::new(
            vec![p.x.to_f64().unwrap_or(f64::NAN)],
            vec![p.y.to_f64().unwrap_or(f64::NAN)],
            vec![p.z.to_f64().unwrap_or(f64::NAN)],
        )
        .mode(Mode::Markers)
        .name(name),
    );
}

fn scalar<T: Float>(n: f64) -> T {
    T::from(n).unwrap()
}

impl<T: Float> Graphics<T> for CuboidMagnet<T> {
    fn add_to_plot(&self, plot: &mut Plot) {
        self.add_to_plot_with_color_scheme(plot, MagnetColorScheme::default());
    }

    fn add_to_plot_with_color_scheme(&self, plot: &mut Plot, scheme: MagnetColorScheme) {
        let half = self.dimensions() / scalar::<T>(2.0);
        let vertices: Vec<_> = [
            (-1., -1., -1.),
            (1., -1., -1.),
            (1., 1., -1.),
            (-1., 1., -1.),
            (-1., -1., 1.),
            (1., -1., 1.),
            (1., 1., 1.),
            (-1., 1., 1.),
        ]
        .into_iter()
        .map(|(x, y, z)| {
            Vector3::new(
                scalar::<T>(x) * half.x,
                scalar::<T>(y) * half.y,
                scalar::<T>(z) * half.z,
            )
        })
        .collect();
        surface(
            plot,
            self,
            &vertices,
            &[
                [0, 2, 1],
                [0, 3, 2],
                [4, 5, 6],
                [4, 6, 7],
                [0, 1, 5],
                [0, 5, 4],
                [1, 2, 6],
                [1, 6, 5],
                [2, 3, 7],
                [2, 7, 6],
                [3, 0, 4],
                [3, 4, 7],
            ],
            "Cuboid magnet",
            Some((self.polarization(), scheme)),
        );
    }
}

fn ring<T: Float>(radius: T, height: T) -> Vec<Vector3<T>> {
    (0..SEGMENTS)
        .map(|i| {
            let angle = std::f64::consts::TAU * i as f64 / SEGMENTS as f64;
            Vector3::new(
                radius * scalar::<T>(angle.cos()),
                radius * scalar::<T>(angle.sin()),
                height,
            )
        })
        .collect()
}

impl<T: Float> Graphics<T> for CylinderMagnet<T> {
    fn add_to_plot(&self, plot: &mut Plot) {
        self.add_to_plot_with_color_scheme(plot, MagnetColorScheme::default());
    }

    fn add_to_plot_with_color_scheme(&self, plot: &mut Plot, scheme: MagnetColorScheme) {
        let half = self.height() / scalar::<T>(2.0);
        let radius = self.diameter() / scalar::<T>(2.0);
        let mut vertices = ring(radius, -half);
        vertices.extend(ring(radius, half));
        vertices.extend([
            Vector3::new(T::zero(), T::zero(), -half),
            Vector3::new(T::zero(), T::zero(), half),
        ]);
        let mut faces = Vec::with_capacity(4 * SEGMENTS);
        for i in 0..SEGMENTS {
            let j = (i + 1) % SEGMENTS;
            faces.extend([
                [i, j, j + SEGMENTS],
                [i, j + SEGMENTS, i + SEGMENTS],
                [2 * SEGMENTS, j, i],
                [2 * SEGMENTS + 1, i + SEGMENTS, j + SEGMENTS],
            ]);
        }
        surface(
            plot,
            self,
            &vertices,
            &faces,
            "Cylinder magnet",
            Some((self.polarization(), scheme)),
        );
    }
}

impl<T: Float> Graphics<T> for SphereMagnet<T> {
    fn add_to_plot(&self, plot: &mut Plot) {
        self.add_to_plot_with_color_scheme(plot, MagnetColorScheme::default());
    }

    fn add_to_plot_with_color_scheme(&self, plot: &mut Plot, scheme: MagnetColorScheme) {
        let radius = self.diameter() / scalar::<T>(2.0);
        let mut vertices = vec![Vector3::new(T::zero(), T::zero(), radius)];
        let latitudes = SEGMENTS / 2;
        for latitude in 1..latitudes {
            let angle = std::f64::consts::PI * latitude as f64 / latitudes as f64;
            vertices.extend(ring(
                radius * scalar::<T>(angle.sin()),
                radius * scalar::<T>(angle.cos()),
            ));
        }
        let south = vertices.len();
        vertices.push(Vector3::new(T::zero(), T::zero(), -radius));
        let mut faces = Vec::new();
        for i in 0..SEGMENTS {
            let j = (i + 1) % SEGMENTS;
            faces.push([0, 1 + i, 1 + j]);
            for latitude in 0..latitudes - 2 {
                let a = 1 + latitude * SEGMENTS;
                let b = a + SEGMENTS;
                faces.extend([[a + i, b + i, b + j], [a + i, b + j, a + j]]);
            }
            let last = 1 + (latitudes - 2) * SEGMENTS;
            faces.push([south, last + j, last + i]);
        }
        surface(
            plot,
            self,
            &vertices,
            &faces,
            "Sphere magnet",
            Some((self.polarization(), scheme)),
        );
    }
}

impl<T: Float> Graphics<T> for TriangleMagnet<T> {
    fn add_to_plot(&self, plot: &mut Plot) {
        self.add_to_plot_with_color_scheme(plot, MagnetColorScheme::default());
    }

    fn add_to_plot_with_color_scheme(&self, plot: &mut Plot, scheme: MagnetColorScheme) {
        surface(
            plot,
            self,
            &self.vertices(),
            &[[0, 1, 2]],
            "Triangle magnet",
            Some((self.polarization(), scheme)),
        );
    }
}
impl<T: Float> Graphics<T> for TetrahedronMagnet<T> {
    fn add_to_plot(&self, plot: &mut Plot) {
        self.add_to_plot_with_color_scheme(plot, MagnetColorScheme::default());
    }

    fn add_to_plot_with_color_scheme(&self, plot: &mut Plot, scheme: MagnetColorScheme) {
        let mut vertices = self.vertices();
        if (vertices[1] - vertices[0])
            .cross(&(vertices[2] - vertices[0]))
            .dot(&(vertices[3] - vertices[0]))
            < T::zero()
        {
            vertices.swap(2, 3);
        }
        surface(
            plot,
            self,
            &vertices,
            &[[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2]],
            "Tetrahedron magnet",
            Some((self.polarization(), scheme)),
        );
    }
}
impl<T: Float> Graphics<T> for CircularCurrent<T> {
    fn add_to_plot(&self, plot: &mut Plot) {
        let mut vertices = ring(self.diameter() / scalar::<T>(2.0), T::zero());
        vertices.push(vertices[0]);
        line(plot, self, &vertices, "Circular current");
    }
}
impl<T: Float> Graphics<T> for PathCurrent<T> {
    fn add_to_plot(&self, plot: &mut Plot) {
        line(plot, self, self.vertices(), "Path current");
    }
}
impl<T: Float> Graphics<T> for TriangleCurrent<T> {
    fn add_to_plot(&self, plot: &mut Plot) {
        let v = self.vertices();
        line(plot, self, &[v[0], v[1], v[2], v[0]], "Triangle current");
    }
}

macro_rules! markers {
    ($($ty:ident => $name:literal),* $(,)?) => {$(
        impl<T: Float> Graphics<T> for $ty<T> {
            fn add_to_plot(&self, plot: &mut Plot) { marker(plot, self, $name); }
        }
    )*};
}
markers!(Dipole => "Dipole", LinearHallSensor => "Linear Hall sensor", HallSwitch => "Hall switch", HallLatch => "Hall latch");

#[cfg(feature = "mesh")]
mod mesh {
    use super::*;
    use crate::{currents::SheetCurrent, magnets::MeshMagnet};

    fn add_mesh<T: Float>(
        plot: &mut Plot,
        object: &impl Transform<T>,
        mesh: &crate::base::mesh::TriMesh<T>,
        name: &str,
        poles: Option<(Vector3<T>, MagnetColorScheme)>,
    ) {
        let vertices: Vec<_> = mesh.triangles().iter().flat_map(|t| t.vertices()).collect();
        let faces: Vec<_> = (0..vertices.len())
            .step_by(3)
            .map(|i| [i, i + 1, i + 2])
            .collect();
        surface(plot, object, &vertices, &faces, name, poles);
    }
    impl<T: Float> Graphics<T> for MeshMagnet<T> {
        fn add_to_plot(&self, plot: &mut Plot) {
            self.add_to_plot_with_color_scheme(plot, MagnetColorScheme::default());
        }

        fn add_to_plot_with_color_scheme(&self, plot: &mut Plot, scheme: MagnetColorScheme) {
            add_mesh(
                plot,
                self,
                self.mesh(),
                "Mesh magnet",
                Some((self.polarization(), scheme)),
            );
        }
    }
    impl<T: Float> Graphics<T> for SheetCurrent<T> {
        fn add_to_plot(&self, plot: &mut Plot) {
            add_mesh(plot, self, self.mesh(), "Sheet current", None);
        }
    }
}

macro_rules! dispatch {
    ($ty:ident { $($variant:ident),* $(,)? } $(, mesh: $mesh:ident)?) => {
        impl<T: Float> Graphics<T> for $ty<T> {
            fn add_to_plot(&self, plot: &mut Plot) {
                self.add_to_plot_with_color_scheme(plot, MagnetColorScheme::default());
            }

            fn add_to_plot_with_color_scheme(&self, plot: &mut Plot, scheme: MagnetColorScheme) {
                match self {
                    $(Self::$variant(object) => object.add_to_plot_with_color_scheme(plot, scheme),)*
                    $(#[cfg(feature = "mesh")] Self::$mesh(object) => object.add_to_plot_with_color_scheme(plot, scheme),)?
                }
            }
        }
    };
}
dispatch!(Magnet { Cylinder, Cuboid, Dipole, Sphere, Tetrahedron, Triangle }, mesh: Mesh);
dispatch!(Current { Circular, Path, Triangle }, mesh: Sheet);
dispatch!(Sensor {
    LinearHall,
    HallSwitch,
    HallLatch
});

impl<T: Float> Graphics<T> for SourceComponent<T> {
    fn add_to_plot(&self, plot: &mut Plot) {
        self.add_to_plot_with_color_scheme(plot, MagnetColorScheme::default());
    }

    fn add_to_plot_with_color_scheme(&self, plot: &mut Plot, scheme: MagnetColorScheme) {
        match self {
            Self::Magnet(object) => object.add_to_plot_with_color_scheme(plot, scheme),
            Self::Current(object) => object.add_to_plot_with_color_scheme(plot, scheme),
            Self::Assembly(object) => object.add_to_plot_with_color_scheme(plot, scheme),
            Self::Custom(object) => marker(plot, object.as_ref(), "Custom source"),
        }
    }
}
impl<T: Float> Graphics<T> for ObserverComponent<T> {
    fn add_to_plot(&self, plot: &mut Plot) {
        self.add_to_plot_with_color_scheme(plot, MagnetColorScheme::default());
    }

    fn add_to_plot_with_color_scheme(&self, plot: &mut Plot, scheme: MagnetColorScheme) {
        match self {
            Self::Sensor(object) => object.add_to_plot_with_color_scheme(plot, scheme),
            Self::Custom(object) => marker(plot, object.as_ref(), "Custom observer"),
        }
    }
}
impl<T: Float> Graphics<T> for SourceAssembly<T> {
    fn add_to_plot(&self, plot: &mut Plot) {
        self.add_to_plot_with_color_scheme(plot, MagnetColorScheme::default());
    }

    fn add_to_plot_with_color_scheme(&self, plot: &mut Plot, scheme: MagnetColorScheme) {
        for object in self.components() {
            object.add_to_plot_with_color_scheme(plot, scheme);
        }
    }
}
impl<T: Float> Graphics<T> for ObserverAssembly<T> {
    fn add_to_plot(&self, plot: &mut Plot) {
        self.add_to_plot_with_color_scheme(plot, MagnetColorScheme::default());
    }

    fn add_to_plot_with_color_scheme(&self, plot: &mut Plot, scheme: MagnetColorScheme) {
        for object in self.components() {
            object.add_to_plot_with_color_scheme(plot, scheme);
        }
    }
}
impl<T: Float, S: Source<T> + Graphics<T>, const N: usize> Graphics<T> for SourceArray<S, N, T> {
    fn add_to_plot(&self, plot: &mut Plot) {
        self.add_to_plot_with_color_scheme(plot, MagnetColorScheme::default());
    }

    fn add_to_plot_with_color_scheme(&self, plot: &mut Plot, scheme: MagnetColorScheme) {
        for object in self.components() {
            object.add_to_plot_with_color_scheme(plot, scheme);
        }
    }
}
impl<T: Float, S: Observer<T> + Graphics<T>, const N: usize> Graphics<T>
    for ObserverArray<S, N, T>
{
    fn add_to_plot(&self, plot: &mut Plot) {
        self.add_to_plot_with_color_scheme(plot, MagnetColorScheme::default());
    }

    fn add_to_plot_with_color_scheme(&self, plot: &mut Plot, scheme: MagnetColorScheme) {
        for object in self.components() {
            object.add_to_plot_with_color_scheme(plot, scheme);
        }
    }
}
