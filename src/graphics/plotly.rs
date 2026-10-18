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
//! ![Cuboid magnet and circular current scene](https://github.com/p-sira/magba/blob/main/examples/figures/rustdoc_scene.png?raw=true)
//!
//! The re-exported Plotly API allows further layout and trace customization.
//! Sensors, dipoles, and ordinary custom collection components appear as markers.
//! Use [`SourceComponent::custom_graphics`] or [`ObserverComponent::custom_graphics`]
//! to retain custom [`Graphics`] implementations inside collections.
//!
//! # Appearance and borrowed views
//!
//! Magnet surfaces are opaque by default. Pole colors follow local polarization
//! and rotate with the magnet. Colors fade along the polarization direction
//! through a grey central belt, continuously across face boundaries. The gradient
//! indicates local polarity. A planar magnet polarized
//! perpendicular to its surface retains a uniform pole color.
//! [`MagnetColorScheme::Din`] uses red north and green south according to
//! DIN SPEC 91411. The alternatives are [`MagnetColorScheme::OkabeIto`]
//! (orange/blue) and [`MagnetColorScheme::Trad`] (red/blue).
//!
//! [`GraphicsOptions`] provides getters, mutable setters such as
//! [`GraphicsOptions::set_opacity`], and consuming builders such as
//! [`GraphicsOptions::with_opacity`]. Opacity setters accept a plain [`f64`] and override surfaces,
//! lines, and markers; the default preserves each renderer's usual opacity.
//! Setting visibility to false emits no traces. Segment counts control circular
//! geometry, with half as many latitude subdivisions for spheres. Rendering
//! panics if opacity is nonfinite or outside `[0, 1]`, or segments is below six.
//!
//! ```
//! use magba::{graphics::{GraphicsOptions, GraphicsView, MagnetColorScheme}, prelude::*};
//! let magnet = CylinderMagnet::<f64>::default();
//! let options = GraphicsOptions::default()
//!     .with_color_scheme(MagnetColorScheme::Din)
//!     .with_opacity(0.5)
//!     .with_segments(96);
//! let plot = magnet.plot_with_options(&options);
//! let view = GraphicsView::new(&magnet).with_options(options);
//! let plot = view.plot();
//! ```
//!
//! ![Cylinder with DIN pole colors and half opacity](https://github.com/p-sira/magba/blob/main/examples/figures/rustdoc_appearance.png?raw=true)
//!
//! Options apply to geometry and propagate through collections. Views borrow an
//! object and store its appearance; recreate them after modifying the object.
//! Rendering validates options even when visibility is disabled. Field plots
//! retain their separate options.
//!
//! # Custom collection renderers
//!
//! Implement [`Graphics`] alongside [`Source`] or [`Observer`], then construct
//! components with [`SourceComponent::custom_graphics`] or
//! [`ObserverComponent::custom_graphics`]. These retain custom rendering through
//! cloning and collection transforms. Ordinary [`SourceComponent::Custom`] and [`ObserverComponent::Custom`] components use markers.
//! In [`Graphics::add_to_plot_with_options`], call [`GraphicsOptions::validate`] and honor
//! [`GraphicsOptions::visible`] before emitting traces. The core physics traits
//! require no rendering methods.
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
//! let grid = FieldGrid::new(Plane::XZ, 0.02, [-0.04, 0.04], [-0.04, 0.04], [101, 101])?;
//! let mut plot = magnet.plot_field_2d(&grid, FieldComponent::Magnitude)?;
//! magnet.add_field_lines_2d(&mut plot, &grid, &[[0.015, 0.0]], FieldLineOptions {
//!     step_size: 0.0003, max_steps: 800, ..Default::default()
//! })?;
//! std::fs::write("field.html", plot.to_html())?;
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```
//!
//! ![Sphere field magnitude and a projected field line](https://github.com/p-sira/magba/blob/main/examples/figures/rustdoc_field_lines.png?raw=true)
//!
//! Reuse sampled fields for heatmaps and arrows without resampling:
//!
//! ```no_run
//! use magba::{prelude::*, graphics::{FieldComponent, FieldGrid, Plane, QuiverOptions}};
//! let magnet = SphereMagnet::<f64>::default().with_diameter(0.02);
//! let grid = FieldGrid::new(Plane::XZ, 0.02, [-0.04, 0.04], [-0.04, 0.04], [21, 21])?;
//! let slice = magnet.sample_field(&grid)?;
//! let mut plot = slice.heatmap(FieldComponent::Magnitude);
//! slice.add_quiver(&mut plot, QuiverOptions { scale: 0.002, normalize: true })?;
//! std::fs::write("field.html", plot.to_html())?;
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```
//!
//! ![Sphere field magnitude with in-plane arrows](https://github.com/p-sira/magba/blob/main/examples/figures/rustdoc_field_quiver.png?raw=true)
//!
//! Use [`FieldGraphics::plot_field_2d`] and [`FieldGraphics::plot_quiver_2d`] for
//! direct plotting. [`FieldGraphics::plot_quiver`] and [`FieldGraphics::add_quiver`]
//! draw 3D arrows. [`FieldGraphics::plot_field_lines`] and
//! [`FieldGraphics::add_field_lines`] trace 3D field lines with direction arrows;
//! [`FieldGraphics::plot_field_lines_2d`] and [`FieldGraphics::add_field_lines_2d`]
//! trace the field projected into the grid's plane.
//! [`FieldSampler::trace_field_lines`] returns coordinates for further analysis.
//! Sources may use [`f32`] or [`f64`]; plotting coordinates use [`f64`] meters.
//!
//! The runnable examples mask magnet interiors before field evaluation: heatmaps
//! show gaps, interior arrows are omitted, and exterior lines stop at surfaces.
//! This masking is an example-specific choice, separate from the plotting API.

pub use super::{
    FieldComponent, FieldGrid, FieldLineOptions, FieldSampler, FieldSlice, GraphicsError,
    GraphicsOptions, GraphicsView, MagnetColorScheme, Plane, QuiverOptions,
};
pub use ::plotly::*;

mod field;
pub use field::FieldGraphics;

use ::plotly::{
    common::{ColorScale, ColorScaleElement, Line, Mode},
    layout::{AspectMode, Axis, LayoutScene},
    mesh3d::IntensityMode,
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

mod custom;
pub use custom::{GraphicsObserver, GraphicsSource};

/// Geometry that can be added to an interactive Plotly scene.
///
/// Implementations convert local geometry to world coordinates. Collections already
/// store world coordinates; do not apply their parent transform a second time.
/// Custom renderers must validate options and honor visibility before adding traces.
pub trait Graphics<T: Float = f64> {
    /// Append traces using default options, preserving the plot layout.
    fn add_to_plot(&self, plot: &mut Plot) {
        self.add_to_plot_with_options(plot, &GraphicsOptions::default());
    }

    /// Append traces with the supplied appearance, preserving the plot layout.
    ///
    /// # Panics
    /// Panics if opacity is nonfinite or outside `[0, 1]`, or segments is below six.
    fn add_to_plot_with_options(&self, plot: &mut Plot, options: &GraphicsOptions);

    /// Create a scene with the supplied appearance and equal axis scales in meters.
    fn plot_with_options(&self, options: &GraphicsOptions) -> Plot {
        options.validate();
        let mut plot = scene();
        if options.visible() {
            self.add_to_plot_with_options(&mut plot, options);
        }
        plot
    }

    /// Create an interactive scene using default appearance.
    fn plot(&self) -> Plot {
        self.plot_with_options(&GraphicsOptions::default())
    }
}

impl<S: ?Sized> GraphicsView<'_, S> {
    /// Render a Plotly scene using this view's stored options.
    ///
    /// Available with `graphics-plotly` for objects implementing [`Graphics`].
    pub fn plot<T: Float>(&self) -> Plot
    where
        S: Graphics<T>,
    {
        self.object().plot_with_options(self.options())
    }

    /// Append Plotly traces using stored options, preserving the plot layout.
    ///
    /// Available with `graphics-plotly` for objects implementing [`Graphics`].
    pub fn add_to_plot<T: Float>(&self, plot: &mut Plot)
    where
        S: Graphics<T>,
    {
        self.options().validate();
        if self.options().visible() {
            self.object().add_to_plot_with_options(plot, self.options());
        }
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

struct SurfacePoles<T: Float> {
    polarization: Vector3<T>,
    scheme: MagnetColorScheme,
}

fn surface<T: Float>(
    plot: &mut Plot,
    object: &impl Transform<T>,
    vertices: &[Vector3<T>],
    faces: &[[usize; 3]],
    name: &str,
    poles: Option<SurfacePoles<T>>,
    options: &GraphicsOptions,
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
    .opacity(
        options
            .opacity()
            .unwrap_or(if poles.is_some() { 1.0 } else { 0.65 }),
    )
    .show_legend(true);
    if let Some(SurfacePoles {
        polarization,
        scheme,
    }) = poles
    {
        let (mode, intensity) = pole_intensity(vertices, faces, polarization);
        trace = trace
            .color("grey")
            .intensity(intensity)
            .intensity_mode(mode)
            .c_min(-1.0)
            .c_max(1.0)
            .color_scale(pole_colorscale(scheme))
            .show_scale(false)
            .hover_template("x: %{x} m<br>y: %{y} m<br>z: %{z} m<extra>%{fullData.name}</extra>");
    }
    plot.add_trace(trace);
}

/// Position along polarization supplies a continuous gradient across face edges.
/// Plotly interpolates vertex intensities before applying the shared colorscale.
fn pole_intensity<T: Float>(
    vertices: &[Vector3<T>],
    faces: &[[usize; 3]],
    polarization: Vector3<T>,
) -> (IntensityMode, Vec<f64>) {
    let magnitude = polarization
        .iter()
        .copied()
        .map(num_traits::Float::abs)
        .fold(T::zero(), num_traits::Float::max);
    let direction = if magnitude > T::zero() && num_traits::Float::is_finite(magnitude) {
        (polarization / magnitude).try_normalize(T::zero())
    } else {
        None
    };
    let Some(direction) = direction else {
        return (IntensityMode::Vertex, vec![0.0; vertices.len()]);
    };
    let projections: Vec<_> = vertices
        .iter()
        .map(|v| direction.dot(&(*v - vertices[0])).to_f64().unwrap_or(0.0))
        .collect();
    let min = projections.iter().copied().fold(f64::INFINITY, f64::min);
    let max = projections
        .iter()
        .copied()
        .fold(f64::NEG_INFINITY, f64::max);
    let extent = vertices
        .iter()
        .map(|v| (*v - vertices[0]).norm().to_f64().unwrap_or(0.0))
        .fold(0.0_f64, f64::max);
    let tolerance = 16.0 * <T as num_traits::Float>::epsilon().to_f64().unwrap_or(0.0) * extent;
    if max - min > tolerance {
        return (
            IntensityMode::Vertex,
            projections
                .into_iter()
                .map(|p| (2.0 * ((p - min) / (max - min)) - 1.0).clamp(-1.0, 1.0))
                .collect(),
        );
    }
    // A planar charged sheet has no extent along its normal. Retain its uniform
    // pole color using the face normal.
    let intensity = faces
        .iter()
        .map(|&[a, b, c]| {
            (vertices[b] - vertices[a])
                .cross(&(vertices[c] - vertices[a]))
                .try_normalize(T::zero())
                .and_then(|normal| direction.dot(&normal).to_f64())
                .unwrap_or(0.0)
        })
        .collect();
    (IntensityMode::Cell, intensity)
}

fn pole_colorscale(scheme: MagnetColorScheme) -> ColorScale {
    let [north, south] = scheme.poles();
    let stops = [
        -1.0_f64, -0.75, -0.5, -0.25, -0.15, 0.0, 0.15, 0.25, 0.5, 0.75, 1.0,
    ]
    .into_iter()
    .map(|level| {
        let (color, rgb) = if level > 0.0 { north } else { south };
        let strength = ((level.abs() - 0.15) / 0.85).clamp(0.0, 1.0);
        let strength = strength * strength * (3.0 - 2.0 * strength);
        let blended =
            rgb.map(|channel| (128.0 + (f64::from(channel) - 128.0) * strength).round() as u8);
        let color = if strength == 0.0 {
            "grey".to_owned()
        } else if strength == 1.0 {
            color.to_owned()
        } else {
            format!("rgb({},{},{})", blended[0], blended[1], blended[2])
        };
        ColorScaleElement((level + 1.0) / 2.0, color)
    })
    .collect();
    ColorScale::Vector(stops)
}

fn line<T: Float>(
    plot: &mut Plot,
    object: &impl Transform<T>,
    vertices: &[Vector3<T>],
    name: &str,
    options: &GraphicsOptions,
) {
    if vertices.is_empty() {
        return;
    }
    let [x, y, z] = coordinates(object, vertices);
    plot.add_trace(
        Scatter3D::new(x, y, z)
            .opacity(options.opacity().unwrap_or(1.0))
            .mode(Mode::Lines)
            .line(Line::new().width(6.0))
            .name(name),
    );
}

fn marker<T: Float>(
    plot: &mut Plot,
    object: &(impl Transform<T> + ?Sized),
    name: &str,
    options: &GraphicsOptions,
) {
    let p = object.pose().position();
    plot.add_trace(
        Scatter3D::new(
            vec![p.x.to_f64().unwrap_or(f64::NAN)],
            vec![p.y.to_f64().unwrap_or(f64::NAN)],
            vec![p.z.to_f64().unwrap_or(f64::NAN)],
        )
        .opacity(options.opacity().unwrap_or(1.0))
        .mode(Mode::Markers)
        .name(name),
    );
}

fn scalar<T: Float>(n: f64) -> T {
    T::from(n).unwrap()
}

impl<T: Float> Graphics<T> for CuboidMagnet<T> {
    fn add_to_plot_with_options(&self, plot: &mut Plot, options: &GraphicsOptions) {
        options.validate();
        if !options.visible() {
            return;
        }
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
            Some(SurfacePoles {
                polarization: self.polarization(),
                scheme: options.color_scheme(),
            }),
            options,
        );
    }
}

fn ring<T: Float>(radius: T, height: T, segments: usize) -> Vec<Vector3<T>> {
    (0..segments)
        .map(|i| {
            let angle = std::f64::consts::TAU * i as f64 / segments as f64;
            Vector3::new(
                radius * scalar::<T>(angle.cos()),
                radius * scalar::<T>(angle.sin()),
                height,
            )
        })
        .collect()
}

impl<T: Float> Graphics<T> for CylinderMagnet<T> {
    fn add_to_plot_with_options(&self, plot: &mut Plot, options: &GraphicsOptions) {
        options.validate();
        if !options.visible() {
            return;
        }
        let half = self.height() / scalar::<T>(2.0);
        let radius = self.diameter() / scalar::<T>(2.0);
        let mut vertices = ring(radius, -half, options.segments());
        vertices.extend(ring(radius, half, options.segments()));
        vertices.extend([
            Vector3::new(T::zero(), T::zero(), -half),
            Vector3::new(T::zero(), T::zero(), half),
        ]);
        let mut faces = Vec::with_capacity(4 * options.segments());
        for i in 0..options.segments() {
            let j = (i + 1) % options.segments();
            faces.extend([
                [i, j, j + options.segments()],
                [i, j + options.segments(), i + options.segments()],
                [2 * options.segments(), j, i],
                [
                    2 * options.segments() + 1,
                    i + options.segments(),
                    j + options.segments(),
                ],
            ]);
        }
        surface(
            plot,
            self,
            &vertices,
            &faces,
            "Cylinder magnet",
            Some(SurfacePoles {
                polarization: self.polarization(),
                scheme: options.color_scheme(),
            }),
            options,
        );
    }
}

impl<T: Float> Graphics<T> for SphereMagnet<T> {
    fn add_to_plot_with_options(&self, plot: &mut Plot, options: &GraphicsOptions) {
        options.validate();
        if !options.visible() {
            return;
        }
        let radius = self.diameter() / scalar::<T>(2.0);
        let mut vertices = vec![Vector3::new(T::zero(), T::zero(), radius)];
        let latitudes = options.segments() / 2;
        for latitude in 1..latitudes {
            let angle = std::f64::consts::PI * latitude as f64 / latitudes as f64;
            vertices.extend(ring(
                radius * scalar::<T>(angle.sin()),
                radius * scalar::<T>(angle.cos()),
                options.segments(),
            ));
        }
        let south = vertices.len();
        vertices.push(Vector3::new(T::zero(), T::zero(), -radius));
        let mut faces = Vec::new();
        for i in 0..options.segments() {
            let j = (i + 1) % options.segments();
            faces.push([0, 1 + i, 1 + j]);
            for latitude in 0..latitudes - 2 {
                let a = 1 + latitude * options.segments();
                let b = a + options.segments();
                faces.extend([[a + i, b + i, b + j], [a + i, b + j, a + j]]);
            }
            let last = 1 + (latitudes - 2) * options.segments();
            faces.push([south, last + j, last + i]);
        }
        surface(
            plot,
            self,
            &vertices,
            &faces,
            "Sphere magnet",
            Some(SurfacePoles {
                polarization: self.polarization(),
                scheme: options.color_scheme(),
            }),
            options,
        );
    }
}

impl<T: Float> Graphics<T> for TriangleMagnet<T> {
    fn add_to_plot_with_options(&self, plot: &mut Plot, options: &GraphicsOptions) {
        options.validate();
        if !options.visible() {
            return;
        }
        surface(
            plot,
            self,
            &self.vertices(),
            &[[0, 1, 2]],
            "Triangle magnet",
            Some(SurfacePoles {
                polarization: self.polarization(),
                scheme: options.color_scheme(),
            }),
            options,
        );
    }
}
impl<T: Float> Graphics<T> for TetrahedronMagnet<T> {
    fn add_to_plot_with_options(&self, plot: &mut Plot, options: &GraphicsOptions) {
        options.validate();
        if !options.visible() {
            return;
        }
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
            Some(SurfacePoles {
                polarization: self.polarization(),
                scheme: options.color_scheme(),
            }),
            options,
        );
    }
}
impl<T: Float> Graphics<T> for CircularCurrent<T> {
    fn add_to_plot_with_options(&self, plot: &mut Plot, options: &GraphicsOptions) {
        options.validate();
        if !options.visible() {
            return;
        }
        let mut vertices = ring(
            self.diameter() / scalar::<T>(2.0),
            T::zero(),
            options.segments(),
        );
        vertices.push(vertices[0]);
        line(plot, self, &vertices, "Circular current", options);
    }
}
impl<T: Float> Graphics<T> for PathCurrent<T> {
    fn add_to_plot_with_options(&self, plot: &mut Plot, options: &GraphicsOptions) {
        options.validate();
        if !options.visible() {
            return;
        }
        line(plot, self, self.vertices(), "Path current", options);
    }
}
impl<T: Float> Graphics<T> for TriangleCurrent<T> {
    fn add_to_plot_with_options(&self, plot: &mut Plot, options: &GraphicsOptions) {
        options.validate();
        if !options.visible() {
            return;
        }
        let v = self.vertices();
        line(
            plot,
            self,
            &[v[0], v[1], v[2], v[0]],
            "Triangle current",
            options,
        );
    }
}

macro_rules! markers {
    ($($ty:ident => $name:literal),* $(,)?) => {$(
        impl<T: Float> Graphics<T> for $ty<T> {
            fn add_to_plot_with_options(&self, plot: &mut Plot, options: &GraphicsOptions) {
                options.validate();
                if !options.visible() {
                    return;
                }
                marker(plot, self, $name, options);
            }
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
        poles: Option<Vector3<T>>,
        options: &GraphicsOptions,
    ) {
        let vertices: Vec<_> = mesh.triangles().iter().flat_map(|t| t.vertices()).collect();
        let faces: Vec<_> = (0..vertices.len())
            .step_by(3)
            .map(|i| [i, i + 1, i + 2])
            .collect();
        surface(
            plot,
            object,
            &vertices,
            &faces,
            name,
            poles.map(|polarization| SurfacePoles {
                polarization,
                scheme: options.color_scheme(),
            }),
            options,
        );
    }
    impl<T: Float> Graphics<T> for MeshMagnet<T> {
        fn add_to_plot_with_options(&self, plot: &mut Plot, options: &GraphicsOptions) {
            options.validate();
            if !options.visible() {
                return;
            }
            add_mesh(
                plot,
                self,
                self.mesh(),
                "Mesh magnet",
                Some(self.polarization()),
                options,
            );
        }
    }
    impl<T: Float> Graphics<T> for SheetCurrent<T> {
        fn add_to_plot_with_options(&self, plot: &mut Plot, options: &GraphicsOptions) {
            options.validate();
            if !options.visible() {
                return;
            }
            add_mesh(plot, self, self.mesh(), "Sheet current", None, options);
        }
    }
}

macro_rules! dispatch {
    ($ty:ident { $($variant:ident),* $(,)? } $(, mesh: $mesh:ident)?) => {
        impl<T: Float> Graphics<T> for $ty<T> {
            fn add_to_plot_with_options(&self, plot: &mut Plot, options: &GraphicsOptions) {
                options.validate();
                if !options.visible() {
                    return;
                }
                match self {
                    $(Self::$variant(object) => object.add_to_plot_with_options(plot, options),)*
                    $(#[cfg(feature = "mesh")] Self::$mesh(object) => object.add_to_plot_with_options(plot, options),)?
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
    fn add_to_plot_with_options(&self, plot: &mut Plot, options: &GraphicsOptions) {
        options.validate();
        if !options.visible() {
            return;
        }
        match self {
            Self::Magnet(object) => object.add_to_plot_with_options(plot, options),
            Self::Current(object) => object.add_to_plot_with_options(plot, options),
            Self::Assembly(object) => object.add_to_plot_with_options(plot, options),
            Self::Custom(object) => marker(plot, object.as_ref(), "Custom source", options),
            Self::CustomGraphics(object) => object.add_to_plot_with_options(plot, options),
        }
    }
}
impl<T: Float> Graphics<T> for ObserverComponent<T> {
    fn add_to_plot_with_options(&self, plot: &mut Plot, options: &GraphicsOptions) {
        options.validate();
        if !options.visible() {
            return;
        }
        match self {
            Self::Sensor(object) => object.add_to_plot_with_options(plot, options),
            Self::Custom(object) => marker(plot, object.as_ref(), "Custom observer", options),
            Self::CustomGraphics(object) => object.add_to_plot_with_options(plot, options),
        }
    }
}
impl<T: Float> Graphics<T> for SourceAssembly<T> {
    fn add_to_plot_with_options(&self, plot: &mut Plot, options: &GraphicsOptions) {
        options.validate();
        if !options.visible() {
            return;
        }
        for object in self.components() {
            object.add_to_plot_with_options(plot, options);
        }
    }
}
impl<T: Float> Graphics<T> for ObserverAssembly<T> {
    fn add_to_plot_with_options(&self, plot: &mut Plot, options: &GraphicsOptions) {
        options.validate();
        if !options.visible() {
            return;
        }
        for object in self.components() {
            object.add_to_plot_with_options(plot, options);
        }
    }
}
impl<T: Float, S: Source<T> + Graphics<T>, const N: usize> Graphics<T> for SourceArray<S, N, T> {
    fn add_to_plot_with_options(&self, plot: &mut Plot, options: &GraphicsOptions) {
        options.validate();
        if !options.visible() {
            return;
        }
        for object in self.components() {
            object.add_to_plot_with_options(plot, options);
        }
    }
}
impl<T: Float, S: Observer<T> + Graphics<T>, const N: usize> Graphics<T>
    for ObserverArray<S, N, T>
{
    fn add_to_plot_with_options(&self, plot: &mut Plot, options: &GraphicsOptions) {
        options.validate();
        if !options.visible() {
            return;
        }
        for object in self.components() {
            object.add_to_plot_with_options(plot, options);
        }
    }
}
