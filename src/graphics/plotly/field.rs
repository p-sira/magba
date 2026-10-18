/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2026 Sira Pornsiriprasert <code@psira.me>
 */

use crate::base::{Float, Source};
use crate::graphics::field::{field_f64, source_point, trace_lines};
use crate::graphics::{
    FieldComponent, FieldGrid, FieldLineOptions, FieldSampler, FieldSlice, GraphicsError, Plane,
    QuiverOptions,
};
use ::plotly::{
    HeatMap, Layout, Plot, Scatter, Scatter3D,
    common::{ColorBar, ColorScalePalette, Line, Mode},
    layout::Axis,
};
use nalgebra::{Point3, Vector3};

/// Validate arrow scaling and compute field-vector displacements.
impl QuiverOptions {
    pub(crate) fn validate(self) -> Result<(), GraphicsError> {
        if !self.scale.is_finite() || self.scale <= 0.0 {
            return Err(GraphicsError("Quiver scale must be finite and positive."));
        }
        Ok(())
    }
    pub(crate) fn displacement(self, field: Vector3<f64>) -> Option<Vector3<f64>> {
        let norm = field.norm();
        if !norm.is_finite() || norm == 0.0 {
            return None;
        }
        let delta = if self.normalize {
            field / norm * self.scale
        } else {
            field * self.scale
        };
        delta.iter().all(|x| x.is_finite()).then_some(delta)
    }
}

/// Resolve scalar field values and labels for plotting.
impl FieldComponent {
    pub(crate) fn value(self, field: Vector3<f64>) -> f64 {
        match self {
            Self::Magnitude => field.norm(),
            Self::X => field.x,
            Self::Y => field.y,
            Self::Z => field.z,
        }
    }
}

/// Build a Plotly layout with equal scales and coordinate labels.
impl Plane {
    fn layout(self) -> Layout {
        let [u, v, _] = self.axes();
        let labels = ["x (m)", "y (m)", "z (m)"];
        Layout::new().x_axis(Axis::new().title(labels[u])).y_axis(
            Axis::new()
                .title(labels[v])
                .scale_anchor("x")
                .scale_ratio(1.0),
        )
    }
}
/// Resolve scalar field values and labels for plotting.
impl FieldComponent {
    fn label(self) -> &'static str {
        match self {
            Self::Magnitude => "|B| (T)",
            Self::X => "Bx (T)",
            Self::Y => "By (T)",
            Self::Z => "Bz (T)",
        }
    }
}
/// Render reusable field samples as Plotly heatmaps and quivers.
impl FieldSlice {
    /// Create a heatmap with equal axis scales. Invalid samples appear as gaps.
    pub fn heatmap(&self, component: FieldComponent) -> Plot {
        let mut plot = Plot::new();
        plot.set_layout(self.grid.plane.layout());
        self.add_heatmap(&mut plot, component);
        plot
    }

    /// Append a heatmap, preserving the plot layout. Components use a zero-centered color scale.
    pub fn add_heatmap(&self, plot: &mut Plot, component: FieldComponent) {
        let [x, y] = self.grid.axes();
        let z: Vec<Vec<Option<f64>>> = self
            .fields
            .chunks(x.len())
            .map(|row| {
                row.iter()
                    .map(|&field| {
                        let value = component.value(field);
                        value.is_finite().then_some(value)
                    })
                    .collect()
            })
            .collect();
        let palette = if component == FieldComponent::Magnitude {
            ColorScalePalette::Viridis
        } else {
            ColorScalePalette::RdBu
        };
        let mut trace = HeatMap::new(x, y, z)
            .name(component.label())
            .color_bar(ColorBar::new().title(component.label()))
            .color_scale(palette.into())
            .connect_gaps(false)
            .hover_on_gaps(false);
        if component != FieldComponent::Magnitude {
            trace = trace.zmid(0.0);
        }
        plot.add_trace(trace);
    }

    /// Create a quiver of the in-plane B components with arrowheads.
    pub fn quiver(&self, options: QuiverOptions) -> Result<Plot, GraphicsError> {
        let mut plot = Plot::new();
        plot.set_layout(self.grid.plane.layout());
        self.add_quiver(&mut plot, options)?;
        Ok(plot)
    }

    /// Append in-plane arrows, preserving the plot layout.
    pub fn add_quiver(&self, plot: &mut Plot, options: QuiverOptions) -> Result<(), GraphicsError> {
        options.validate()?;
        let plane = self.grid.plane;
        let normal = plane.axes()[2];
        let mut paths = Vec::new();
        for (point, &field) in self.grid.points().iter().zip(&self.fields) {
            if !field.iter().all(|v| v.is_finite()) {
                continue;
            }
            let mut field = field;
            field[normal] = 0.0;
            if let Some(delta) = options.displacement(field) {
                let mut side = Vector3::zeros();
                side[normal] = 1.0;
                paths.extend(arrow(*point, delta, side.cross(&delta)));
            }
        }
        add_paths_2d(plot, plane, &paths, "B direction", "#172554");
        Ok(())
    }
}

/// Plot magnetic field heatmaps, quivers, and directed field lines with Plotly.
///
/// Available for every [`Source`], including collections and trait objects, with
/// the `graphics-plotly` feature. Use [`super::Graphics`] to add source geometry
/// and [`FieldSampler`] for backend-independent sampling and integration.
pub trait FieldGraphics<T: Float = f64>: FieldSampler<T> {
    /// Sample and plot a 2D scalar field heatmap.
    fn plot_field_2d(
        &self,
        grid: &FieldGrid,
        component: FieldComponent,
    ) -> Result<Plot, GraphicsError> {
        Ok(self.sample_field(grid)?.heatmap(component))
    }

    /// Sample and plot arrows for the field's in-plane components.
    fn plot_quiver_2d(
        &self,
        grid: &FieldGrid,
        options: QuiverOptions,
    ) -> Result<Plot, GraphicsError> {
        options.validate()?;
        self.sample_field(grid)?.quiver(options)
    }

    /// Sample and plot world-space field arrows in a 3D scene.
    fn plot_quiver(
        &self,
        points: &[Point3<f64>],
        options: QuiverOptions,
    ) -> Result<Plot, GraphicsError> {
        let mut plot = super::scene();
        self.add_quiver(&mut plot, points, options)?;
        Ok(plot)
    }

    /// Append 3D field arrows to an existing scene without changing its layout.
    fn add_quiver(
        &self,
        plot: &mut Plot,
        points: &[Point3<f64>],
        options: QuiverOptions,
    ) -> Result<(), GraphicsError> {
        options.validate()?;
        let samples: Result<Vec<_>, _> = points.iter().copied().map(source_point::<T>).collect();
        let fields = self.compute_B_batch(&samples?);
        let mut paths = Vec::new();
        for (&point, field) in points.iter().zip(fields) {
            if let Some(delta) = options.displacement(field_f64(field)) {
                let unit = delta.normalize();
                let reference = if unit.z.abs() < 0.9 {
                    Vector3::z()
                } else {
                    Vector3::x()
                };
                paths.extend(arrow(point, delta, reference.cross(&delta)));
            }
        }
        add_paths_3d(plot, &paths, "B direction", "#2563eb");
        Ok(())
    }

    /// Plot 3D field trajectories. Add source geometry with [`super::Graphics::add_to_plot`].
    fn plot_field_lines(
        &self,
        seeds: &[Point3<f64>],
        options: FieldLineOptions,
    ) -> Result<Plot, GraphicsError> {
        let mut plot = super::scene();
        self.add_field_lines(&mut plot, seeds, options)?;
        Ok(plot)
    }

    /// Append 3D field trajectories with arrowheads indicating the B direction.
    fn add_field_lines(
        &self,
        plot: &mut Plot,
        seeds: &[Point3<f64>],
        options: FieldLineOptions,
    ) -> Result<(), GraphicsError> {
        let mut paths = self.trace_field_lines(seeds, options)?;
        add_line_arrowheads(&mut paths, options.step_size, None);
        add_paths_3d(plot, &paths, "Field lines", "#ea580c");
        Ok(())
    }

    /// Plot streamlines of the field projected into the grid's plane, clipped to its ranges.
    ///
    /// Seeds are `[horizontal, vertical]` coordinates in meters. These coincide with
    /// 3D field lines only where B has no component normal to the plane.
    fn plot_field_lines_2d(
        &self,
        grid: &FieldGrid,
        seeds: &[[f64; 2]],
        options: FieldLineOptions,
    ) -> Result<Plot, GraphicsError> {
        let mut plot = Plot::new();
        plot.set_layout(grid.plane.layout());
        self.add_field_lines_2d(&mut plot, grid, seeds, options)?;
        Ok(plot)
    }

    /// Append projected streamlines with direction arrowheads. Grid and world bounds apply.
    fn add_field_lines_2d(
        &self,
        plot: &mut Plot,
        grid: &FieldGrid,
        seeds: &[[f64; 2]],
        options: FieldLineOptions,
    ) -> Result<(), GraphicsError> {
        let seeds: Vec<_> = seeds
            .iter()
            .map(|&uv| grid.plane.point(uv, grid.offset))
            .collect();
        let mut paths = trace_lines(self, &seeds, options, Some(grid))?;
        add_line_arrowheads(&mut paths, options.step_size, Some(grid.plane));
        add_paths_2d(plot, grid.plane, &paths, "Field lines", "#ea580c");
        Ok(())
    }
}
/// Plot sampled fields and directed field lines for any magnetic source.
impl<T: Float, S: Source<T> + ?Sized> FieldGraphics<T> for S {}

fn arrow(tail: Point3<f64>, delta: Vector3<f64>, side: Vector3<f64>) -> [Vec<Point3<f64>>; 2] {
    let tip = tail + delta;
    let side = side.normalize() * delta.norm() * 0.12;
    let back = tip - delta * 0.3;
    [vec![tail, tip], vec![back + side, tip, back - side]]
}

fn path_coordinates(paths: &[Vec<Point3<f64>>]) -> [Vec<Option<f64>>; 3] {
    let mut xyz = [Vec::new(), Vec::new(), Vec::new()];
    for path in paths {
        for point in path {
            for i in 0..3 {
                xyz[i].push(Some(point[i]));
            }
        }
        for axis in &mut xyz {
            axis.push(None);
        }
    }
    xyz
}
fn add_paths_2d(
    plot: &mut Plot,
    plane: Plane,
    paths: &[Vec<Point3<f64>>],
    name: &str,
    color: &'static str,
) {
    if paths.is_empty() {
        return;
    }
    let mut xyz = path_coordinates(paths);
    let [u, v, _] = plane.axes();
    let x = core::mem::take(&mut xyz[u]);
    let y = core::mem::take(&mut xyz[v]);
    plot.add_trace(
        Scatter::new(x, y)
            .mode(Mode::Lines)
            .name(name)
            .connect_gaps(false)
            .line(Line::new().color(color).width(1.5)),
    );
}
fn add_paths_3d(plot: &mut Plot, paths: &[Vec<Point3<f64>>], name: &str, color: &'static str) {
    if paths.is_empty() {
        return;
    }
    let [x, y, z] = path_coordinates(paths);
    plot.add_trace(
        Scatter3D::new(x, y, z)
            .mode(Mode::Lines)
            .name(name)
            .connect_gaps(false)
            .line(Line::new().color(color).width(3.0)),
    );
}

// Place two heads at equal arc-length intervals, independent of adaptive step density.
// Trajectories are ordered along B, including the branch integrated backwards.
fn add_line_arrowheads(paths: &mut Vec<Vec<Point3<f64>>>, step_size: f64, plane: Option<Plane>) {
    let mut heads = Vec::new();
    for path in paths.iter() {
        let length: f64 = path.windows(2).map(|p| (p[1] - p[0]).norm()).sum();
        if length <= 0.0 {
            continue;
        }
        let size = (8.0 * step_size).min(length * 0.2);
        for fraction in [1.0 / 3.0, 2.0 / 3.0] {
            let target = length * fraction;
            let mut distance = 0.0;
            for segment in path.windows(2) {
                let delta = segment[1] - segment[0];
                let span = delta.norm();
                if span > 0.0 && distance + span >= target {
                    let unit = delta / span;
                    let tip = segment[0] + unit * (target - distance);
                    let reference = plane
                        .map(|p| {
                            let mut normal = Vector3::zeros();
                            normal[p.axes()[2]] = 1.0;
                            normal
                        })
                        .unwrap_or_else(|| {
                            if unit.z.abs() < 0.9 {
                                Vector3::z()
                            } else {
                                Vector3::x()
                            }
                        });
                    heads.push(
                        arrow(tip - unit * size, unit * size, reference.cross(&unit))[1].clone(),
                    );
                    break;
                }
                distance += span;
            }
        }
    }
    paths.extend(heads);
}

#[cfg(test)]
mod tests;
