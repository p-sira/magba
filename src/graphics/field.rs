/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2026 Sira Pornsiriprasert <code@psira.me>
 */

use nalgebra::{Point3, Vector3};
use plotly::{
    HeatMap, Layout, Plot, Scatter, Scatter3D,
    common::{ColorBar, ColorScalePalette, Line, Mode},
    layout::Axis,
};

use crate::base::{Float, Source};

/// Invalid graphics parameters or coordinates that cannot be represented by the source.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GraphicsError(&'static str);

impl core::fmt::Display for GraphicsError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(self.0)
    }
}
impl std::error::Error for GraphicsError {}

/// Coordinate plane used for a field cross-section. Coordinates are in meters.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Plane {
    /// Horizontal x, vertical y, fixed z.
    XY,
    /// Horizontal x, vertical z, fixed y.
    XZ,
    /// Horizontal y, vertical z, fixed x.
    YZ,
}

impl Plane {
    fn axes(self) -> [usize; 3] {
        match self {
            Self::XY => [0, 1, 2],
            Self::XZ => [0, 2, 1],
            Self::YZ => [1, 2, 0],
        }
    }

    /// Embed a pair of plane coordinates in world space at the given offset.
    pub fn point(self, uv: [f64; 2], offset: f64) -> Point3<f64> {
        let [u, v, normal] = self.axes();
        let mut point = Point3::origin();
        point[u] = uv[0];
        point[v] = uv[1];
        point[normal] = offset;
        point
    }

    /// Project a world-space point to this plane's horizontal and vertical coordinates.
    pub fn project(self, point: Point3<f64>) -> [f64; 2] {
        let [u, v, _] = self.axes();
        [point[u], point[v]]
    }

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

/// A validated, uniformly spaced rectangular sampling grid.
///
/// Ranges are inclusive, ordered from low to high. Resolution is `[columns, rows]`.
/// Coordinates use `f64`; both `f32` and `f64` sources can be sampled.
#[derive(Debug, Clone)]
pub struct FieldGrid {
    plane: Plane,
    offset: f64,
    horizontal: [f64; 2],
    vertical: [f64; 2],
    resolution: [usize; 2],
}

impl FieldGrid {
    /// Create a grid with at least two samples per axis and finite, increasing ranges.
    pub fn new(
        plane: Plane,
        offset: f64,
        horizontal: [f64; 2],
        vertical: [f64; 2],
        resolution: [usize; 2],
    ) -> Result<Self, GraphicsError> {
        if !offset.is_finite() || !valid_range(horizontal) || !valid_range(vertical) {
            return Err(GraphicsError(
                "Grid offset and ranges must be finite, with increasing ranges.",
            ));
        }
        if resolution.iter().any(|&n| n < 2) || resolution[0].checked_mul(resolution[1]).is_none() {
            return Err(GraphicsError(
                "Grid resolution must have at least two samples per axis and fit in usize.",
            ));
        }
        Ok(Self {
            plane,
            offset,
            horizontal,
            vertical,
            resolution,
        })
    }

    /// Plane in which samples lie.
    pub fn plane(&self) -> Plane {
        self.plane
    }

    /// World-space sample positions, ordered by rows, then columns.
    pub fn points(&self) -> Vec<Point3<f64>> {
        let [x, y] = self.axes();
        y.iter()
            .flat_map(|&v| {
                x.iter()
                    .map(move |&u| self.plane.point([u, v], self.offset))
            })
            .collect()
    }

    fn axes(&self) -> [Vec<f64>; 2] {
        fn linspace([min, max]: [f64; 2], count: usize) -> Vec<f64> {
            (0..count)
                .map(|i| {
                    let t = i as f64 / (count - 1) as f64;
                    (1.0 - t) * min + t * max
                })
                .collect()
        }
        [
            linspace(self.horizontal, self.resolution[0]),
            linspace(self.vertical, self.resolution[1]),
        ]
    }

    fn contains(&self, point: Point3<f64>) -> bool {
        let [u, v] = self.plane.project(point);
        (self.horizontal[0]..=self.horizontal[1]).contains(&u)
            && (self.vertical[0]..=self.vertical[1]).contains(&v)
    }
}

fn valid_range([min, max]: [f64; 2]) -> bool {
    min.is_finite() && max.is_finite() && min < max && (max - min).is_finite()
}

/// Scalar quantity to display in a heatmap, measured in tesla.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FieldComponent {
    Magnitude,
    X,
    Y,
    Z,
}

impl FieldComponent {
    fn value(self, field: Vector3<f64>) -> f64 {
        match self {
            Self::Magnitude => field.norm(),
            Self::X => field.x,
            Self::Y => field.y,
            Self::Z => field.z,
        }
    }
    fn label(self) -> &'static str {
        match self {
            Self::Magnitude => "|B| (T)",
            Self::X => "Bx (T)",
            Self::Y => "By (T)",
            Self::Z => "Bz (T)",
        }
    }
}

/// Quiver arrow sizing. Zero and non-finite vectors are omitted.
#[derive(Debug, Clone, Copy)]
pub struct QuiverOptions {
    /// Arrow length in meters when normalized, or a multiplier in meters per tesla otherwise.
    pub scale: f64,
    /// Give arrows equal length to emphasize direction. Defaults to true.
    pub normalize: bool,
}
impl Default for QuiverOptions {
    fn default() -> Self {
        Self {
            scale: 0.1,
            normalize: true,
        }
    }
}
impl QuiverOptions {
    fn validate(self) -> Result<(), GraphicsError> {
        if !self.scale.is_finite() || self.scale <= 0.0 {
            return Err(GraphicsError("Quiver scale must be finite and positive."));
        }
        Ok(())
    }
    fn displacement(self, field: Vector3<f64>) -> Option<Vector3<f64>> {
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

/// A sampled field cross-section, reusable for several plots without resampling.
#[derive(Debug, Clone)]
pub struct FieldSlice {
    grid: FieldGrid,
    fields: Vec<Vector3<f64>>,
}
impl FieldSlice {
    /// Sampling grid.
    pub fn grid(&self) -> &FieldGrid {
        &self.grid
    }
    /// World-space B vectors in tesla, ordered by rows, then columns.
    pub fn fields(&self) -> &[Vector3<f64>] {
        &self.fields
    }

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

/// Adaptive arc-length RK4 integration settings for field lines.
///
/// Integration runs forwards and backwards from every seed using normalized B.
/// Each direction stops at `max_steps`, bounds, zero/non-finite B, or a closed loop.
/// These are visual approximations; decrease the step size to assess convergence.
#[derive(Debug, Clone, Copy)]
pub struct FieldLineOptions {
    /// Maximum arc-length step in meters. Must be finite and positive.
    pub step_size: f64,
    /// Allowed local position error relative to `step_size`. Defaults to 1e-4.
    /// Steps shrink near rapid field changes, down to one millionth of `step_size`.
    pub tolerance: f64,
    /// Maximum RK4 steps per seed in each direction. Must be nonzero.
    pub max_steps: usize,
    /// Optional inclusive world-space bounds: `[[xmin, xmax], [ymin, ymax], [zmin, zmax]]`.
    pub bounds: Option<[[f64; 2]; 3]>,
    /// Stop when B magnitude is at or below this threshold in tesla.
    pub min_field: f64,
}
impl Default for FieldLineOptions {
    fn default() -> Self {
        Self {
            step_size: 0.01,
            tolerance: 1e-4,
            max_steps: 1000,
            bounds: None,
            min_field: 0.0,
        }
    }
}
impl FieldLineOptions {
    fn validate(&self) -> Result<(), GraphicsError> {
        if !self.step_size.is_finite()
            || self.step_size <= 0.0
            || !self.tolerance.is_finite()
            || self.tolerance <= 0.0
            || self.tolerance >= 1.0
            || self.max_steps == 0
            || !self.min_field.is_finite()
            || self.min_field < 0.0
        {
            return Err(GraphicsError(
                "Field lines require a positive finite step, tolerance between zero and one, nonzero step limit, and a nonnegative finite field threshold.",
            ));
        }
        if let Some(bounds) = self.bounds
            && bounds
                .iter()
                .any(|&[min, max]| !min.is_finite() || !max.is_finite() || min > max)
        {
            return Err(GraphicsError(
                "Field-line bounds must be finite and ordered.",
            ));
        }
        Ok(())
    }
    fn contains(&self, point: Point3<f64>) -> bool {
        point.iter().all(|x| x.is_finite())
            && self.bounds.is_none_or(|bounds| {
                (0..3).all(|i| (bounds[i][0]..=bounds[i][1]).contains(&point[i]))
            })
    }
}

/// Field plotting conveniences for every magnetic source, including collections and trait objects.
///
/// Coordinates and integration settings use `f64` meters. Sampling converts coordinates
/// to the source's scalar type, reporting unrepresentable inputs. Computed non-finite
/// fields become heatmap gaps, omitted arrows, or field-line stopping points.
pub trait FieldGraphics<T: Float = f64>: Source<T> {
    /// Sample a plane once, using the source's batch field implementation.
    fn sample_field(&self, grid: &FieldGrid) -> Result<FieldSlice, GraphicsError> {
        let points: Result<Vec<_>, _> = grid.points().into_iter().map(source_point::<T>).collect();
        let fields = self
            .compute_B_batch(&points?)
            .into_iter()
            .map(field_f64)
            .collect();
        Ok(FieldSlice {
            grid: grid.clone(),
            fields,
        })
    }

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

    /// Return 3D trajectories ordered from against B, through the seed, to along B.
    fn trace_field_lines(
        &self,
        seeds: &[Point3<f64>],
        options: FieldLineOptions,
    ) -> Result<Vec<Vec<Point3<f64>>>, GraphicsError> {
        trace_lines(self, seeds, options, None)
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

    /// Append 3D field trajectories to an existing scene.
    fn add_field_lines(
        &self,
        plot: &mut Plot,
        seeds: &[Point3<f64>],
        options: FieldLineOptions,
    ) -> Result<(), GraphicsError> {
        let paths = self.trace_field_lines(seeds, options)?;
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

    /// Append projected streamlines to a 2D plot. Both the grid and optional world bounds apply.
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
        let paths = trace_lines(self, &seeds, options, Some(grid))?;
        add_paths_2d(plot, grid.plane, &paths, "Field lines", "#ea580c");
        Ok(())
    }
}
impl<T: Float, S: Source<T> + ?Sized> FieldGraphics<T> for S {}

fn source_point<T: Float>(point: Point3<f64>) -> Result<Point3<T>, GraphicsError> {
    let mut result = Point3::origin();
    for i in 0..3 {
        result[i] = T::from(point[i])
            .filter(|v| v.is_finite())
            .ok_or(GraphicsError(
                "Sample coordinates must be finite and representable by the source scalar type.",
            ))?;
    }
    Ok(result)
}
fn field_f64<T: Float>(field: Vector3<T>) -> Vector3<f64> {
    field.map(|v| v.to_f64().unwrap_or(f64::NAN))
}

fn trace_lines<T: Float, S: Source<T> + ?Sized>(
    source: &S,
    seeds: &[Point3<f64>],
    options: FieldLineOptions,
    grid: Option<&FieldGrid>,
) -> Result<Vec<Vec<Point3<f64>>>, GraphicsError> {
    options.validate()?;
    for &seed in seeds {
        source_point::<T>(seed)?;
    }
    let inside = |p| options.contains(p) && grid.is_none_or(|g| g.contains(p));
    let direction = |p: Point3<f64>| -> Option<Vector3<f64>> {
        if !inside(p) {
            return None;
        }
        let mut field = field_f64(source.compute_B(source_point::<T>(p).ok()?));
        if !field.iter().all(|v| v.is_finite()) {
            return None;
        }
        if let Some(grid) = grid {
            field[grid.plane.axes()[2]] = 0.0;
        }
        let norm = field.norm();
        if !norm.is_finite() || norm <= options.min_field {
            return None;
        }
        Some(field / norm)
    };
    let mut paths = Vec::new();
    for &seed in seeds {
        if direction(seed).is_none() {
            continue;
        }
        let integrate = |sign: f64| {
            let mut h = options.step_size * sign;
            let mut path = vec![seed];
            let rk4 = |p, h| -> Option<Point3<f64>> {
                let k1 = direction(p)?;
                let k2 = direction(p + k1 * (h / 2.0))?;
                let k3 = direction(p + k2 * (h / 2.0))?;
                let k4 = direction(p + k3 * h)?;
                Some(p + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (h / 6.0))
            };
            for _ in 0..options.max_steps {
                let p = *path.last().unwrap();
                let mut accepted = None;
                // Step doubling estimates local error and resolves material boundaries.
                for _ in 0..20 {
                    let coarse = rk4(p, h);
                    let fine = rk4(p, h / 2.0).and_then(|mid| rk4(mid, h / 2.0));
                    if let (Some(coarse), Some(fine)) = (coarse, fine) {
                        let error = (fine - coarse).norm() / 15.0;
                        if error <= options.step_size * options.tolerance {
                            accepted = Some(fine);
                            if error < options.step_size * options.tolerance / 32.0 {
                                h = sign * (2.0 * h.abs()).min(options.step_size);
                            }
                            break;
                        }
                    }
                    h /= 2.0;
                    if h.abs() < options.step_size * 1e-6 {
                        break;
                    }
                }
                let Some(next) = accepted else {
                    break;
                };
                if !inside(next)
                    || direction(next).is_none()
                    || (next - p).norm() < options.step_size * 1e-9
                {
                    break;
                }
                path.push(next);
                if path.len() > 8
                    && (next - seed).norm() < options.step_size
                    && path
                        .iter()
                        .any(|p| (p - seed).norm() > 4.0 * options.step_size)
                {
                    path.push(seed);
                    break;
                }
            }
            path
        };
        let mut path = integrate(-1.0);
        path.reverse();
        if path.len() > 1 && path.first() == path.last() {
            paths.push(path);
            continue;
        }
        path.pop();
        path.extend(integrate(1.0));
        if path.len() > 1 {
            paths.push(path);
        }
    }
    Ok(paths)
}

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

#[cfg(test)]
mod tests;
