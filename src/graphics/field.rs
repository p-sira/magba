/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2026 Sira Pornsiriprasert <code@psira.me>
 */

use nalgebra::{Point3, Vector3};

use crate::base::{Float, Source};

/// Invalid graphics parameters or coordinates that cannot be represented by the source.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GraphicsError(pub(crate) &'static str);

/// Display the validation or coordinate-conversion error.
impl core::fmt::Display for GraphicsError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(self.0)
    }
}
/// Expose graphics failures through the standard error interface.
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

/// Convert between plane coordinates and world coordinates.
impl Plane {
    pub(crate) fn axes(self) -> [usize; 3] {
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
}

/// A validated, uniformly spaced rectangular sampling grid.
///
/// Ranges are inclusive, ordered from low to high. Resolution is `[columns, rows]`.
/// Coordinates use [`f64`]; both [`f32`] and [`f64`] sources can be sampled.
#[derive(Debug, Clone)]
pub struct FieldGrid {
    pub(crate) plane: Plane,
    pub(crate) offset: f64,
    pub(crate) horizontal: [f64; 2],
    pub(crate) vertical: [f64; 2],
    pub(crate) resolution: [usize; 2],
}

/// Construct and inspect a validated field-sampling grid.
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

    /// Fixed world-space coordinate normal to the plane, in meters.
    pub fn offset(&self) -> f64 {
        self.offset
    }

    /// Inclusive horizontal range in meters.
    pub fn horizontal(&self) -> [f64; 2] {
        self.horizontal
    }

    /// Inclusive vertical range in meters.
    pub fn vertical(&self) -> [f64; 2] {
        self.vertical
    }

    /// Sample counts as `[columns, rows]`.
    pub fn resolution(&self) -> [usize; 2] {
        self.resolution
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

    pub(crate) fn axes(&self) -> [Vec<f64>; 2] {
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

    pub(crate) fn contains(&self, point: Point3<f64>) -> bool {
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

/// Quiver arrow sizing. Zero and non-finite vectors are omitted.
#[derive(Debug, Clone, Copy)]
pub struct QuiverOptions {
    /// Arrow length in meters when normalized, or a multiplier in meters per tesla otherwise.
    pub scale: f64,
    /// Give arrows equal length to emphasize direction. Defaults to true.
    pub normalize: bool,
}
/// Use normalized arrows with a length of 0.1 meters.
impl Default for QuiverOptions {
    fn default() -> Self {
        Self {
            scale: 0.1,
            normalize: true,
        }
    }
}

/// A sampled field cross-section, reusable for several plots without resampling.
#[derive(Debug, Clone)]
pub struct FieldSlice {
    pub(crate) grid: FieldGrid,
    pub(crate) fields: Vec<Vector3<f64>>,
}
/// Inspect the sampled grid and magnetic field vectors.
impl FieldSlice {
    /// Sampling grid.
    pub fn grid(&self) -> &FieldGrid {
        &self.grid
    }
    /// World-space B vectors in tesla, ordered by rows, then columns.
    pub fn fields(&self) -> &[Vector3<f64>] {
        &self.fields
    }
}

/// Adaptive arc-length RK4 integration settings for field lines.
///
/// Integration runs forwards and backwards from every seed using normalized B.
/// Each direction stops at [`max_steps`](Self::max_steps), bounds, zero/non-finite B, an unresolved
/// turn, or a closed loop.
/// These are visual approximations; decrease the step size to assess convergence.
#[derive(Debug, Clone, Copy)]
pub struct FieldLineOptions {
    /// Maximum arc-length step in meters. Must be finite and positive.
    pub step_size: f64,
    /// Allowed local position error relative to [`step_size`](Self::step_size). Defaults to 1e-4.
    /// Steps shrink near rapid field changes, down to one millionth of `step_size`.
    pub tolerance: f64,
    /// Maximum RK4 steps per seed in each direction. Must be nonzero.
    pub max_steps: usize,
    /// Optional inclusive world-space bounds: `[[xmin, xmax], [ymin, ymax], [zmin, zmax]]`.
    pub bounds: Option<[[f64; 2]; 3]>,
    /// Stop when B magnitude is at or below this threshold in tesla.
    pub min_field: f64,
}
/// Use adaptive tracing with the default step size and stopping limits.
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
/// Validate field-line settings and check trajectory bounds.
impl FieldLineOptions {
    pub(crate) fn validate(&self) -> Result<(), GraphicsError> {
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
    pub(crate) fn contains(&self, point: Point3<f64>) -> bool {
        point.iter().all(|x| x.is_finite())
            && self.bounds.is_none_or(|bounds| {
                (0..3).all(|i| (bounds[i][0]..=bounds[i][1]).contains(&point[i]))
            })
    }
}

/// Field sampling and trajectory integration for every source, including collections and trait objects.
///
/// Coordinates and integration settings use [`f64`] meters. Sampling converts coordinates
/// to the source's scalar type, reporting unrepresentable inputs. Computed non-finite
/// fields become heatmap gaps, omitted arrows, or field-line stopping points.
pub trait FieldSampler<T: Float = f64>: Source<T> {
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

    /// Return 3D trajectories ordered from against B, through the seed, to along B.
    fn trace_field_lines(
        &self,
        seeds: &[Point3<f64>],
        options: FieldLineOptions,
    ) -> Result<Vec<Vec<Point3<f64>>>, GraphicsError> {
        trace_lines(self, seeds, options, None)
    }
}
/// Provide field sampling and trajectory integration for all sources.
impl<T: Float, S: Source<T> + ?Sized> FieldSampler<T> for S {}

pub(crate) fn source_point<T: Float>(point: Point3<f64>) -> Result<Point3<T>, GraphicsError> {
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
pub(crate) fn field_f64<T: Float>(field: Vector3<T>) -> Vector3<f64> {
    field.map(|v| v.to_f64().unwrap_or(f64::NAN))
}

pub(crate) fn trace_lines<T: Float, S: Source<T> + ?Sized>(
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
                        // Reject unresolved turns whose RK stages cancel or advance
                        // against the field, even if coarse and fine steps agree.
                        let delta = fine - p;
                        let length = delta.norm();
                        let tolerance = options.step_size * options.tolerance;
                        let advances = length > h.abs() * 0.5
                            && direction(p).is_some_and(|d| delta.dot(&d) * sign > 0.0);
                        // Below the position tolerance, permit a kink across a
                        // material boundary; larger segments must follow local B.
                        let follows = length <= tolerance
                            || direction(p + delta / 2.0)
                                .is_some_and(|d| delta.dot(&d) * sign > length * 0.99);
                        if error <= tolerance && advances && follows {
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

#[cfg(test)]
mod tests;
