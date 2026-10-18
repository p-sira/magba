/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2026 Sira Pornsiriprasert <code@psira.me>
 */

use nalgebra::{Point3, Vector3};

use crate::base::{Float, Pose, Source, Transform};

/// A source whose excluded positions produce non-finite field samples.
///
/// The predicate returns `true` to exclude a world-space position. It receives
/// the source so masks can follow its pose and geometry. Excluded positions are
/// never evaluated: heatmaps show gaps, quivers omit arrows, and field lines
/// stop before entering the mask. Rendering the source geometry is unaffected.
/// Masks are checked at sample positions and integration stages; features smaller
/// than the tracing step may be missed. Reduce the step size to assess convergence.
///
/// ```
/// use magba::{graphics::{FieldGrid, FieldMask, Plane}, prelude::*};
/// use nalgebra::Point3;
///
/// let magnet = SphereMagnet::<f64>::default();
/// let exterior = FieldMask::new(magnet, |m: &SphereMagnet, p: Point3<f64>| {
///     (p - m.position()).norm() <= m.diameter() / 2.0
/// });
/// let grid = FieldGrid::new(Plane::XY, 0.0, [-2.0, 2.0], [-2.0, 2.0], [21, 21])?;
/// let slice = exterior.sample_field(&grid)?;
/// # assert!(slice.fields()[220].iter().all(|v| v.is_nan()));
/// # Ok::<(), magba::graphics::GraphicsError>(())
/// ```
#[derive(Clone)]
pub struct FieldMask<S, F> {
    source: S,
    excluded: F,
}

impl<S, F> FieldMask<S, F> {
    /// Wrap a source with an exclusion predicate.
    ///
    /// To implement [`Source`], the predicate must implement
    /// `Fn(&S, Point3<T>) -> bool + Clone + Send + Sync`.
    pub fn new(source: S, excluded: F) -> Self {
        Self { source, excluded }
    }

    /// Inspect the source used by the predicate and field calculations.
    pub fn source(&self) -> &S {
        &self.source
    }

    /// Mutate the source's pose or geometry. The predicate sees these changes.
    pub fn source_mut(&mut self) -> &mut S {
        &mut self.source
    }

    /// Recover the unmasked source.
    pub fn into_source(self) -> S {
        self.source
    }
}

impl<T: Float, S: Transform<T>, F> Transform<T> for FieldMask<S, F> {
    fn pose(&self) -> &Pose<T> {
        self.source.pose()
    }
    fn pose_mut(&mut self) -> &mut Pose<T> {
        self.source.pose_mut()
    }
    fn set_pose(&mut self, pose: Pose<T>) {
        self.source.set_pose(pose);
    }
}

impl<T, S, F> Source<T> for FieldMask<S, F>
where
    T: Float,
    S: Source<T> + Clone,
    F: Fn(&S, Point3<T>) -> bool + Clone + Send + Sync,
{
    fn relative_complexity(&self) -> usize {
        self.source.relative_complexity()
    }

    fn compute_B(&self, point: Point3<T>) -> Vector3<T> {
        if (self.excluded)(&self.source, point) {
            Vector3::repeat(T::nan())
        } else {
            self.source.compute_B(point)
        }
    }

    fn compute_B_batch(&self, points: &[Point3<T>]) -> Vec<Vector3<T>> {
        let mut indices = Vec::new();
        let samples: Vec<_> = points
            .iter()
            .enumerate()
            .filter_map(|(i, &p)| {
                if (self.excluded)(&self.source, p) {
                    None
                } else {
                    indices.push(i);
                    Some(p)
                }
            })
            .collect();
        let mut fields = vec![Vector3::repeat(T::nan()); points.len()];
        if !samples.is_empty() {
            for (i, field) in indices
                .into_iter()
                .zip(self.source.compute_B_batch(&samples))
            {
                fields[i] = field;
            }
        }
        fields
    }
}

#[cfg(feature = "graphics-plotly")]
impl<T: Float, S: super::Graphics<T>, F> super::Graphics<T> for FieldMask<S, F> {
    fn add_to_plot_with_options(
        &self,
        plot: &mut super::plotly::Plot,
        options: &super::GraphicsOptions,
    ) {
        self.source.add_to_plot_with_options(plot, options);
    }
}
