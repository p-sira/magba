#![cfg(feature = "graphics")]

use magba::{
    base::Pose,
    graphics::{FieldGrid, FieldLineOptions, FieldMask, Plane},
    prelude::*,
};
use nalgebra::{Point3, Vector3};

#[derive(Clone, Default)]
struct GuardedSource(Pose<f32>);
impl Transform<f32> for GuardedSource {
    fn pose(&self) -> &Pose<f32> {
        &self.0
    }
    fn pose_mut(&mut self) -> &mut Pose<f32> {
        &mut self.0
    }
}
impl Source<f32> for GuardedSource {
    fn compute_B(&self, point: Point3<f32>) -> Vector3<f32> {
        assert!(point.x > self.pose().position().x);
        Vector3::x()
    }
    fn compute_B_batch(&self, points: &[Point3<f32>]) -> Vec<Vector3<f32>> {
        assert!(!points.is_empty());
        points.iter().map(|&p| self.compute_B(p)).collect()
    }
}

#[test]
fn mask_preserves_grid_order_and_stops_tracing_before_excluded_positions() {
    let mut source = FieldMask::new(
        GuardedSource::default(),
        |s: &GuardedSource, p: Point3<f32>| p.x <= s.pose().position().x,
    );
    let grid = FieldGrid::new(Plane::XY, 0.25, [-1.0, 1.0], [-2.0, 2.0], [3, 2]).unwrap();
    let slice = source.sample_field(&grid).unwrap();
    let grid = slice.grid();
    assert_eq!(grid.plane(), Plane::XY);
    assert_eq!(grid.offset(), 0.25);
    assert_eq!(grid.horizontal(), [-1.0, 1.0]);
    assert_eq!(grid.vertical(), [-2.0, 2.0]);
    assert_eq!(grid.resolution(), [3, 2]);
    assert_eq!(grid.points()[2], Point3::new(1.0, -2.0, 0.25));
    assert_eq!(grid.points()[3], Point3::new(-1.0, 2.0, 0.25));
    for (i, field) in slice.fields().iter().enumerate() {
        if i % 3 == 2 {
            assert_eq!(*field, Vector3::x());
        } else {
            assert!(field.iter().all(|v| v.is_nan()));
        }
    }
    let options = FieldLineOptions {
        step_size: 0.125,
        max_steps: 20,
        ..Default::default()
    };
    assert!(
        source
            .trace_field_lines(&[Point3::origin()], options)
            .unwrap()
            .is_empty()
    );
    let paths = source
        .trace_field_lines(&[Point3::new(0.5, 0.0, 0.0)], options)
        .unwrap();
    assert!(!paths.is_empty());
    assert!(paths.iter().flatten().all(|p| p.x > 0.0));

    // The predicate uses the current source pose, including changes after cloning.
    source.source_mut().pose_mut().set_position([2.0, 0.0, 0.0]);
    let clone = source.clone();
    assert!(
        clone
            .sample_field(grid)
            .unwrap()
            .fields()
            .iter()
            .all(|b| b.iter().all(|v| v.is_nan()))
    );
    assert!(clone.compute_B_batch(&[]).is_empty());
    assert_eq!(
        clone.into_source().pose().position(),
        source.source().pose().position()
    );
}

#[test]
fn mask_delegates_pose_and_complexity_and_follows_pose_changes() {
    let magnet = Dipole::<f64>::default();
    let mut masked = FieldMask::new(magnet.clone(), |s: &Dipole, p: Point3<f64>| {
        p.x <= s.position().x
    });
    assert_eq!(masked.relative_complexity(), magnet.relative_complexity());
    assert_eq!(masked.pose(), magnet.pose());

    let pose = *magnet.with_position([2.0, 0.0, 0.0]).pose();
    masked.set_pose(pose);
    assert_eq!(*masked.pose(), pose);
    assert!(
        masked
            .compute_B(Point3::new(1.0, 1.0, 1.0))
            .iter()
            .all(|v| v.is_nan())
    );
    masked.pose_mut().set_position([0.0; 3]);
    let point = Point3::new(1.0, 1.0, 1.0);
    assert_eq!(masked.compute_B(point), masked.source().compute_B(point));
}

#[cfg(feature = "graphics-plotly")]
#[test]
fn mask_preserves_geometry_and_rendering_options() {
    use magba::graphics::GraphicsOptions;

    let magnet = CuboidMagnet::<f64>::default().with_position([1.0, 2.0, 3.0]);
    let masked = FieldMask::new(magnet.clone(), |_: &CuboidMagnet, _: Point3<f64>| true);
    let options = GraphicsOptions::default().with_opacity(0.25);
    assert_eq!(
        masked.plot_with_options(&options).to_json(),
        magnet.plot_with_options(&options).to_json()
    );
    assert!(
        masked
            .compute_B(Point3::origin())
            .iter()
            .all(|v| v.is_nan())
    );
}
