#![cfg(feature = "graphics-plotly")]
#[path = "../examples/graphics/common/exterior.rs"]
mod exterior;

use exterior::{Exterior, Interior};
use magba::{
    base::Pose,
    graphics::{FieldGrid, FieldLineOptions, Plane},
    prelude::*,
    sources,
};
use nalgebra::{Point3, UnitQuaternion, Vector3};
use std::sync::{
    Arc,
    atomic::{AtomicUsize, Ordering},
};

#[derive(Clone)]
struct GuardedSource {
    pose: Pose,
    calls: Arc<AtomicUsize>,
}
impl Transform<f64> for GuardedSource {
    fn pose(&self) -> &Pose {
        &self.pose
    }
    fn pose_mut(&mut self) -> &mut Pose {
        &mut self.pose
    }
}
impl Interior for GuardedSource {
    fn contains(&self, point: Point3<f64>) -> bool {
        point.coords.norm_squared() <= 1.0
    }
}
impl Source<f64> for GuardedSource {
    fn compute_B(&self, point: Point3<f64>) -> Vector3<f64> {
        assert!(!self.contains(point), "An interior field was evaluated");
        self.calls.fetch_add(1, Ordering::Relaxed);
        Vector3::x()
    }
    fn compute_B_batch(&self, points: &[Point3<f64>]) -> Vec<Vector3<f64>> {
        points.iter().map(|&p| self.compute_B(p)).collect()
    }
}

#[test]
fn excluded_points_never_reach_the_source_during_sampling_or_integration() {
    let calls = Arc::new(AtomicUsize::new(0));
    let source = Exterior(GuardedSource {
        pose: Pose::default(),
        calls: calls.clone(),
    });
    let grid = FieldGrid::new(Plane::XY, 0.0, [-2.0, 2.0], [-2.0, 2.0], [3, 3]).unwrap();
    let slice = source.sample_field(&grid).unwrap();
    assert!(slice.fields()[4].iter().all(|v| v.is_nan()));
    assert_eq!(calls.load(Ordering::Relaxed), 8);
    assert!(
        source
            .compute_B(Point3::origin())
            .iter()
            .all(|v| v.is_nan())
    );
    assert_eq!(calls.load(Ordering::Relaxed), 8);
    let options = FieldLineOptions {
        step_size: 0.1,
        max_steps: 80,
        bounds: Some([[-3.0, 3.0]; 3]),
        ..Default::default()
    };
    assert!(
        source
            .trace_field_lines(&[Point3::origin()], options)
            .unwrap()
            .is_empty()
    );
    assert_eq!(calls.load(Ordering::Relaxed), 8);
    let paths = source
        .trace_field_lines(&[Point3::new(2.0, 0.0, 0.0)], options)
        .unwrap();
    assert!(!paths.is_empty());
    assert!(paths.iter().flatten().all(|&p| !source.0.contains(p)));
}

#[test]
fn volume_masks_follow_poses_and_collection_components() {
    let position = Point3::new(0.3, -0.2, 0.1);
    let rotation = UnitQuaternion::from_scaled_axis(Vector3::new(0.4, 0.1, 0.2));
    let sphere = SphereMagnet::<f64>::default()
        .with_diameter(0.02)
        .with_position(position)
        .with_orientation(rotation);
    let cube = CuboidMagnet::default()
        .with_dimensions([0.02; 3])
        .with_position(position)
        .with_orientation(rotation);
    let cylinder = CylinderMagnet::default()
        .with_diameter(0.02)
        .with_height(0.02)
        .with_position(position)
        .with_orientation(rotation);
    for shape in [&sphere as &dyn Interior, &cube, &cylinder] {
        assert!(shape.contains(position));
        assert!(!shape.contains(position + rotation * Vector3::new(0.02, 0.0, 0.0)));
    }
    let tetrahedron = TetrahedronMagnet::<f64>::default()
        .with_position(position)
        .with_orientation(rotation);
    let inside = position + rotation * Vector3::repeat(0.1);
    assert!(tetrahedron.contains(inside));
    assert!(!tetrahedron.contains(position + rotation * Vector3::repeat(1.0)));
    let mut vertices = tetrahedron.vertices();
    vertices.swap(2, 3);
    assert!(tetrahedron.clone().with_vertices(vertices).contains(inside));
    let collection = sources!(
        sources!(sphere),
        cube,
        cylinder,
        tetrahedron,
        CircularCurrent::default()
    );
    assert!(collection.contains(position));
    assert!(!collection.contains(Point3::new(5.0, 5.0, 5.0)));
}
