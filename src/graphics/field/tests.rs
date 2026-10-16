use super::*;
use crate::magnets::{Dipole, StableFieldMagnet};
use serde_json::{Value, json};

fn grid(plane: Plane) -> FieldGrid {
    FieldGrid::new(plane, 0.25, [-1.0, 1.0], [-2.0, 2.0], [3, 5]).unwrap()
}

#[test]
fn slices_preserve_world_components_and_plane_coordinates() {
    let source = StableFieldMagnet::new(Vector3::new(3.0, 4.0, 12.0));
    for plane in [Plane::XY, Plane::XZ, Plane::YZ] {
        let grid = grid(plane);
        let slice = source.sample_field(&grid).unwrap();
        assert_eq!(slice.fields(), vec![Vector3::new(3.0, 4.0, 12.0); 15]);
        for component in [
            FieldComponent::Magnitude,
            FieldComponent::X,
            FieldComponent::Y,
            FieldComponent::Z,
        ] {
            let plot: Value = serde_json::from_str(&slice.heatmap(component).to_json()).unwrap();
            let expected = match component {
                FieldComponent::Magnitude => 13.0,
                FieldComponent::X => 3.0,
                FieldComponent::Y => 4.0,
                FieldComponent::Z => 12.0,
            };
            assert_eq!(plot["data"][0]["z"], json!(vec![vec![expected; 3]; 5]));
            assert_eq!(plot["data"][0]["x"], json!([-1.0, 0.0, 1.0]));
            assert_eq!(plot["layout"]["yaxis"]["scaleanchor"], "x");
        }
        for p in grid.points() {
            assert_eq!(plane.point(plane.project(p), 0.25), p);
        }
    }
}

#[test]
fn quivers_project_normalize_and_draw_arrowheads() {
    let source = StableFieldMagnet::new(Vector3::new(3.0, 4.0, 12.0));
    let plot = source
        .plot_quiver_2d(
            &grid(Plane::XY),
            QuiverOptions {
                scale: 0.5,
                normalize: true,
            },
        )
        .unwrap();
    let plot: Value = serde_json::from_str(&plot.to_json()).unwrap();
    let trace = &plot["data"][0];
    let dx = trace["x"][1].as_f64().unwrap() - trace["x"][0].as_f64().unwrap();
    let dy = trace["y"][1].as_f64().unwrap() - trace["y"][0].as_f64().unwrap();
    assert!((dx - 0.3).abs() < 1e-14);
    assert!((dy - 0.4).abs() < 1e-14);
    assert!(trace["x"][2].is_null()); // Disconnected shaft and arrowhead.
    assert!(trace["x"][3].is_number());
    let source = StableFieldMagnet::<f64>::new(Vector3::z());
    let plot = source
        .plot_quiver_2d(&grid(Plane::XY), QuiverOptions::default())
        .unwrap();
    let plot: Value = serde_json::from_str(&plot.to_json()).unwrap();
    assert_eq!(plot["data"], json!([]));
}

#[test]
fn field_lines_follow_uniform_field_and_stop_at_bounds() {
    let source = StableFieldMagnet::new(Vector3::new(2.0, 0.0, 0.0));
    let options = FieldLineOptions {
        step_size: 0.125,
        bounds: Some([[-1.0, 1.0]; 3]),
        ..Default::default()
    };
    let paths = source
        .trace_field_lines(&[Point3::origin()], options)
        .unwrap();
    let path = &paths[0];
    assert_eq!(path.len(), 17);
    assert_eq!(path.first().unwrap(), &Point3::new(-1.0, 0.0, 0.0));
    assert_eq!(path.last().unwrap(), &Point3::new(1.0, 0.0, 0.0));
    assert!(
        path.windows(2)
            .all(|p| (p[1].x - p[0].x - 0.125).abs() < 1e-15)
    );
}

#[test]
fn rk4_converges_to_analytic_dipole_line() {
    // For a z dipole through (1, 0, 0), r = sin²(theta), hence r³ = x².
    let source = Dipole::<f64>::default();
    let mut errors = Vec::new();
    for step_size in [0.04, 0.02] {
        let paths = source
            .trace_field_lines(
                &[Point3::new(1.0, 0.0, 0.0)],
                FieldLineOptions {
                    step_size,
                    max_steps: (0.8 / step_size) as usize,
                    ..Default::default()
                },
            )
            .unwrap();
        let error = paths[0]
            .iter()
            .map(|p| (p.coords.norm().powi(3) - p.x.powi(2)).abs())
            .fold(0.0, f64::max);
        errors.push(error);
    }
    assert!(errors[1] < 1e-8);
    assert!(errors[0] / errors[1] > 12.0, "RK4 errors: {errors:?}");
}

#[test]
fn zero_and_invalid_fields_stop_without_invalid_plot_coordinates() {
    for field in [
        Vector3::zeros(),
        Vector3::new(f64::NAN, 1.0, 0.0),
        Vector3::new(0.0, 0.0, f64::INFINITY),
    ] {
        let source = StableFieldMagnet::new(field);
        assert!(
            source
                .trace_field_lines(&[Point3::origin()], FieldLineOptions::default())
                .unwrap()
                .is_empty()
        );
        let plot = source
            .plot_quiver(&[Point3::origin()], QuiverOptions::default())
            .unwrap();
        let plot: Value = serde_json::from_str(&plot.to_json()).unwrap();
        assert_eq!(plot["data"], json!([]));
    }
    let source = StableFieldMagnet::new(Vector3::new(f64::NAN, 0.0, 0.0));
    let plot: Value = serde_json::from_str(
        &source
            .sample_field(&grid(Plane::XY))
            .unwrap()
            .heatmap(FieldComponent::Magnitude)
            .to_json(),
    )
    .unwrap();
    assert!(plot["data"][0]["z"][0][0].is_null());
}

#[test]
fn parameters_are_validated_and_f32_trait_objects_work() {
    assert!(FieldGrid::new(Plane::XY, 0.0, [1.0, -1.0], [0.0, 1.0], [2, 2]).is_err());
    assert!(FieldGrid::new(Plane::XY, f64::NAN, [0.0, 1.0], [0.0, 1.0], [2, 2]).is_err());
    assert!(FieldGrid::new(Plane::XY, 0.0, [0.0, 1.0], [0.0, 1.0], [1, 2]).is_err());
    let source = StableFieldMagnet::<f32>::new(Vector3::z());
    let object: &dyn Source<f32> = &source;
    assert_eq!(
        object
            .sample_field(&grid(Plane::XZ))
            .unwrap()
            .fields()
            .len(),
        15
    );
    assert!(
        source
            .plot_quiver(&[Point3::new(f64::MAX, 0.0, 0.0)], QuiverOptions::default())
            .is_err()
    );
    assert!(
        source
            .plot_quiver(
                &[],
                QuiverOptions {
                    scale: f64::NAN,
                    normalize: true
                }
            )
            .is_err()
    );
    assert!(
        source
            .trace_field_lines(
                &[],
                FieldLineOptions {
                    max_steps: 0,
                    ..Default::default()
                }
            )
            .is_err()
    );
    assert!(
        source
            .trace_field_lines(
                &[],
                FieldLineOptions {
                    bounds: Some([[1.0, -1.0]; 3]),
                    ..Default::default()
                }
            )
            .is_err()
    );
}

#[test]
fn projected_lines_stay_in_slice_and_respect_step_budget() {
    let source = StableFieldMagnet::new(Vector3::new(1.0, 2.0, 3.0));
    let grid = grid(Plane::XZ);
    let options = FieldLineOptions {
        max_steps: 3,
        step_size: 0.01,
        ..Default::default()
    };
    let paths = trace_lines(
        &source,
        &[grid.plane.point([0.0, 0.0], grid.offset)],
        options,
        Some(&grid),
    )
    .unwrap();
    assert_eq!(paths[0].len(), 7);
    assert!(paths[0].iter().all(|p| p.y == grid.offset));
    let plot = source
        .plot_field_lines_2d(&grid, &[[0.0, 0.0]], options)
        .unwrap();
    let plot: Value = serde_json::from_str(&plot.to_json()).unwrap();
    assert_eq!(plot["data"][0]["type"], "scatter");
}

#[test]
fn closed_lines_are_drawn_once() {
    #[derive(Clone)]
    struct CircularField(crate::base::Pose);
    impl crate::base::Transform<f64> for CircularField {
        fn pose(&self) -> &crate::base::Pose {
            &self.0
        }
        fn pose_mut(&mut self) -> &mut crate::base::Pose {
            &mut self.0
        }
    }
    impl Source<f64> for CircularField {
        fn compute_B(&self, p: Point3<f64>) -> Vector3<f64> {
            Vector3::new(-p.y, p.x, 0.0)
        }
        fn compute_B_batch(&self, points: &[Point3<f64>]) -> Vec<Vector3<f64>> {
            points.iter().map(|&p| self.compute_B(p)).collect()
        }
    }
    let paths = CircularField(Default::default())
        .trace_field_lines(
            &[Point3::new(1.0, 0.0, 0.0)],
            FieldLineOptions {
                step_size: 0.02,
                max_steps: 1000,
                ..Default::default()
            },
        )
        .unwrap();
    let path = &paths[0];
    assert_eq!(path.first(), path.last());
    assert!(
        path.len() > 300 && path.len() < 320,
        "{} vertices",
        path.len()
    );
    assert!(path.iter().all(|p| (p.coords.norm() - 1.0).abs() < 1e-8));
}

#[test]
fn adaptive_steps_close_lines_across_a_magnet_surface() {
    let source = crate::magnets::SphereMagnet::<f64>::default().with_diameter(0.02);
    let seed = Point3::new(0.014, 0.0, 0.0);
    let paths = source
        .trace_field_lines(
            &[seed],
            FieldLineOptions {
                step_size: 0.0004,
                max_steps: 1000,
                ..Default::default()
            },
        )
        .unwrap();
    let path = &paths[0];
    assert_eq!(path.first(), path.last());
    assert!(path.iter().any(|p| p.coords.norm() < 0.009));
    assert!(path.len() < 1000);
}
