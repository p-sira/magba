use super::*;
use crate::magnets::{Dipole, StableFieldMagnet};

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
