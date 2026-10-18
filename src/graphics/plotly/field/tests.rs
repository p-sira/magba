use super::*;
use crate::magnets::StableFieldMagnet;
use serde_json::{Value, json};

fn grid(plane: Plane) -> FieldGrid {
    FieldGrid::new(plane, 0.25, [-1.0, 1.0], [-2.0, 2.0], [3, 5]).unwrap()
}

#[test]
fn slices_preserve_world_components_and_plane_coordinates() {
    let source = StableFieldMagnet::new(Vector3::new(3.0, 4.0, 12.0));
    for plane in [Plane::XY, Plane::XZ, Plane::YZ] {
        let grid = grid(plane);
        assert_eq!(grid.plane(), plane);
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
    let quiver = source
        .sample_field(&grid(Plane::XY))
        .unwrap()
        .quiver(QuiverOptions::default())
        .unwrap();
    let quiver: Value = serde_json::from_str(&quiver.to_json()).unwrap();
    assert_eq!(quiver["data"], json!([]));
}

#[test]
fn parameters_are_validated_and_f32_trait_objects_work() {
    let error = FieldGrid::new(Plane::XY, 0.0, [1.0, -1.0], [0.0, 1.0], [2, 2]).unwrap_err();
    assert!(error.to_string().contains("finite"));
    let _: &dyn std::error::Error = &error;
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
fn streamline_heads_point_along_b_and_stay_in_the_projection_plane() {
    for plane in [None, Some(Plane::XY), Some(Plane::XZ), Some(Plane::YZ)] {
        let direction = plane
            .map(|p| p.point([1.0, 0.0], 0.0).coords)
            .unwrap_or(Vector3::z());
        let path = vec![
            Point3::origin(),
            Point3::from(direction * 0.01),
            Point3::from(direction),
        ];
        let mut paths = vec![path];
        add_line_arrowheads(&mut paths, 0.01, plane);
        assert_eq!(paths.len(), 3);
        for head in &paths[1..] {
            assert!((head[1] - head[0]).dot(&direction) > 0.0);
            assert!((head[1] - head[2]).dot(&direction) > 0.0);
            if let Some(plane) = plane {
                assert!(head.iter().all(|p| p[plane.axes()[2]] == 0.0));
            }
        }
        assert!((paths[1][1].coords.dot(&direction) - 1.0 / 3.0).abs() < 1e-14);
    }
    let mut paths = vec![vec![], vec![Point3::origin(), Point3::origin()]];
    add_line_arrowheads(&mut paths, 0.01, None);
    assert_eq!(paths.len(), 2);
}

#[test]
fn plotting_conveniences_keep_traces_and_respect_existing_layouts() {
    use ::plotly::common::Title;
    let source = StableFieldMagnet::new(Vector3::new(1.0, 2.0, 3.0));
    let grid = grid(Plane::YZ);
    let slice = source.sample_field(&grid).unwrap();
    assert_eq!(slice.grid().points(), grid.points());
    let mut plot = source.plot_field_2d(&grid, FieldComponent::Y).unwrap();
    let layout = plot.layout().clone().title(Title::with_text("Preserved"));
    plot.set_layout(layout.clone());
    slice
        .add_quiver(
            &mut plot,
            QuiverOptions {
                scale: 0.1,
                normalize: false,
            },
        )
        .unwrap();
    assert_eq!(
        serde_json::to_value(plot.layout()).unwrap(),
        serde_json::to_value(layout).unwrap()
    );
    let plot: Value = serde_json::from_str(&plot.to_json()).unwrap();
    assert_eq!(plot["data"].as_array().unwrap().len(), 2);
    assert_eq!(plot["data"][0]["zmid"], 0.0);
    assert_eq!(plot["data"][0]["z"][0][0], 2.0);

    let lines = FieldLineOptions {
        max_steps: 4,
        ..Default::default()
    };
    let mut plot = source.plot_field_lines(&[Point3::origin()], lines).unwrap();
    source
        .add_quiver(&mut plot, &[Point3::origin()], QuiverOptions::default())
        .unwrap();
    let plot: Value = serde_json::from_str(&plot.to_json()).unwrap();
    assert_eq!(plot["data"].as_array().unwrap().len(), 2);
    let trace = &plot["data"][0];
    // One trajectory plus two disconnected direction arrowheads.
    assert_eq!(
        trace["x"]
            .as_array()
            .unwrap()
            .iter()
            .filter(|p| p.is_null())
            .count(),
        3
    );
    assert_eq!(plot["layout"]["scene"]["aspectmode"], "data");
    // The 3D arrow perpendicular fallback is also well-defined along z.
    let vertical = StableFieldMagnet::<f64>::new(Vector3::z());
    let plot = vertical
        .plot_quiver(&[Point3::origin()], QuiverOptions::default())
        .unwrap();
    let plot: Value = serde_json::from_str(&plot.to_json()).unwrap();
    assert!(
        plot["data"][0]["x"]
            .as_array()
            .unwrap()
            .iter()
            .all(|p| p.is_null() || p.as_f64().unwrap().is_finite())
    );
}
