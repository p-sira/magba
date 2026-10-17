#![cfg(feature = "graphics-plotly")]

use magba::{
    graphics::{Graphics, plotly::Plot},
    prelude::*,
    sensors::hall_effect::{HallLatch, HallSwitch, LinearHallSensor},
    sources,
};
use nalgebra::{UnitQuaternion, Vector3};
use serde_json::{Value, json};

fn data(object: &impl Graphics) -> Value {
    serde_json::from_str(&object.plot().to_json()).unwrap()
}

#[test]
fn cuboid_geometry_uses_dimensions_rotation_and_translation() {
    let magnet = CuboidMagnet::<f64>::default()
        .with_dimensions([2.0, 4.0, 6.0])
        .with_orientation(UnitQuaternion::from_axis_angle(
            &Vector3::z_axis(),
            std::f64::consts::FRAC_PI_2,
        ))
        .with_position([10.0, 20.0, 30.0]);
    let plot = data(&magnet);
    let trace = &plot["data"][0];
    for (axis, expected) in [("x", 12.0), ("y", 19.0), ("z", 27.0)] {
        assert!((trace[axis][0].as_f64().unwrap() - expected).abs() < 1e-12);
    }
    assert_eq!(trace["i"].as_array().unwrap().len(), 12);
    assert_eq!(trace["opacity"], 1.0);
    assert_eq!(plot["layout"]["scene"]["aspectmode"], "data");
    assert_eq!(plot["layout"]["scene"]["xaxis"]["title"]["text"], "x (m)");
}

#[test]
fn nested_assemblies_apply_parent_transforms_once() {
    let magnet = TriangleMagnet::<f64>::default();
    let mut assembly = sources!(sources!(magnet.clone()));
    let rotation = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.7);
    assembly.rotate(rotation);
    assembly.translate([1.0, 2.0, 3.0]);
    let expected = magnet
        .with_orientation(rotation)
        .with_position([1.0, 2.0, 3.0]);
    assert_eq!(data(&assembly)["data"], data(&expected)["data"]);
}

#[test]
fn built_in_variants_and_arrays_produce_traces() {
    let sources = sources!(
        CuboidMagnet::<f64>::default(),
        CylinderMagnet::default(),
        SphereMagnet::default(),
        Dipole::default(),
        TriangleMagnet::default(),
        TetrahedronMagnet::default(),
        CircularCurrent::default(),
        TriangleCurrent::default(),
        PathCurrent::default().with_vertices(vec![Vector3::zeros(), Vector3::x()]),
    );
    let plot = data(&sources);
    let traces = plot["data"].as_array().unwrap();
    assert_eq!(traces.len(), 9);
    for trace in traces {
        for axis in ["x", "y", "z"] {
            assert!(
                trace[axis]
                    .as_array()
                    .unwrap()
                    .iter()
                    .all(|v| v.as_f64().is_some())
            );
        }
        if trace["type"] == "mesh3d" {
            let n = trace["x"].as_array().unwrap().len() as u64;
            for axis in ["i", "j", "k"] {
                assert!(
                    trace[axis]
                        .as_array()
                        .unwrap()
                        .iter()
                        .all(|v| v.as_u64().unwrap() < n)
                );
            }
        }
    }
    let source_array = SourceArray::from([CuboidMagnet::<f64>::default(), CuboidMagnet::default()]);
    assert_eq!(data(&source_array)["data"].as_array().unwrap().len(), 2);

    let observers = ObserverAssembly::from([
        ObserverComponent::from(LinearHallSensor::<f64>::default()),
        HallSwitch::default().into(),
        HallLatch::default().into(),
    ]);
    assert_eq!(data(&observers)["data"].as_array().unwrap().len(), 3);
    let observers = ObserverArray::from([LinearHallSensor::<f64>::default()]);
    assert_eq!(data(&observers)["data"].as_array().unwrap().len(), 1);
}

#[test]
fn currents_close_loops_and_keep_paths_open() {
    let circle = data(&CircularCurrent::<f64>::default());
    for axis in ["x", "y", "z"] {
        let values = circle["data"][0][axis].as_array().unwrap();
        assert_eq!(values.first(), values.last());
    }
    let path = PathCurrent::<f64>::default().with_vertices(vec![Vector3::zeros(), Vector3::x()]);
    assert_eq!(data(&path)["data"][0]["x"], json!([0.0, 1.0]));
    assert_eq!(data(&PathCurrent::<f64>::default())["data"], json!([]));
    assert_eq!(data(&SourceAssembly::<f64>::default())["data"], json!([]));
}

#[test]
fn f32_and_custom_components_support_existing_plots() {
    let mut plot = Plot::new();
    CuboidMagnet::<f32>::default().add_to_plot(&mut plot);
    let custom = SourceComponent::Custom(Box::new(
        Dipole::<f64>::default().with_position([1.0, 2.0, 3.0]),
    ));
    custom.add_to_plot(&mut plot);
    ObserverComponent::Custom(Box::new(LinearHallSensor::<f64>::default())).add_to_plot(&mut plot);
    let value: Value = serde_json::from_str(&plot.to_json()).unwrap();
    assert_eq!(value["data"].as_array().unwrap().len(), 3);
    assert_eq!(value["data"][1]["x"], json!([1.0]));
    assert_eq!(value["data"][1]["mode"], "markers");
    let html = plot.to_html();
    assert!(html.contains("Plotly.newPlot"));
    assert!(!html.contains("src=\"https://cdn.plot.ly/"));
}

#[cfg(feature = "mesh")]
#[test]
fn mesh_objects_use_supplied_triangles_and_world_pose() {
    use magba::base::mesh::TriMesh;
    let mesh = TriMesh::new_unchecked([Vector3::zeros(), Vector3::x(), Vector3::y()], [[0, 1, 2]]);
    let magnet = MeshMagnet::<f64>::default()
        .with_mesh(mesh.clone())
        .with_position([2.0, 3.0, 4.0]);
    let sheet = SheetCurrent::<f64>::default()
        .with_mesh(mesh)
        .with_position([2.0, 3.0, 4.0]);
    for plot in [data(&magnet), data(&sheet)] {
        assert_eq!(plot["data"][0]["x"], json!([2.0, 3.0, 2.0]));
        assert_eq!(plot["data"][0]["i"], json!([0]));
        assert_eq!(plot["data"][0]["j"], json!([1]));
        assert_eq!(plot["data"][0]["k"], json!([2]));
    }
}

#[test]
fn magnet_poles_follow_local_polarization_and_keep_neutral_faces_grey() {
    let magnet = CuboidMagnet::<f64>::default();
    let colors = data(&magnet)["data"][0]["facecolor"].clone();
    assert_eq!(colors[0], "#00b050");
    assert_eq!(colors[2], "#e71111");
    assert!(colors.as_array().unwrap()[4..].iter().all(|c| c == "grey"));
    assert_eq!(data(&magnet)["data"][0]["color"], "grey");

    let rotated = magnet
        .clone()
        .with_orientation(UnitQuaternion::from_axis_angle(
            &Vector3::y_axis(),
            std::f64::consts::FRAC_PI_2,
        ));
    assert_eq!(data(&rotated)["data"][0]["facecolor"], colors);
    let reversed = magnet.clone().with_polarization([0.0, 0.0, -1.0]);
    assert_eq!(data(&reversed)["data"][0]["facecolor"][0], colors[2]);
    assert_eq!(data(&reversed)["data"][0]["facecolor"][2], colors[0]);
    let transverse = magnet.clone().with_polarization([1.0, 0.0, 0.0]);
    let transverse = data(&transverse);
    assert_eq!(transverse["data"][0]["facecolor"][0], "grey");
    assert_eq!(transverse["data"][0]["facecolor"][6], "#e71111");
    let neutral = magnet.with_polarization([0.0; 3]);
    assert!(
        data(&neutral)["data"][0]["facecolor"]
            .as_array()
            .unwrap()
            .iter()
            .all(|c| c == "grey")
    );
}

#[test]
fn optional_palettes_propagate_through_nested_collections_and_arrays() {
    use magba::graphics::MagnetColorScheme;
    let assembly = sources!(
        sources!(CuboidMagnet::<f64>::default()),
        CylinderMagnet::default()
    );
    for (scheme, north, south) in [
        (MagnetColorScheme::OkabeIto, "#E69F00", "#0072B2"),
        (MagnetColorScheme::Trad, "red", "blue"),
    ] {
        let plot: Value =
            serde_json::from_str(&assembly.plot_with_color_scheme(scheme).to_json()).unwrap();
        for trace in plot["data"].as_array().unwrap() {
            let colors = trace["facecolor"].as_array().unwrap();
            for color in [north, south, "grey"] {
                assert!(colors.iter().any(|c| c == color));
            }
        }
        let array = SourceArray::from([SphereMagnet::<f64>::default()]);
        let plot: Value =
            serde_json::from_str(&array.plot_with_color_scheme(scheme).to_json()).unwrap();
        let colors = plot["data"][0]["vertexcolor"].as_array().unwrap();
        assert!(colors.iter().any(|c| c == north));
        assert!(colors.iter().any(|c| c == south));
        assert!(
            colors
                .iter()
                .any(|c| c.as_str().unwrap().starts_with("rgb("))
        );
    }
}

#[test]
fn pole_colors_ease_from_a_grey_belt_to_full_color() {
    use magba::graphics::MagnetColorScheme;

    let magnet = CuboidMagnet::<f64>::default().with_polarization([0.5, 0.0, 1.0]);
    for (scheme, north, south) in [
        (MagnetColorScheme::Din, "rgb(166,87,87)", "rgb(81,146,110)"),
        (
            MagnetColorScheme::OkabeIto,
            "rgb(166,139,81)",
            "rgb(81,123,146)",
        ),
        (MagnetColorScheme::Trad, "rgb(175,81,81)", "rgb(81,81,175)"),
    ] {
        let plot: Value =
            serde_json::from_str(&magnet.plot_with_color_scheme(scheme).to_json()).unwrap();
        let colors = plot["data"][0]["facecolor"].as_array().unwrap();
        assert_eq!(colors[6], north);
        assert!(colors.iter().any(|color| color == south));
        assert!(colors.iter().any(|color| color == "grey"));
        let scaled = magnet.clone().with_polarization([5.0, 0.0, 10.0]);
        let scaled_plot: Value =
            serde_json::from_str(&scaled.plot_with_color_scheme(scheme).to_json()).unwrap();
        assert_eq!(
            scaled_plot["data"][0]["facecolor"],
            plot["data"][0]["facecolor"]
        );
        let near_neutral = magnet.clone().with_polarization([0.1, 0.0, 1.0]);
        let neutral_plot: Value =
            serde_json::from_str(&near_neutral.plot_with_color_scheme(scheme).to_json()).unwrap();
        assert_eq!(neutral_plot["data"][0]["facecolor"][6], "grey");
        assert_eq!(neutral_plot["data"][0]["facecolor"][10], "grey");
        assert_eq!(
            neutral_plot["data"][0]["facecolor"][2],
            plot["data"][0]["facecolor"][2]
        );
    }
}

#[test]
fn sphere_colors_interpolate_between_vertices_and_follow_local_polarization() {
    let magnet = SphereMagnet::<f64>::default();
    let plot = data(&magnet);
    let mesh = &plot["data"][0];
    let colors = mesh["vertexcolor"].as_array().unwrap();
    assert_eq!(colors.len(), mesh["x"].as_array().unwrap().len());
    assert!(mesh["facecolor"].is_null());
    assert_eq!(colors.first().unwrap(), "#e71111");
    assert_eq!(colors.last().unwrap(), "#00b050");
    assert_eq!(colors[1 + 11 * 48], "grey");
    let rotated = magnet
        .clone()
        .with_orientation(UnitQuaternion::from_axis_angle(
            &Vector3::y_axis(),
            std::f64::consts::FRAC_PI_2,
        ));
    assert_eq!(
        data(&rotated)["data"][0]["vertexcolor"],
        mesh["vertexcolor"]
    );
    let neutral = magnet.with_polarization([0.0; 3]);
    assert!(
        data(&neutral)["data"][0]["vertexcolor"]
            .as_array()
            .unwrap()
            .iter()
            .all(|c| c == "grey")
    );
}

#[test]
fn tetrahedron_poles_are_independent_of_vertex_winding() {
    let magnet = TetrahedronMagnet::<f64>::default();
    let plot = data(&magnet);
    let colors = plot["data"][0]["facecolor"].as_array().unwrap();
    assert!(colors.iter().any(|c| c == "#e71111"));
    assert!(colors.iter().any(|c| c == "#00b050"));
    let mut vertices = magnet.vertices();
    vertices.swap(2, 3);
    assert_eq!(
        data(&magnet)["data"],
        data(&magnet.clone().with_vertices(vertices))["data"]
    );
}

#[test]
fn curved_magnets_and_enum_entry_points_render_solid_geometry() {
    let cylinder = CylinderMagnet::<f64>::default()
        .with_diameter(2.0)
        .with_height(4.0);
    let sphere = SphereMagnet::<f64>::default().with_diameter(2.0);
    for plot in [
        data(&cylinder),
        data(&sphere),
        data(&Magnet::from(cylinder)),
        data(&Magnet::from(sphere)),
    ] {
        let mesh = &plot["data"][0];
        assert_eq!(mesh["opacity"], 1.0);
        assert_eq!(
            mesh["x"]
                .as_array()
                .unwrap()
                .iter()
                .filter_map(Value::as_f64)
                .fold(f64::NEG_INFINITY, f64::max),
            1.0
        );
        assert!(
            mesh[if mesh["vertexcolor"].is_array() {
                "vertexcolor"
            } else {
                "facecolor"
            }]
            .as_array()
            .unwrap()
            .iter()
            .any(|color| color == "#e71111")
        );
    }
    assert_eq!(
        data(&Current::from(CircularCurrent::<f64>::default()))["data"][0]["type"],
        "scatter3d"
    );
    assert_eq!(
        data(&Sensor::from(LinearHallSensor::<f64>::default()))["data"][0]["mode"],
        "markers"
    );
}

#[cfg(feature = "mesh")]
#[test]
fn mesh_enums_render_faces_and_empty_meshes_are_omitted() {
    use magba::base::mesh::TriMesh;
    let mesh = TriMesh::new_unchecked([Vector3::zeros(), Vector3::x(), Vector3::y()], [[0, 1, 2]]);
    let magnet = Magnet::from(MeshMagnet::<f64>::default().with_mesh(mesh.clone()));
    let current = Current::from(SheetCurrent::<f64>::default().with_mesh(mesh));
    assert_eq!(data(&magnet)["data"][0]["opacity"], 1.0);
    assert_eq!(data(&current)["data"][0]["opacity"], 0.65);
    assert_eq!(data(&MeshMagnet::<f64>::default())["data"], json!([]));
    assert_eq!(data(&SheetCurrent::<f64>::default())["data"], json!([]));
}
