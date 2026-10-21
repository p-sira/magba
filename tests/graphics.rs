#![cfg(feature = "graphics-plotly")]

use magba::{
    graphics::{Graphics, GraphicsOptions, GraphicsView, plotly::Plot},
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
fn hidden_objects_preserve_existing_plot_traces_and_layout() {
    fn check(object: &impl Graphics) {
        let mut plot = CuboidMagnet::<f64>::default().plot();
        let before = plot.to_json();
        object.add_to_plot_with_options(&mut plot, &GraphicsOptions::default().with_visible(false));
        assert_eq!(plot.to_json(), before);
    }

    check(&CuboidMagnet::<f64>::default());
    check(&CylinderMagnet::<f64>::default());
    check(&SphereMagnet::<f64>::default());
    check(&TetrahedronMagnet::<f64>::default());
    check(&TriangleMagnet::<f64>::default());
    check(&Dipole::<f64>::default());
    check(&CircularCurrent::<f64>::default());
    check(&TriangleCurrent::<f64>::default());
    check(&PathCurrent::<f64>::default());
    check(&LinearHallSensor::<f64>::default());
    check(&HallSwitch::<f64>::default());
    check(&HallLatch::<f64>::default());
    check(&Magnet::from(CuboidMagnet::<f64>::default()));
    check(&Current::from(CircularCurrent::<f64>::default()));
    check(&Sensor::from(LinearHallSensor::<f64>::default()));
    check(&SourceComponent::from(Dipole::<f64>::default()));
    check(&ObserverComponent::from(HallLatch::<f64>::default()));
    check(&SourceAssembly::from([Dipole::<f64>::default()]));
    check(&ObserverAssembly::from([HallLatch::<f64>::default()]));
    check(&SourceArray::from([Dipole::<f64>::default()]));
    check(&ObserverArray::from([HallLatch::<f64>::default()]));
    #[cfg(feature = "mesh")]
    {
        check(&MeshMagnet::<f64>::default());
        check(&SheetCurrent::<f64>::default());
    }
}

#[test]
fn boxed_graphics_objects_delegate_pose_mutation_and_debug_formatting() {
    use magba::{
        base::Pose,
        graphics::{GraphicsObserver, GraphicsSource},
    };

    let mut source: Box<dyn GraphicsSource> = Box::new(Dipole::<f64>::default());
    source.pose_mut().set_position([1.0, 2.0, 3.0]);
    let expected = Dipole::<f64>::default().with_position([1.0, 2.0, 3.0]);
    assert_eq!(format!("{source:?}"), format!("{expected}"));
    let point = nalgebra::point![4.0, 5.0, 6.0];
    assert_eq!(source.compute_B(point), expected.compute_B(point));

    let mut observer: Box<dyn GraphicsObserver> = Box::new(LinearHallSensor::<f64>::default());
    observer.pose_mut().set_position([1.0, 2.0, 3.0]);
    let expected = LinearHallSensor::<f64>::default().with_position([1.0, 2.0, 3.0]);
    assert_eq!(format!("{observer:?}"), format!("{expected}"));
    assert_eq!(observer.read(&source), expected.read(&source));
    source.set_pose(Pose::default());
    observer.set_pose(Pose::default());
    assert_eq!(*source.pose(), Pose::default());
    assert_eq!(*observer.pose(), Pose::default());
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
    let mut sheet = SheetCurrent::<f64>::default().with_position([2.0, 3.0, 4.0]);
    sheet.set_mesh_and_current_densities(mesh, vec![Vector3::x()]);
    let options = GraphicsOptions::default().with_opacity(0.25);
    for object in [&magnet as &dyn Graphics, &sheet] {
        let plot: Value =
            serde_json::from_str(&object.plot_with_options(&options).to_json()).unwrap();
        assert_eq!(plot["data"][0]["opacity"], 0.25);
    }
    for plot in [data(&magnet), data(&sheet)] {
        assert_eq!(plot["data"][0]["x"], json!([2.0, 3.0, 2.0]));
        assert_eq!(plot["data"][0]["i"], json!([0]));
        assert_eq!(plot["data"][0]["j"], json!([1]));
        assert_eq!(plot["data"][0]["k"], json!([2]));
    }
}

#[test]
fn magnet_poles_follow_local_polarization_across_face_boundaries() {
    let magnet = CuboidMagnet::<f64>::default();
    let plot = data(&magnet);
    let mesh = &plot["data"][0];
    let intensities = &mesh["intensity"];
    assert_eq!(
        *intensities,
        json!([-1.0, -1.0, -1.0, -1.0, 1.0, 1.0, 1.0, 1.0])
    );
    assert_eq!(mesh["intensitymode"], "vertex");
    assert_eq!(mesh["cmin"], -1.0);
    assert_eq!(mesh["cmax"], 1.0);
    assert_eq!(mesh["showscale"], false);
    assert!(mesh["facecolor"].is_null());
    assert!(mesh["vertexcolor"].is_null());
    // Side triangles span both poles. Native intensity interpolation passes
    // through the grey center, using the original mesh without extra faces.
    assert_eq!(
        mesh["intensity"][mesh["i"][4].as_u64().unwrap() as usize],
        -1.0
    );
    assert_eq!(
        mesh["intensity"][mesh["k"][4].as_u64().unwrap() as usize],
        1.0
    );

    let rotated = magnet
        .clone()
        .with_orientation(UnitQuaternion::from_axis_angle(
            &Vector3::y_axis(),
            std::f64::consts::FRAC_PI_2,
        ))
        .with_position([10.0, 20.0, 30.0]);
    assert_eq!(data(&rotated)["data"][0]["intensity"], *intensities);
    let reversed = data(&magnet.clone().with_polarization([0.0, 0.0, -1.0]));
    for (original, reversed) in intensities
        .as_array()
        .unwrap()
        .iter()
        .zip(reversed["data"][0]["intensity"].as_array().unwrap())
    {
        assert_eq!(original.as_f64().unwrap(), -reversed.as_f64().unwrap());
    }
    let transverse = data(&magnet.clone().with_polarization([1.0, 0.0, 0.0]));
    assert_eq!(
        transverse["data"][0]["intensity"],
        json!([-1.0, 1.0, 1.0, -1.0, -1.0, 1.0, 1.0, -1.0])
    );
    let neutral = data(&magnet.with_polarization([0.0; 3]));
    assert!(
        neutral["data"][0]["intensity"]
            .as_array()
            .unwrap()
            .iter()
            .all(|v| v == 0.0)
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
        let plot: Value = serde_json::from_str(
            &assembly
                .plot_with_options(&GraphicsOptions::default().with_color_scheme(scheme))
                .to_json(),
        )
        .unwrap();
        for trace in plot["data"].as_array().unwrap() {
            let colors = trace["colorscale"].as_array().unwrap();
            assert_eq!(colors.first().unwrap()[1], south);
            assert_eq!(colors.last().unwrap()[1], north);
            assert!(colors.iter().any(|c| c[1] == "grey"));
        }
        let array = SourceArray::from([SphereMagnet::<f64>::default()]);
        let plot: Value = serde_json::from_str(
            &array
                .plot_with_options(&GraphicsOptions::default().with_color_scheme(scheme))
                .to_json(),
        )
        .unwrap();
        let colors = plot["data"][0]["colorscale"].as_array().unwrap();
        assert_eq!(colors.first().unwrap()[1], south);
        assert_eq!(colors.last().unwrap()[1], north);
        assert!(
            colors
                .iter()
                .any(|c| c[1].as_str().unwrap().starts_with("rgb("))
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
        let options = GraphicsOptions::default().with_color_scheme(scheme);
        let plot: Value =
            serde_json::from_str(&magnet.plot_with_options(&options).to_json()).unwrap();
        let colors = plot["data"][0]["colorscale"].as_array().unwrap();
        assert!(colors.contains(&json!([0.75, north])));
        assert!(colors.contains(&json!([0.25, south])));
        for stop in [0.425, 0.5, 0.575] {
            assert!(colors.contains(&json!([stop, "grey"])));
        }
        // A tilted polarization changes where the gradient crosses each face.
        for i in 0..8 {
            let x = plot["data"][0]["x"][i].as_f64().unwrap();
            let z = plot["data"][0]["z"][i].as_f64().unwrap();
            let intensity = plot["data"][0]["intensity"][i].as_f64().unwrap();
            assert!((intensity - (0.5 * x + z) / 0.75).abs() < 1e-12);
        }
        let scaled = magnet.clone().with_polarization([5.0, 0.0, 10.0]);
        let scaled = data(&scaled);
        for (a, b) in plot["data"][0]["intensity"]
            .as_array()
            .unwrap()
            .iter()
            .zip(scaled["data"][0]["intensity"].as_array().unwrap())
        {
            assert!((a.as_f64().unwrap() - b.as_f64().unwrap()).abs() < 1e-12);
        }
    }
}

#[test]
fn sphere_intensities_interpolate_and_follow_local_polarization() {
    let magnet = SphereMagnet::<f64>::default();
    let plot = data(&magnet);
    let mesh = &plot["data"][0];
    let colors = mesh["intensity"].as_array().unwrap();
    assert_eq!(colors.len(), mesh["x"].as_array().unwrap().len());
    assert!(mesh["facecolor"].is_null());
    assert_eq!(colors.first().unwrap(), 1.0);
    assert_eq!(colors.last().unwrap(), -1.0);
    assert!(colors[1 + 11 * 48].as_f64().unwrap().abs() < 1e-12);
    let rotated = magnet
        .clone()
        .with_orientation(UnitQuaternion::from_axis_angle(
            &Vector3::y_axis(),
            std::f64::consts::FRAC_PI_2,
        ));
    assert_eq!(data(&rotated)["data"][0]["intensity"], mesh["intensity"]);
    let neutral = magnet.with_polarization([0.0; 3]);
    assert!(
        data(&neutral)["data"][0]["intensity"]
            .as_array()
            .unwrap()
            .iter()
            .all(|c| c == 0.0)
    );
}

#[test]
fn tetrahedron_poles_are_independent_of_vertex_winding() {
    let magnet = TetrahedronMagnet::<f64>::default();
    let plot = data(&magnet);
    let colors = plot["data"][0]["intensity"].as_array().unwrap();
    assert!(colors.iter().any(|c| c == 1.0));
    assert!(colors.iter().any(|c| c == -1.0));
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
            mesh["colorscale"]
                .as_array()
                .unwrap()
                .iter()
                .any(|stop| stop[1] == "#e71111")
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
    let mut sheet = SheetCurrent::<f64>::default();
    sheet.set_mesh_and_current_densities(mesh, vec![Vector3::x()]);
    let current = Current::from(sheet);
    assert_eq!(data(&magnet)["data"][0]["opacity"], 1.0);
    assert_eq!(data(&current)["data"][0]["opacity"], 0.65);
    assert_eq!(data(&MeshMagnet::<f64>::default())["data"], json!([]));
    assert_eq!(data(&SheetCurrent::<f64>::default())["data"], json!([]));
}

#[test]
fn options_and_views_propagate_through_all_geometry_kinds() {
    let sources = sources!(
        sources!(CylinderMagnet::<f64>::default()),
        CircularCurrent::default(),
        Dipole::default(),
    );
    let mut options = GraphicsOptions::default().with_segments(12);
    options.set_opacity(0.4);
    options.set_color_scheme(magba::graphics::MagnetColorScheme::Trad);
    let expected = sources.plot_with_options(&options).to_json();
    let view = GraphicsView::new(&sources)
        .with_segments(12)
        .with_opacity(0.4)
        .with_color_scheme(magba::graphics::MagnetColorScheme::Trad);
    assert_eq!(view.plot().to_json(), expected);
    assert_eq!(view.options(), &options);
    let value: Value = serde_json::from_str(&expected).unwrap();
    let traces = value["data"].as_array().unwrap();
    assert_eq!(traces.len(), 3);
    for trace in traces {
        assert_eq!(trace["opacity"], 0.4);
    }
    assert_eq!(traces[0]["x"].as_array().unwrap().len(), 26);
    assert_eq!(traces[1]["x"].as_array().unwrap().len(), 13);
    assert!(
        traces[0]["colorscale"]
            .as_array()
            .unwrap()
            .contains(&json!([1.0, "red"]))
    );

    let mut plot = sources.plot();
    let original = plot.to_json();
    GraphicsView::new(&sources)
        .with_visible(false)
        .add_to_plot(&mut plot);
    assert_eq!(plot.to_json(), original);
    let hidden = options.with_visible(false);
    sources.add_to_plot_with_options(&mut plot, &hidden);
    assert_eq!(plot.to_json(), original);
    assert_eq!(
        serde_json::from_str::<Value>(&sources.plot_with_options(&hidden).to_json()).unwrap()["data"],
        json!([])
    );

    let array = SourceArray::from([CuboidMagnet::<f64>::default()]);
    let observers = ObserverArray::from([LinearHallSensor::<f64>::default()]);
    let observer_assembly = ObserverAssembly::from([LinearHallSensor::<f64>::default()]);
    for object in [&array as &dyn Graphics, &observers, &observer_assembly] {
        let plot = object.plot_with_options(&options);
        let value: Value = serde_json::from_str(&plot.to_json()).unwrap();
        assert_eq!(value["data"][0]["opacity"], 0.4);
        assert_eq!(
            object.plot().to_json(),
            object
                .plot_with_options(&GraphicsOptions::default())
                .to_json()
        );
    }

    let sphere = SphereMagnet::<f32>::default();
    let erased: &dyn Graphics<f32> = &sphere;
    let view = GraphicsView::new(erased).with_options(options.with_segments(6));
    let value: Value = serde_json::from_str(&view.plot().to_json()).unwrap();
    assert_eq!(value["data"][0]["x"].as_array().unwrap().len(), 14);
    let mut appended = Plot::new();
    view.add_to_plot(&mut appended);
    assert_eq!(
        serde_json::from_str::<Value>(&appended.to_json()).unwrap()["data"],
        value["data"]
    );
}

#[test]
fn invalid_options_are_rejected_before_any_traces_are_added() {
    for options in [
        GraphicsOptions::default().with_opacity(f64::NAN),
        GraphicsOptions::default().with_opacity(f64::INFINITY),
        GraphicsOptions::default().with_opacity(-0.1),
        GraphicsOptions::default().with_opacity(1.1),
        GraphicsOptions::default().with_segments(5),
    ] {
        for visible in [false, true] {
            let options = options.with_visible(visible);
            let mut plot = Plot::new();
            let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                SourceAssembly::<f64>::default().add_to_plot_with_options(&mut plot, &options);
            }));
            assert!(result.is_err());
            assert_eq!(
                serde_json::from_str::<Value>(&plot.to_json()).unwrap()["data"],
                json!([])
            );
            assert!(
                std::panic::catch_unwind(|| GraphicsView::new(&Dipole::<f64>::default())
                    .with_options(options)
                    .plot())
                .is_err()
            );
        }
    }
    for opacity in [0.0, 1.0] {
        GraphicsOptions::default().with_opacity(opacity).validate();
    }
}

#[derive(Clone)]
struct Custom<S>(S);

impl<S: Transform<f64>> Transform<f64> for Custom<S> {
    fn pose(&self) -> &magba::base::Pose {
        self.0.pose()
    }
    fn pose_mut(&mut self) -> &mut magba::base::Pose {
        self.0.pose_mut()
    }
    fn set_pose(&mut self, pose: magba::base::Pose) {
        self.0.set_pose(pose);
    }
}

impl<S: Source<f64> + Clone> Source<f64> for Custom<S> {
    delegate::delegate! {
        to self.0 {
            fn compute_B(&self, point: nalgebra::Point3<f64>) -> Vector3<f64>;
            fn compute_B_batch(&self, points: &[nalgebra::Point3<f64>]) -> Vec<Vector3<f64>>;
            fn relative_complexity(&self) -> usize;
            fn format(&self, f: &mut core::fmt::Formatter<'_>, indent: &str) -> core::fmt::Result;
        }
    }
}

impl<S: Observer<f64> + Clone> Observer<f64> for Custom<S> {
    fn read(&self, source: &dyn Source<f64>) -> magba::base::SensorOutput {
        self.0.read(source)
    }
    fn format(&self, f: &mut core::fmt::Formatter<'_>, indent: &str) -> core::fmt::Result {
        self.0.format(f, indent)
    }
}

impl<S: Graphics> Graphics for Custom<S> {
    fn add_to_plot_with_options(&self, plot: &mut Plot, options: &GraphicsOptions) {
        self.0.add_to_plot_with_options(plot, options);
    }
}

#[test]
fn custom_collection_geometry_survives_cloning_and_transforms() {
    let magnet = CylinderMagnet::<f64>::default();
    let source = SourceComponent::custom_graphics(Custom(magnet.clone()));
    assert_ne!(source, source.clone());
    assert_eq!(source.relative_complexity(), magnet.relative_complexity());
    let point = nalgebra::point![2.0, 3.0, 4.0];
    assert_eq!(source.compute_B(point), magnet.compute_B(point));
    assert_eq!(
        source.compute_B_batch(&[point]),
        magnet.compute_B_batch(&[point])
    );
    assert_eq!(
        format!("{}", &source as &dyn Source<f64>),
        format!("{magnet}")
    );
    let mut actual = sources!(sources!(source)).clone();
    let mut expected = sources!(sources!(magnet));
    let rotation = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.7);
    actual.rotate(rotation);
    expected.rotate(rotation);
    actual.translate([1.0, 2.0, 3.0]);
    expected.translate([1.0, 2.0, 3.0]);
    let options = GraphicsOptions::default()
        .with_opacity(0.3)
        .with_segments(10);
    assert_eq!(
        actual.plot_with_options(&options).to_json(),
        expected.plot_with_options(&options).to_json()
    );
    assert_eq!(actual.compute_B(point), expected.compute_B(point));

    let sensor = LinearHallSensor::<f64>::default();
    let observer = ObserverComponent::custom_graphics(Custom(sensor.clone()));
    assert_ne!(observer, observer.clone());
    assert_eq!(observer.read(&expected), sensor.read(&expected));
    assert_eq!(
        format!("{}", &observer as &dyn Observer<f64>),
        format!("{sensor}")
    );
    let mut actual = ObserverAssembly::from([observer]).clone();
    let mut expected_observers = ObserverAssembly::from([sensor]);
    actual.rotate(rotation);
    expected_observers.rotate(rotation);
    actual.translate([2.0, 3.0, 4.0]);
    expected_observers.translate([2.0, 3.0, 4.0]);
    assert_eq!(
        actual.plot_with_options(&options).to_json(),
        expected_observers.plot_with_options(&options).to_json()
    );
    assert_eq!(
        actual.components().next().unwrap().read(&expected),
        expected_observers
            .components()
            .next()
            .unwrap()
            .read(&expected)
    );
}

#[test]
fn solid_primitives_use_polarization_gradients_and_planar_sheets_keep_their_pole() {
    let solids = [
        Magnet::from(CylinderMagnet::<f64>::default().with_polarization([1.0, 2.0, 3.0])),
        Magnet::from(CuboidMagnet::default().with_polarization([1.0, 2.0, 3.0])),
        Magnet::from(SphereMagnet::default().with_polarization([1.0, 2.0, 3.0])),
        Magnet::from(TetrahedronMagnet::default().with_polarization([1.0, 2.0, 3.0])),
    ];
    for source in solids {
        let value = data(&source);
        let trace = &value["data"][0];
        assert_eq!(trace["intensitymode"], "vertex");
        let intensities: Vec<_> = trace["intensity"]
            .as_array()
            .unwrap()
            .iter()
            .map(|v| v.as_f64().unwrap())
            .collect();
        assert_eq!(intensities.len(), trace["x"].as_array().unwrap().len());
        assert_eq!(
            intensities.iter().copied().fold(f64::INFINITY, f64::min),
            -1.0
        );
        assert_eq!(
            intensities
                .iter()
                .copied()
                .fold(f64::NEG_INFINITY, f64::max),
            1.0
        );
        // Position along the chosen direction must determine color, independently
        // of face normals. Check against world coordinates at the identity pose.
        let projections: Vec<_> = (0..intensities.len())
            .map(|i| {
                trace["x"][i].as_f64().unwrap()
                    + 2.0 * trace["y"][i].as_f64().unwrap()
                    + 3.0 * trace["z"][i].as_f64().unwrap()
            })
            .collect();
        let min = projections.iter().copied().fold(f64::INFINITY, f64::min);
        let max = projections
            .iter()
            .copied()
            .fold(f64::NEG_INFINITY, f64::max);
        for (value, projection) in intensities.iter().zip(projections) {
            assert!((value - (2.0 * (projection - min) / (max - min) - 1.0)).abs() < 1e-12);
        }
    }
    for (polarization, pole) in [([0.0, 0.0, 1.0], 1.0), ([0.0, 0.0, -1.0], -1.0)] {
        let sheet = data(&TriangleMagnet::<f64>::default().with_polarization(polarization));
        assert_eq!(sheet["data"][0]["intensitymode"], "cell");
        assert_eq!(sheet["data"][0]["intensity"], json!([pole]));
    }
    let tilted_sheet = TriangleMagnet::<f64>::default()
        .with_vertices([
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, -1.0),
            Vector3::new(0.0, 1.0, -1.0),
        ])
        .with_polarization([1.0, 1.0, 1.0]);
    assert_eq!(data(&tilted_sheet)["data"][0]["intensitymode"], "cell");
    for magnitude in [1e-300, 1e300] {
        let tiny_or_large =
            data(&CuboidMagnet::<f64>::default().with_polarization([0.0, 0.0, magnitude]));
        assert_eq!(
            tiny_or_large["data"][0]["intensity"],
            json!([-1.0, -1.0, -1.0, -1.0, 1.0, 1.0, 1.0, 1.0])
        );
    }
    let flat = data(&CuboidMagnet::<f64>::default().with_dimensions([1.0, 1.0, 0.0]));
    assert!(
        flat["data"][0]["intensity"]
            .as_array()
            .unwrap()
            .iter()
            .all(|v| v.as_f64().unwrap().is_finite())
    );
}
