#[allow(dead_code)]
pub mod exterior;

use magba::graphics::plotly::Plot;
use serde_json::{Value, json};

/// Shared presentation for field examples; coordinates and field samples stay in SI units.
pub fn styled_figure(plot: &Plot) -> Result<Value, Box<dyn std::error::Error>> {
    let mut figure: Value = serde_json::from_str(&plot.to_json())?;
    // Leave room between the outermost ticks and the corners of the 3D box.
    for coordinate in ["x", "y", "z"] {
        let values = figure["data"]
            .as_array()
            .into_iter()
            .flatten()
            .filter_map(|trace| trace[coordinate].as_array())
            .flatten()
            .filter_map(Value::as_f64)
            .filter(|value| value.is_finite());
        let (min, max) = values.fold((f64::INFINITY, f64::NEG_INFINITY), |(min, max), value| {
            (min.min(value), max.max(value))
        });
        if figure["layout"]["scene"].is_object() && min < max {
            let padding = (max - min) * 0.08;
            figure["layout"]["scene"][format!("{coordinate}axis")]["range"] =
                json!([min - padding, max + padding]);
        }
    }
    let layout = &mut figure["layout"];
    layout["font"] = json!({"family": "Arial, sans-serif", "size": 18, "color": "#263445"});
    layout["paper_bgcolor"] = json!("white");
    if layout["xaxis"].is_object() {
        layout["plot_bgcolor"] = json!("#f8fafc");
    }
    for name in ["xaxis", "yaxis"] {
        if layout[name].is_object() {
            style_axis(&mut layout[name]);
            layout[name]["constrain"] = json!("domain");
        }
    }
    if layout["scene"].is_object() {
        let scene = &mut layout["scene"];
        scene["camera"] =
            json!({"eye": {"x": 1.4, "y": 1.4, "z": 1.2}, "center": {"x": 0, "y": 0, "z": -0.08}});
        for name in ["xaxis", "yaxis", "zaxis"] {
            style_axis(&mut scene[name]);
            scene[name]["tickfont"]["size"] = json!(13);
            scene[name]["showbackground"] = json!(true);
            scene[name]["backgroundcolor"] = json!("#f8fafc");
        }
    }
    let heatmap = figure["data"]
        .as_array()
        .is_some_and(|data| data.iter().any(|trace| trace["type"] == "heatmap"));
    if heatmap {
        for name in ["xaxis", "yaxis"] {
            figure["layout"][name]["showgrid"] = json!(false);
        }
    }
    for trace in figure["data"].as_array_mut().ok_or("Missing plot traces")? {
        match trace["name"].as_str() {
            Some("Field lines") => {
                trace["line"]["width"] = json!(if heatmap { 2.2 } else { 4.0 });
            }
            Some("B direction") => {
                trace["line"]["color"] = json!(if heatmap { "#172554" } else { "#1d4ed8" });
                trace["line"]["width"] = json!(2.2);
            }
            _ => {}
        }
        if trace["type"] == "heatmap" {
            trace["colorbar"]["x"] = json!(0.88);
            trace["colorbar"]["outlinewidth"] = json!(0);
            trace["colorbar"]["thickness"] = json!(18);
            trace["colorbar"]["tickfont"] = json!({"size": 16});
        }
    }
    Ok(figure)
}

fn style_axis(axis: &mut Value) {
    axis["showgrid"] = json!(true);
    axis["gridcolor"] = json!("#dce3eb");
    axis["zeroline"] = json!(false);
    axis["showline"] = json!(true);
    axis["linecolor"] = json!("#a8b4c3");
    axis["tickfont"] = json!({"size": 15});
    axis["tickformat"] = json!(".3~f");
    axis["nticks"] = json!(5);
    axis["title"]["font"] = json!({"size": 18});
}

#[allow(dead_code)]
pub fn save(plot: Plot, title: &str, filename: &str) -> Result<(), Box<dyn std::error::Error>> {
    let mut figure = styled_figure(&plot)?;
    let layout = &mut figure["layout"];
    layout["title"] = json!({"text": title, "x": 0.06, "xanchor": "left", "font": {"size": 25}});
    layout["width"] = json!(1100);
    layout["height"] = json!(800);
    layout["margin"] = json!({"l": 85, "r": 90, "t": 125, "b": 80});
    layout["legend"] = json!({
        "font": {"size": 17}, "orientation": "h", "x": 0.0, "y": 1.025,
        "yanchor": "bottom", "itemsizing": "constant",
    });
    save_html(&figure_html(&figure), filename)
}

/// Compact, consistently styled previews for each magnet type's documentation.
#[allow(dead_code)]
pub fn save_preview(plot: Plot, filename: &str) -> Result<(), Box<dyn std::error::Error>> {
    let mut figure = styled_figure(&plot)?;
    let layout = &mut figure["layout"];
    layout["width"] = json!(880);
    layout["height"] = json!(600);
    layout["margin"] = json!({"l": 45, "r": 45, "t": 65, "b": 45});
    layout["font"]["size"] = json!(16);
    layout["legend"] = json!({
        "font": {"size": 15}, "orientation": "h", "x": 0.0, "y": 1.03,
        "yanchor": "bottom", "itemsizing": "constant",
    });
    save_html(&figure_html(&figure), filename)
}

/// Keep Plotly embedded for offline viewing and use the same figure for PNG export.
pub fn figure_html(figure: &Value) -> String {
    let script = format!(
        r#"<script>
window.addEventListener('load', async () => {{
    const figure = {figure};
    await Plotly.react(document.querySelector('.js-plotly-plot'), figure.data, figure.layout, {{
        responsive: true,
        toImageButtonOptions: {{format: 'png', width: figure.layout.width, height: figure.layout.height, scale: 1.5}}
    }});
    window.magbaFigureReady = true;
}});
</script></body>"#
    );
    Plot::new().to_html().replace("</body>", &script)
}

pub fn save_html(html: &str, filename: &str) -> Result<(), Box<dyn std::error::Error>> {
    let path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| filename.to_owned());
    std::fs::write(&path, html)?;
    println!("Open {path} in a browser. Use the camera button to export a PNG figure.");
    Ok(())
}

/// Seed the 20 mm sphere's equatorial loops, polar fans, and central axis.
#[allow(dead_code)]
pub fn sphere_field_seeds(azimuths: usize) -> Vec<nalgebra::Point3<f64>> {
    use nalgebra::Point3;

    let polar_angle = 20.0_f64.to_radians();
    let mut seeds = Vec::new();
    for i in 0..azimuths {
        let angle = std::f64::consts::TAU * i as f64 / azimuths as f64;
        for r in [0.014, 0.035] {
            seeds.push(Point3::new(r * angle.cos(), r * angle.sin(), 0.0));
        }
    }
    for i in 0..4 {
        let angle = std::f64::consts::TAU * i as f64 / 4.0;
        for sign in [-1.0, 1.0] {
            seeds.push(Point3::new(
                0.0105 * polar_angle.sin() * angle.cos(),
                0.0105 * polar_angle.sin() * angle.sin(),
                sign * 0.0105 * polar_angle.cos(),
            ));
        }
    }
    seeds.extend([
        Point3::new(0.0, 0.0, -0.0105),
        Point3::new(0.0, 0.0, 0.0105),
    ]);
    seeds
}

/// Render a primitive's field from three rings of equatorial seeds.
#[allow(dead_code)]
pub fn field_scene(
    source: &(impl magba::graphics::Graphics + magba::graphics::FieldGraphics),
    extent: f64,
    title: &str,
    filename: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    use magba::graphics::FieldLineOptions;
    use nalgebra::Point3;

    let seeds: Vec<_> = (0..8)
        .flat_map(|i| {
            let angle = std::f64::consts::TAU * i as f64 / 8.0;
            [0.35, 0.625, 0.85]
                .map(|r| Point3::new(extent * r * angle.cos(), extent * r * angle.sin(), 0.0))
        })
        .collect();
    let mut plot = source.plot();
    source.add_field_lines(
        &mut plot,
        &seeds,
        FieldLineOptions {
            step_size: extent / 100.0,
            max_steps: 400,
            bounds: Some([[-extent, extent]; 3]),
            ..Default::default()
        },
    )?;
    save(plot, title, filename)
}
