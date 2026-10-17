//! A 2×2 figure showing geometry, collections, heatmaps, quivers, and streamlines.
#[allow(dead_code)]
mod common;

use common::exterior::Exterior;

use magba::{
    graphics::{
        FieldComponent, FieldGrid, FieldLineOptions, MagnetColorScheme, Plane, QuiverOptions,
        plotly::{
            Plot, Scatter,
            common::{Line, Mode},
        },
    },
    prelude::*,
    sensors::hall_effect::LinearHallSensor,
    sources,
};
use nalgebra::Point3;
use serde_json::{Value, json};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let sphere = SphereMagnet::<f64>::default().with_diameter(0.02);
    let sphere = Exterior(sphere);
    let grid = FieldGrid::new(Plane::XZ, 0.0, [-0.035, 0.035], [-0.035, 0.035], [101, 101])?;
    let slice = sphere.sample_field(&grid)?;
    let lines = FieldLineOptions {
        step_size: 0.0004,
        max_steps: 600,
        bounds: Some([[-0.04, 0.04]; 3]),
        ..Default::default()
    };
    let seeds: Vec<_> = (0..6)
        .flat_map(|i| {
            let angle = std::f64::consts::TAU * i as f64 / 6.0;
            [0.014, 0.025].map(|r| Point3::new(r * angle.cos(), r * angle.sin(), 0.0))
        })
        .collect();
    let mut geometry = sphere.plot();
    sphere.add_field_lines(&mut geometry, &seeds, lines)?;
    LinearHallSensor::<f64>::default()
        .with_position([0.0, 0.0, 0.025])
        .add_to_plot(&mut geometry);

    let mut magnitude = slice.heatmap(FieldComponent::Magnitude);
    let seeds_2d: Vec<_> = [-1.0, 1.0]
        .into_iter()
        .flat_map(|sign| [0.014, 0.025].map(|x| [sign * x, 0.0]))
        .collect();
    sphere.add_field_lines_2d(&mut magnitude, &grid, &seeds_2d, lines)?;
    let angles: Vec<_> = (0..=100)
        .map(|i| std::f64::consts::TAU * i as f64 / 100.0)
        .collect();
    magnitude.add_trace(
        Scatter::new(
            angles.iter().map(|t| 0.01 * t.cos()).collect(),
            angles.iter().map(|t| 0.01 * t.sin()).collect(),
        )
        .mode(Mode::Lines)
        .line(Line::new().color("white").width(2.0))
        .show_legend(false),
    );

    let mut components = slice.heatmap(FieldComponent::Z);
    let arrows = FieldGrid::new(Plane::XZ, 0.0, [-0.032, 0.032], [-0.032, 0.032], [17, 17])?;
    sphere.sample_field(&arrows)?.add_quiver(
        &mut components,
        QuiverOptions {
            scale: 0.003,
            normalize: true,
        },
    )?;

    let collection = sources!(
        CuboidMagnet::<f64>::default()
            .with_dimensions([0.012, 0.012, 0.02])
            .with_position([-0.015, 0.0, 0.0]),
        CylinderMagnet::default()
            .with_diameter(0.012)
            .with_height(0.02)
            .with_polarization([0.0, 0.0, -1.0])
            .with_position([0.015, 0.0, 0.0]),
        CircularCurrent::default()
            .with_diameter(0.06)
            .with_current(10.0),
    );
    let collection = Exterior(collection);
    let mut combined = collection.plot_with_color_scheme(MagnetColorScheme::OkabeIto);
    let arrows = FieldGrid::new(Plane::XY, 0.025, [-0.03, 0.03], [-0.03, 0.03], [6, 6])?;
    collection.add_quiver(
        &mut combined,
        &arrows.points(),
        QuiverOptions {
            scale: 0.006,
            normalize: true,
        },
    )?;
    let html = subplot_html([geometry, magnitude, components, combined])?;
    common::save_html(&html, "graphics_showcase.html")
}

/// Compose native Plotly figures without resampling or retracing their fields.
/// Plotly.rs does not yet expose 3D scene domains, so use Plotly.js layout JSON
/// for the mixed Cartesian/3D subplot arrangement. Plotly.js stays embedded offline.
fn subplot_html(panels: [Plot; 4]) -> Result<String, Box<dyn std::error::Error>> {
    let mut data = Vec::new();
    let mut layout = json!({
        "title": {"text": "Magba · Magnetic fields in four views", "x": 0.5},
        "width": 1400, "height": 1100,
        "margin": {"l": 75, "r": 110, "t": 100, "b": 90},
        "font": {"family": "Arial, sans-serif", "size": 14},
        "showlegend": false,
        "annotations": [],
    });
    let titles = [
        "3D geometry and directed field lines",
        "Field magnitude and streamlines",
        "Bz and in-plane field arrows",
        "Collection and 3D field arrows · Okabe–Ito",
    ];
    for (i, panel) in panels.into_iter().enumerate() {
        let panel: Value = serde_json::from_str(&panel.to_json())?;
        let x = if i % 2 == 0 {
            [0.0, 0.41]
        } else {
            [0.57, 0.98]
        };
        let y = if i < 2 { [0.58, 1.0] } else { [0.0, 0.42] };
        let scene = if i == 0 { "scene" } else { "scene2" };
        let (xa, ya, xlayout, ylayout) = if i == 1 {
            ("x", "y", "xaxis", "yaxis")
        } else {
            ("x2", "y2", "xaxis2", "yaxis2")
        };
        if i == 0 || i == 3 {
            layout[scene] = panel["layout"]["scene"].clone();
            layout[scene]["domain"] = json!({"x": x, "y": y});
            layout[scene]["camera"] = json!({"eye": {"x": 1.7, "y": 1.7, "z": 1.4}});
        } else {
            layout[xlayout] = panel["layout"]["xaxis"].clone();
            layout[ylayout] = panel["layout"]["yaxis"].clone();
            layout[xlayout]["domain"] = json!(x);
            layout[ylayout]["domain"] = json!(y);
            layout[xlayout]["anchor"] = json!(ya);
            layout[ylayout]["anchor"] = json!(xa);
            layout[ylayout]["scaleanchor"] = json!(xa);
            layout[xlayout]["range"] = json!([-0.035, 0.035]);
            layout[ylayout]["range"] = json!([-0.035, 0.035]);
        }
        for mut trace in panel["data"]
            .as_array()
            .ok_or("Missing panel traces")?
            .iter()
            .cloned()
        {
            if i == 0 || i == 3 {
                trace["scene"] = json!(scene);
            } else {
                trace["xaxis"] = json!(xa);
                trace["yaxis"] = json!(ya);
                if trace["type"] == "heatmap" {
                    trace["colorbar"]["x"] = json!(x[1] + 0.015);
                    trace["colorbar"]["y"] = json!((y[0] + y[1]) / 2.0);
                    trace["colorbar"]["len"] = json!(0.38);
                    trace["colorbar"]["thickness"] = json!(14);
                    // Clip only the display scale so interior fields don't obscure the exterior.
                    if i == 1 {
                        trace["zmin"] = json!(0.0);
                        trace["zmax"] = json!(0.2);
                    } else {
                        trace["zmin"] = json!(-0.15);
                        trace["zmax"] = json!(0.15);
                    }
                }
            }
            data.push(trace);
        }
        layout["annotations"]
            .as_array_mut()
            .ok_or("Missing annotations")?
            .push(json!({
                "text": titles[i], "x": (x[0]+x[1])/2.0, "y": y[1]+0.035,
                "xref": "paper", "yref": "paper", "showarrow": false,
                "xanchor": "center", "font": {"size": 18},
            }));
    }
    let figure = json!({"data": data, "layout": layout});
    let script = format!(
        r#"<script>
window.addEventListener('load', async () => {{
    const figure = {figure};
    await Plotly.react(document.querySelector('.js-plotly-plot'), figure.data, figure.layout, {{responsive: true}});
    window.magbaShowcaseReady = true;
}});
</script></body>"#
    );
    Ok(Plot::new().to_html().replace("</body>", &script))
}
