//! A 2×2 figure showing geometry, collections, heatmaps, quivers, and streamlines.
#[allow(dead_code)]
mod common;

use common::exterior::exterior;

use magba::{
    graphics::{
        FieldComponent, FieldGrid, FieldLineOptions, Plane, QuiverOptions,
        plotly::{
            Plot, Scatter,
            common::{Line, Mode},
        },
    },
    prelude::*,
    sources,
};
use serde_json::json;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let sphere = SphereMagnet::<f64>::default().with_diameter(0.02);
    let sphere = exterior(sphere);
    let grid = FieldGrid::new(Plane::XZ, 0.0, [-0.035, 0.035], [-0.035, 0.035], [181, 181])?;
    let slice = sphere.sample_field(&grid)?;
    let lines = FieldLineOptions {
        step_size: 0.0004,
        max_steps: 600,
        bounds: Some([[-0.04, 0.04]; 3]),
        ..Default::default()
    };
    let seeds = common::sphere_field_seeds(6);
    let mut geometry = sphere.plot();
    sphere.add_field_lines(&mut geometry, &seeds, lines)?;

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
    components.add_trace(
        Scatter::new(
            angles.iter().map(|t| 0.01 * t.cos()).collect(),
            angles.iter().map(|t| 0.01 * t.sin()).collect(),
        )
        .mode(Mode::Lines)
        .line(Line::new().color("white").width(2.0))
        .show_legend(false),
    );
    let arrows = FieldGrid::new(Plane::XZ, 0.0, [-0.032, 0.032], [-0.032, 0.032], [13, 13])?;
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
    let collection = exterior(collection);
    let mut combined = collection.plot();
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
        "title": {
            "text": "Magba · Magnetic fields in four views", "x": 0.5,
            "font": {"size": 30, "color": "#263445"},
        },
        "width": 1400, "height": 1100,
        "margin": {"l": 75, "r": 110, "t": 100, "b": 90},
        "font": {"family": "Arial, sans-serif", "size": 17, "color": "#263445"},
        "showlegend": false,
        "paper_bgcolor": "white", "plot_bgcolor": "#f8fafc",
        "annotations": [],
    });
    let titles = [
        "<b>A</b>  Sphere field lines",
        "<b>B</b>  Magnitude and streamlines",
        "<b>C</b>  Axial field and direction",
        "<b>D</b>  Combined sources and field",
    ];
    for (i, panel) in panels.into_iter().enumerate() {
        let panel = common::styled_figure(&panel)?;
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
            layout[scene]["camera"]["center"]["z"] = json!(-0.16);
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
                    // Clip only the display scale so near-surface peaks don't obscure weaker fields.
                    if i == 1 {
                        trace["zmin"] = json!(0.0);
                        trace["zmax"] = json!(0.2);
                        trace["colorbar"]["tickvals"] = json!([0, 0.05, 0.1, 0.15, 0.2]);
                        trace["colorbar"]["ticktext"] =
                            json!(["0", "0.05", "0.10", "0.15", "≥0.20"]);
                    } else {
                        trace["zmin"] = json!(-0.15);
                        trace["zmax"] = json!(0.15);
                        trace["colorscale"] = json!([
                            [0.0, "#2166ac"],
                            [0.25, "#92c5de"],
                            [0.5, "#f7f7f7"],
                            [0.75, "#f4a582"],
                            [1.0, "#b2182b"]
                        ]);
                        trace["colorbar"]["tickvals"] = json!([-0.15, 0, 0.15]);
                        trace["colorbar"]["ticktext"] = json!(["≤−0.15", "0", "≥0.15"]);
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
                "xanchor": "center", "font": {"size": 22},
            }));
    }
    let figure = json!({"data": data, "layout": layout});
    Ok(common::figure_html(&figure))
}
