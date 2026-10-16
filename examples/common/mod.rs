use magba::graphics::plotly::{
    Plot,
    common::{Font, Orientation, Title},
    layout::{Legend, Margin},
};

pub fn save(mut plot: Plot, title: &str, filename: &str) -> Result<(), Box<dyn std::error::Error>> {
    plot.set_layout(
        plot.layout()
            .clone()
            .title(Title::with_text(title))
            .width(1100)
            .height(800)
            .margin(Margin::new().top(140))
            .font(Font::new().family("Arial, sans-serif").size(15))
            .legend(
                Legend::new()
                    .orientation(Orientation::Horizontal)
                    .x(0.0)
                    .y(1.08),
            ),
    );
    let path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| filename.to_owned());
    std::fs::write(&path, plot.to_html())?;
    println!("Open {path} in a browser. Use the camera button to export a PNG figure.");
    Ok(())
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
