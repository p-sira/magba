mod common;

use magba::{
    graphics::MagnetColorScheme, prelude::*, sensors::hall_effect::LinearHallSensor, sources,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let sources = sources!(
        CylinderMagnet::<f64>::default().with_position([-1.0, 0.0, 0.0]),
        CuboidMagnet::default().with_position([1.0, 0.0, 0.0]),
        CircularCurrent::default().with_diameter(4.0),
    );
    let mut plot = sources.plot();
    LinearHallSensor::<f64>::default()
        .with_position([0.0, 0.0, 1.5])
        .add_to_plot(&mut plot);
    // Optional pole palettes; ordinary plot() uses red/green (DIN).
    SphereMagnet::<f64>::default()
        .with_position([-1.0, 2.0, 0.0])
        .add_to_plot_with_color_scheme(&mut plot, MagnetColorScheme::OkabeIto);
    CuboidMagnet::<f64>::default()
        .with_position([1.0, 2.0, 0.0])
        .add_to_plot_with_color_scheme(&mut plot, MagnetColorScheme::Trad);
    common::save(plot, "Sources and magnet pole palettes", "magba.html")
}
