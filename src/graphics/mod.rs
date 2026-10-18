/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2026 Sira Pornsiriprasert <code@psira.me>
 */

//! Backend-independent field sampling, grids, and adaptive trajectory integration.
//!
//! Available with `graphics`, which enables `std`. Enable `graphics-plotly` for
//! interactive Plotly rendering.
//! Future renderers can reuse [`FieldSampler`] and [`FieldSlice`] without depending
//! on Plotly.
//!
//! [`GraphicsOptions`] controls object appearance and [`GraphicsView`] stores
//! options alongside a borrowed object. Neither type depends on Plotly.
//!
//! # Example
//!
//! Enable `graphics-plotly` to create a scene with a magnet and a current loop:
//!
//! ```no_run
//! # fn main() -> Result<(), Box<dyn std::error::Error>> {
//! # #[cfg(feature = "graphics-plotly")]
//! # {
//! use magba::prelude::*;
//!
//! let magnet = CuboidMagnet::<f64>::default();
//! let mut plot = magnet.plot();
//! CircularCurrent::<f64>::default()
//!     .with_position([0.0, 0.0, 2.0])
//!     .add_to_plot(&mut plot);
//! std::fs::write("scene.html", plot.to_html())?;
//! # }
//! # Ok(())
//! # }
//! ```
//!
//! ![Cuboid magnet and circular current scene](https://github.com/p-sira/magba/blob/main/examples/figures/rustdoc_scene.png?raw=true)

#![cfg_attr(
    feature = "graphics-plotly",
    doc = "See [`plotly`] for scenes, custom collection geometry, and field-plot examples."
)]

mod options;
pub use options::{GraphicsOptions, GraphicsView};

mod field;
pub use field::{
    FieldComponent, FieldGrid, FieldLineOptions, FieldSampler, FieldSlice, GraphicsError, Plane,
    QuiverOptions,
};

#[cfg(feature = "graphics-plotly")]
pub mod plotly;
#[cfg(feature = "graphics-plotly")]
pub use plotly::{FieldGraphics, Graphics, GraphicsObserver, GraphicsSource};

/// Pole colors used for magnet surfaces, fading through a grey central belt.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum MagnetColorScheme {
    /// Red north and green south, following DIN SPEC 91411.
    #[default]
    Din,
    /// Orange north and blue south from the Okabe–Ito palette (`okabe-ito`).
    OkabeIto,
    /// Traditional red north and blue south (`trad`).
    Trad,
}

#[cfg(feature = "graphics-plotly")]
impl MagnetColorScheme {
    pub(super) fn poles(self) -> [(&'static str, [u8; 3]); 2] {
        match self {
            Self::Din => [("#e71111", [231, 17, 17]), ("#00b050", [0, 176, 80])],
            Self::OkabeIto => [("#E69F00", [230, 159, 0]), ("#0072B2", [0, 114, 178])],
            Self::Trad => [("red", [255, 0, 0]), ("blue", [0, 0, 255])],
        }
    }
}
