/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2026 Sira Pornsiriprasert <code@psira.me>
 */

//! Backend-independent field sampling, grids, and adaptive trajectory integration.
//!
//! Available with `std`. Enable `graphics-plotly` for interactive Plotly rendering
//! through the `plotly` module. Future renderers can reuse `FieldSampler` and
//! `FieldSlice` without depending on Plotly.
//!
//! With `graphics-plotly`, `GraphicsOptions` controls object appearance and
//! `GraphicsView` stores options alongside a borrowed graphical object. See
//! `graphics::plotly` for scenes, custom collection geometry, and field-plot examples.

mod field;
pub use field::{
    FieldComponent, FieldGrid, FieldLineOptions, FieldSampler, FieldSlice, GraphicsError, Plane,
    QuiverOptions,
};

#[cfg(feature = "graphics-plotly")]
pub mod plotly;
#[cfg(feature = "graphics-plotly")]
pub use plotly::{
    FieldGraphics, Graphics, GraphicsObserver, GraphicsOptions, GraphicsSource, GraphicsView,
};

/// Pole colors used for magnet surfaces, fading to grey as surface charge vanishes.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum MagnetColorScheme {
    /// Red north and green south, following Magpylib's DIN SPEC 91411 palette.
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
