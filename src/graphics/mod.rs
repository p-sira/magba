/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2026 Sira Pornsiriprasert <code@psira.me>
 */

//! Backend-independent field sampling, grids, and adaptive trajectory integration.
//!
//! Available with `std`. Enable `graphics-plotly` for interactive Plotly rendering
//! through the `plotly` module. Future renderers can reuse `FieldSampler` and
//! `FieldSlice` without depending on Plotly.

mod field;
pub use field::{
    FieldComponent, FieldGrid, FieldLineOptions, FieldSampler, FieldSlice, GraphicsError, Plane,
    QuiverOptions,
};

#[cfg(feature = "graphics-plotly")]
pub mod plotly;
#[cfg(feature = "graphics-plotly")]
pub use plotly::{FieldGraphics, Graphics};

/// Pole colors used for magnet surfaces. The uncharged body is always grey.
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
    pub(super) fn poles(self) -> (&'static str, &'static str) {
        match self {
            Self::Din => ("#e71111", "#00b050"),
            Self::OkabeIto => ("#E69F00", "#0072B2"),
            Self::Trad => ("red", "blue"),
        }
    }
}
