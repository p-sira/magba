/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2026 Sira Pornsiriprasert <code@psira.me>
 */

use getset::{CopyGetters, Setters, WithSetters};

use super::MagnetColorScheme;

/// Appearance shared by object renderers, independently of physical properties.
///
/// Default options preserve each object's usual opacity. Set an explicit override
/// with [`with_opacity(0.5)`](Self::with_opacity). Options propagate through collections; field plots
/// retain their own sampling and rendering settings.
///
/// ```
/// use magba::graphics::{GraphicsOptions, MagnetColorScheme};
/// let options = GraphicsOptions::default()
///     .with_color_scheme(MagnetColorScheme::OkabeIto)
///     .with_opacity(0.5)
///     .with_segments(96);
/// assert_eq!(options.opacity(), Some(0.5));
/// ```
#[derive(Clone, Copy, Debug, PartialEq, CopyGetters, Setters, WithSetters)]
#[getset(get_copy = "pub", set = "pub", set_with = "pub")]
pub struct GraphicsOptions {
    /// Palette used to shade magnetic poles. Defaults to DIN red/green.
    color_scheme: MagnetColorScheme,
    /// Optional trace opacity override. Defaults to each renderer's usual opacity.
    #[getset(skip)]
    opacity: Option<f64>,
    /// Whether to emit traces. Defaults to true.
    visible: bool,
    /// Circular subdivisions, at least six. Defaults to 48.
    /// Spheres use half this count for latitude subdivisions.
    segments: usize,
}

impl Default for GraphicsOptions {
    fn default() -> Self {
        Self {
            color_scheme: MagnetColorScheme::default(),
            opacity: None,
            visible: true,
            segments: 48,
        }
    }
}

impl GraphicsOptions {
    /// Return the explicit opacity override, if any.
    pub fn opacity(&self) -> Option<f64> {
        self.opacity
    }

    /// Override opacity for every trace. Validated when rendering.
    pub fn set_opacity(&mut self, opacity: f64) {
        self.opacity = Some(opacity);
    }

    /// Override opacity for every trace, consuming these options.
    pub fn with_opacity(mut self, opacity: f64) -> Self {
        self.set_opacity(opacity);
        self
    }

    /// Validate before rendering, even when visibility is disabled.
    ///
    /// # Panics
    /// Panics if opacity is nonfinite or outside `[0, 1]`, or segments is below six.
    pub fn validate(&self) {
        assert!(
            self.opacity
                .is_none_or(|v| v.is_finite() && (0.0..=1.0).contains(&v)),
            "Graphics opacity must be finite and between zero and one."
        );
        assert!(
            self.segments >= 6,
            "Graphics segments must be at least six."
        );
    }
}

/// A borrowed graphical object together with reusable appearance options.
///
/// Stores any object without depending on a rendering backend. Rendering
/// methods are supplied by enabled backends. The object cannot be mutated while the view's shared borrow remains in use.
///
/// ```
/// use magba::{graphics::GraphicsView, prelude::*};
/// let magnet = CylinderMagnet::<f32>::default();
/// let view = GraphicsView::new(&magnet).with_opacity(0.5).with_segments(64);
/// assert!(core::ptr::eq(view.object(), &magnet));
/// assert_eq!(view.options().opacity(), Some(0.5));
/// ```
#[derive(Clone, Debug)]
pub struct GraphicsView<'a, S: ?Sized> {
    object: &'a S,
    options: GraphicsOptions,
}

impl<'a, S: ?Sized> GraphicsView<'a, S> {
    /// Borrow an object using default rendering options.
    pub fn new(object: &'a S) -> Self {
        Self {
            object,
            options: GraphicsOptions::default(),
        }
    }

    /// Return this view's appearance settings.
    pub fn options(&self) -> &GraphicsOptions {
        &self.options
    }

    /// Replace this view's appearance settings.
    pub fn with_options(mut self, options: GraphicsOptions) -> Self {
        self.options = options;
        self
    }

    /// Choose the magnet pole palette.
    pub fn with_color_scheme(mut self, scheme: MagnetColorScheme) -> Self {
        self.options.set_color_scheme(scheme);
        self
    }

    /// Set the opacity override using a value between zero and one.
    pub fn with_opacity(mut self, opacity: f64) -> Self {
        self.options.set_opacity(opacity);
        self
    }

    /// Choose whether this view emits traces.
    pub fn with_visible(mut self, visible: bool) -> Self {
        self.options.set_visible(visible);
        self
    }

    /// Set circular subdivisions, at least six.
    pub fn with_segments(mut self, segments: usize) -> Self {
        self.options.set_segments(segments);
        self
    }

    /// Return the borrowed object for rendering with another backend.
    pub fn object(&self) -> &'a S {
        self.object
    }
}
