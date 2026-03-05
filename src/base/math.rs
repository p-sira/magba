/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

const MU0: f64 = 1.2566370614359173e-6;

/// Generic trait for floating point numbers compatible with all [Magba](crate) implementations.
///
/// Supports [f32] and [f64]. Magba defaults to `f64`. You can explicitly use `f32` for improved
/// performance at the cost of precision.
///
/// # Examples
///
/// Constants: Permeability of free space (μ₀)
/// ```
/// # use magba::base::Float;
/// assert_eq!(f64::mu0(), 1.2566370614359173e-6);
/// assert_eq!(f32::mu0(), 1.256637e-6);
///
/// assert_ne!(f64::mu0(), f32::mu0() as f64);
/// ```
///
/// Using in magnetic field calculations
/// ```
/// # use magba::prelude::*;
/// # use nalgebra::vector;
/// # use approx::{assert_relative_eq, assert_relative_ne};
/// let cylinder_f64 = CylinderMagnet::<f64>::default();
/// let cylinder_f32 = CylinderMagnet::<f32>::default();
///
/// let field_f64 = cylinder_f64.compute_B([0.0, 0.0, 0.5].into());
/// let field_f32 = cylinder_f32.compute_B([0.0, 0.0, 0.5].into());
///
/// assert_relative_eq!(field_f64, vector![0.0, 0.0, 0.4472135954999579]);
/// assert_relative_eq!(field_f32, vector![0.0, 0.0, 0.4472136]);
///
/// assert_relative_ne!(field_f64, field_f32.cast::<f64>());
/// ```
pub trait Float: nalgebra::RealField + num_traits::Float + Copy {
    /// Permeability of free space (μ₀) = 4π × 10⁻⁷ H/m.
    fn mu0() -> Self;
}

impl Float for f32 {
    #[inline]
    fn mu0() -> Self {
        MU0 as f32
    }
}

impl Float for f64 {
    #[inline]
    fn mu0() -> Self {
        MU0
    }
}
