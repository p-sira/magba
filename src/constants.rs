/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Physical constants used in magnetic field calculations.

/// Permeability of free space (μ₀) = 4π × 10⁻⁷ H/m.
pub const MU0: f64 = 4e-7 * std::f64::consts::PI;

pub trait MagneticConstants {
    /// Permeability of free space (μ₀) = 4π × 10⁻⁷ H/m.
    fn mu0() -> Self;
}

macro_rules! impl_magnetic_constants {
    ($t: ident) => {
        impl MagneticConstants for $t {
            #[inline]
            fn mu0() -> $t {
                4e-7 as $t * std::$t::consts::PI
            }
        }
    };
}

impl_magnetic_constants!(f32);
impl_magnetic_constants!(f64);
