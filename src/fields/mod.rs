/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Analytical computation of magnetic fields for various source geometries.
//!
//! ## Item requiring `unstable` features
//! - [field_cuboid]
//! - [field_cylinder]
//! - [field_dipole]

crate::crate_util::pub_on_feature! {
    "unstable", mod {
        field_cuboid, field_cylinder, field_dipole
    }
}

pub use field_cuboid::{cuboid_B, sum_multiple_cuboid_B};
pub use field_cylinder::{cylinder_B, sum_multiple_cylinder_B};
pub use field_dipole::{dipole_B, sum_multiple_dipole_B};
