/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Analytical computation of magnetic fields for various source geometries.

pub mod field_cylinder;
pub use field_cylinder::{cylinder_B, sum_multiple_cylinder_B};

pub mod field_cuboid;
pub use field_cuboid::{cuboid_B, sum_multiple_cuboid_B};

pub mod field_dipole;
pub use field_dipole::{dipole_B, sum_multiple_dipole_B};
