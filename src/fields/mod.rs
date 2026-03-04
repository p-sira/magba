/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Analytical computation of magnetic fields for various source geometries.

mod field_cuboid;
mod field_cylinder;
mod field_dipole;

pub use field_cuboid::{cuboid_B, cuboid_B_batch, sum_multiple_cuboid_B};
pub use field_cylinder::{cylinder_B, cylinder_B_batch, sum_multiple_cylinder_B};
pub use field_dipole::{dipole_B, dipole_B_batch, sum_multiple_dipole_B};

#[cfg(feature = "unstable")]
pub mod unstable {
    use super::*;
    pub use field_cuboid::local_cuboid_B;
    pub use field_cylinder::{
        cylinder_B_cyl, local_cylinder_B, unit_axial_cylinder_B_cyl, unit_diametric_cylinder_B_cyl,
    };
    pub use field_dipole::local_dipole_B;
}
#[cfg(feature = "unstable")]
pub use unstable::*;
