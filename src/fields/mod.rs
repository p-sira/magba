/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Analytical computation of magnetic fields for various source geometries.
//!
//! # Argument Conventions
//!
//! The arguments in the field computation functions follow the order of
//! observer position(s), magnet position, magnet orientation, and the other
//! arguments needed for the specific geometry. For the batch functions, the
//! last argument is a slice for receiving the output. This is to avoid
//! allocations and enable use in `no-alloc` contexts.
//!
//! # Batch and Non-Batch Functions
//!
//! Field functions have two variants:
//!
//! - **Non-batch functions** (e.g., [cuboid_B], [dipole_B]):
//!   Compute the magnetic field at a single observation point.
//! - **Batch functions** (e.g., [cuboid_B_batch], [dipole_B_batch]):
//!   Compute the magnetic field at multiple observation points simultaneously.
//!
//! ## Parallelization
//!
//! Batch functions use Rayon when enabled (default) and the work exceeds their
//! parallelization thresholds. Otherwise, they evaluate points serially.
//!
//! To tune Rayon thresholds locally, run
//! `cargo bench --bench threshold --features unstable,io-stl`, inspect
//! `benches/par_threshold.md`, then edit the matching `*_B_batch` thresholds.
//! Path, mesh, and sheet-current thresholds scale with per-point work.
//!
//! # Examples
//!
//! ```
//! # use magba::fields::*;
//! # use magba::prelude::*;
//! # use nalgebra::{point, vector, UnitQuaternion};
//! let field = cuboid_B(
//!     point![0.0, 0.0, 0.0],      // Observer position (m)
//!     point![0.0, 0.0, 0.0],      // Magnet position (m)
//!     UnitQuaternion::identity(), // Magnet orientation as unit quaternion
//!     vector![0.0, 0.0, 1.0],     // Magnet polarization (T)
//!     vector![0.01, 0.01, 0.02],  // Magnet dimensions (m)
//! );
//! ```
//!
//! # Internal Functions (`unstable`)
//!
//! The following functions with `unstable` badges are internal
//! functions. They are subject to change without prior notice.
//! However, you can access them via the `unstable` feature flag:
//!
//! ```bash
//! cargo add magba --features unstable
//! ```

mod bulirsch_cel_axial;

mod field_circular;
mod field_cuboid;
mod field_cylinder;
mod field_dipole;
#[cfg(feature = "alloc")]
mod field_path_current;
#[cfg(feature = "mesh")]
mod field_sheet_current;
mod field_sphere;
mod field_tetrahedron;
mod field_triangle;
mod field_triangle_current;

pub use field_circular::{circular_B, circular_B_batch};
pub use field_cuboid::{cuboid_B, cuboid_B_batch};
pub use field_cylinder::{cylinder_B, cylinder_B_batch};
pub use field_dipole::{dipole_B, dipole_B_batch};
#[cfg(feature = "alloc")]
pub use field_path_current::{path_current_B, path_current_B_batch};
#[cfg(feature = "mesh")]
pub use field_sheet_current::{sheet_current_B, sheet_current_B_batch};
pub use field_sphere::{sphere_B, sphere_B_batch};
pub use field_tetrahedron::{tetrahedron_B, tetrahedron_B_batch};
pub use field_triangle::{triangle_B, triangle_B_batch};
pub use field_triangle_current::{triangle_current_B, triangle_current_B_batch};

#[cfg(feature = "mesh")]
mod field_mesh;
#[cfg(feature = "mesh")]
pub use field_mesh::{mesh_B, mesh_B_batch};

crate::crate_utils::need_unstable! {
    pub use field_cuboid::local_cuboid_B;
    pub use field_cylinder::{
        cylinder_B_cyl, local_cylinder_B, unit_axial_cylinder_B_cyl, unit_diametric_cylinder_B_cyl,
    };
    pub use field_dipole::local_dipole_B;
    pub use field_sphere::local_sphere_B;
    pub use field_tetrahedron::{local_tetrahedron_B, tetrahedron_B_precomputed, precompute_tetrahedron};
    pub use field_triangle::local_triangle_B;

    #[cfg(feature = "mesh")]
    pub use field_mesh::local_mesh_B;

    #[cfg(feature = "alloc")]
    pub use field_path_current::local_path_current_B;
    pub use field_triangle_current::local_triangle_current_B;
    #[cfg(feature = "mesh")]
    pub use field_sheet_current::local_sheet_current_B;
}
