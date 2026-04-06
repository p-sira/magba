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
//! The field computation functions are available in three variants:
//!
//! - **Non-batch functions** (e.g., [cuboid_B], [dipole_B]):
//!   Compute the magnetic field at a single observation point.
//! - **Batch functions** (e.g., [cuboid_B_batch], [dipole_B_batch]):
//!   Compute the magnetic field at multiple observation points simultaneously.
//! - **Sum multiple functions** (e.g., [sum_multiple_cuboid_B], [sum_multiple_dipole_B]):
//!   Compute the magnetic field at a single observation point by summing the
//!   fields of multiple magnets.
//!
//! ## Parallelization
//!
//! Batch and sum multiple functions are parallelized if the `rayon` feature is
//! enabled (default). If the `rayon` feature is disabled, the batch functions
//! will process the points serially on a single thread.
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

mod field_circular;
mod field_cuboid;
mod field_cylinder;
mod field_dipole;
mod field_sphere;
mod field_tetrahedron;
mod field_triangle;

pub use field_circular::{circular_B, circular_B_batch, sum_multiple_circular_B};
pub use field_cuboid::{cuboid_B, cuboid_B_batch, sum_multiple_cuboid_B};
pub use field_cylinder::{cylinder_B, cylinder_B_batch, sum_multiple_cylinder_B};
pub use field_dipole::{dipole_B, dipole_B_batch, sum_multiple_dipole_B};
pub use field_sphere::{sphere_B, sphere_B_batch, sum_multiple_sphere_B};
pub use field_tetrahedron::{sum_multiple_tetrahedron_B, tetrahedron_B, tetrahedron_B_batch};
pub use field_triangle::{sum_multiple_triangle_B, triangle_B, triangle_B_batch};

#[cfg(feature = "mesh")]
mod field_mesh;
#[cfg(feature = "mesh")]
pub use field_mesh::{mesh_B, mesh_B_batch, sum_multiple_mesh_B};

crate::crate_utils::need_unstable! {
    pub use field_cuboid::local_cuboid_B;
    pub use field_cylinder::{
        cylinder_B_cyl, local_cylinder_B, unit_axial_cylinder_B_cyl, unit_diametric_cylinder_B_cyl,
    };
    pub use field_dipole::local_dipole_B;
    pub use field_sphere::local_sphere_B;
    pub use field_tetrahedron::local_tetrahedron_B;
    pub use field_triangle::local_triangle_B;

    #[cfg(feature = "mesh")]
    pub use field_mesh::local_mesh_B;
}
