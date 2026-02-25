/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use crate::magnets::define_magnet::define_magnet;

define_magnet! {
    /// Placeholder magnet type with zero field.
    ///
    /// # Fields
    ///
    /// - `position`: Position of the dipole (m)
    /// - `orientation`: Orientation as unit quaternion
    ///
    /// # Examples
    ///
    /// ```
    /// # use magba::*;
    /// # use nalgebra::{Point3, Vector3};
    /// let zero: ZeroMagnet = ZeroMagnet::default();
    /// assert_eq!(zero.compute_B(Point3::origin()), Vector3::zeros());
    /// ```
    ZeroMagnet
    field_fn: zero_field
    args: {}
    arg_display: "Placeholder";
    arg_fmt: []
}
