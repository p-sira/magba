/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use nalgebra::Vector3;

use crate::sources::magnets::define_magnet;

define_magnet! {
    /// Magnetic dipole in 3D space.
    ///
    /// # Fields
    /// - `position`: Position of the dipole (m)
    /// - `orientation`: Orientation as unit quaternion
    /// - `moment`: Magnetic dipole moment vector (A·m²)
    ///
    /// # Example
    /// ```
    /// use magba::sources::Dipole;
    /// use nalgebra::*;
    ///
    /// let dipole = Dipole::<f64>::new(
    ///     Point3::origin(),
    ///     UnitQuaternion::identity(),
    ///     Vector3::z(),
    /// );
    /// ```
    Dipole
    field_fn: dipole_B
    args: {moment:Vector3<T>}
    arg_display: "m={}";
    arg_fmt: [format_vector3]
    on_new: []
}
