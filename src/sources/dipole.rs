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
    ///     Point3::origin(),           // position (m)
    ///     UnitQuaternion::identity(), // orientation
    ///     Vector3::z(),               // moment (A·m²)
    /// );
    /// ```
    ///
    /// # References
    /// - Ortner, Michael, and Lucas Gabriel Coliado Bandeira. “Magpylib: A Free Python Package for Magnetic Field Computation.” SoftwareX 11 (January 1, 2020): 100466. <https://doi.org/10.1016/j.softx.2020.100466>.
    Dipole
    field_fn: dipole_B
    args: { moment:Vector3<T> = Vector3::z() }
    arg_display: "m={}";
    arg_fmt: [format_vector3]
    on_new: []
}

#[cfg(test)]
crate::testing_util::generate_tests! {
    Dipole
    filename: dipole
    params: { moment: Vector3::new(1.0, 2.0, 3.0) }
    rtols: {
        static: 2e-10,
        static_small: 2e-10,
        translate: 2e-10,
        rotate: 2e-10,
    }
}
