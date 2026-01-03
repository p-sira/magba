/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use nalgebra::Vector3;

use crate::sources::magnets::define_magnet;

define_magnet! {
    /// Uniformly magnetized cylindrical magnet in 3D space.
    ///
    /// # Fields
    /// - `position`: Center of the cylinder (m)
    /// - `orientation`: Orientation as unit quaternion
    /// - `polarization`: Polarization vector (T)
    /// - `diameter`: Cylinder diameter (m)
    /// - `height`: Cylinder height (m)
    ///
    /// # Example
    /// ```
    /// use magba::sources::CylinderMagnet;
    /// use nalgebra::*;
    ///
    /// let magnet = CylinderMagnet::new(
    ///     Point3::origin(),           // position (m)
    ///     UnitQuaternion::identity(), // orientation
    ///     Vector3::z(),               // polarization (T)
    ///     0.01,                       // diameter (m)
    ///     0.02,                       // height (m)
    /// );
    /// ```
    /// # References
    /// - Caciagli, Alessio, Roel J. Baars, Albert P. Philipse, and Bonny W. M. Kuipers. “Exact Expression for the Magnetic Field of a Finite Cylinder with Arbitrary Uniform Magnetization.” Journal of Magnetism and Magnetic Materials 456 (June 15, 2018): 423–32. <https://doi.org/10.1016/j.jmmm.2018.02.003>.
    /// - Derby, Norman, and Stanislaw Olbert. “Cylindrical Magnets and Ideal Solenoids.” American Journal of Physics 78, no. 3 (March 1, 2010): 229–35. <https://doi.org/10.1119/1.3256157>.
    /// - Ortner, Michael, and Lucas Gabriel Coliado Bandeira. “Magpylib: A Free Python Package for Magnetic Field Computation.” SoftwareX 11 (January 1, 2020): 100466. <https://doi.org/10.1016/j.softx.2020.100466>.
    CylinderMagnet
    field_fn: cylinder_B
    args: {polarization:Vector3<T> = Vector3::z(), diameter v:T = T::one(), height v:T = T::one()}
    arg_display: "pol={}, d={}, h={}";
    arg_fmt: [format_vector3, format_float, format_float]
    on_new: [
        if diameter < T::zero() || height < T::zero() {
            panic!("diameter and height cannot be negative.")
        }
    ]
}

#[cfg(test)]
crate::testing_util::generate_tests! {
    CylinderMagnet
    filename: cylinder
    params: { polarization: Vector3::new(1.0, 2.0, 3.0), d: 0.1, h: 0.2}
    rtols: {
        static: 5e-10,
        static_small: 5e-10,
        translate: 2e-10,
        rotate: 2e-10,
    }
}
