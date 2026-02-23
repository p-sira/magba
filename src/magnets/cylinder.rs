/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use nalgebra::Vector3;

use crate::magnets::define_magnet;

define_magnet! {
    /// Uniformly magnetized cylindrical magnet.
    ///
    /// # Fields
    ///
    /// - `position`: Center of the cylinder (m)
    /// - `orientation`: Orientation as unit quaternion
    /// - `polarization`: Polarization vector (T)
    /// - `diameter`: Cylinder diameter (m)
    /// - `height`: Cylinder height (m)
    ///
    /// # Examples
    ///
    /// ```
    /// # use magba::CylinderMagnet;
    /// # use nalgebra::*;
    /// let magnet = CylinderMagnet::new(
    ///     [0.0, 0.0, 0.0],              // position (m)
    ///     UnitQuaternion::identity(),   // orientation
    ///     [0.0, 0.0, 1.0],              // polarization (T)
    ///     0.01,                         // diameter (m)
    ///     0.02,                         // height (m)
    /// );
    /// ```
    ///
    /// # References
    ///
    /// - Caciagli, Alessio, Roel J. Baars, Albert P. Philipse, and Bonny W. M. Kuipers. “Exact Expression for the Magnetic Field of a Finite Cylinder with Arbitrary Uniform Magnetization.” Journal of Magnetism and Magnetic Materials 456 (June 15, 2018): 423–32. <https://doi.org/10.1016/j.jmmm.2018.02.003>.
    /// - Derby, Norman, and Stanislaw Olbert. “Cylindrical Magnets and Ideal Solenoids.” American Journal of Physics 78, no. 3 (March 1, 2010): 229–35. <https://doi.org/10.1119/1.3256157>.
    /// - Ortner, Michael, and Lucas Gabriel Coliado Bandeira. “Magpylib: A Free Python Package for Magnetic Field Computation.” SoftwareX 11 (January 1, 2020): 100466. <https://doi.org/10.1016/j.softx.2020.100466>.
    CylinderMagnet
    field_fn: cylinder_B
    args: {
        polarization: Vector3<T> = Vector3::z(),
        diameter: @val T = T::one(); where diameter > T::zero(); else "Diameter cannot be negative.",
        height: @val T = T::one(); where height > T::zero(); else "Height cannot be negative.",
    }
    arg_display: "pol={}, d={}, h={}";
    arg_fmt: [format_vector3, format_float, format_float]
}

#[cfg(test)]
crate::testing_util::generate_tests! {
    CylinderMagnet
    filename: cylinder
    params: { polarization: vector![1.0, 2.0, 3.0], d: 0.1, h: 0.2}
    rtols: {
        static: 5e-10,
        static_small: 5e-10,
        translate: 2e-10,
        rotate: 2e-10,
    }
}

#[cfg(test)]
mod tests {
    use crate::magnets::CylinderMagnet;

    #[test]
    #[should_panic]
    fn test_input_validation() {
        CylinderMagnet::default().with_diameter(-1.0_f64);
    }
}
