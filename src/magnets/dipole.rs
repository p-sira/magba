/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use nalgebra::Vector3;

use crate::crate_utils::define_source;

define_source! {
    /// Magnetic dipole source.
    ///
    /// # References
    ///
    /// - Ortner, Michael, and Lucas Gabriel Coliado Bandeira. “Magpylib: A Free Python Package for Magnetic Field Computation.” SoftwareX 11 (January 1, 2020): 100466. <https://doi.org/10.1016/j.softx.2020.100466>.
    Dipole
    field_fn: dipole_B
    args: { moment:Vector3<T> = Vector3::z() }
    arg_display: "m={}";
    arg_fmt: [format_vector3]
    docs: {
        new: {
            /// Construct a magnetic [Dipole].
            ///
            /// # Examples
            ///
            /// ```
            /// # use magba::magnets::Dipole;
            /// # use nalgebra::UnitQuaternion;
            /// let dipole = Dipole::new(
            ///     [0.0, 0.0, 0.0],              // position (m)
            ///     UnitQuaternion::identity(),   // orientation as unit quaternion
            ///     [0.0, 0.0, 1.0],              // moment: Magnetic dipole moment vector (A·m²)
            /// );
            /// ```
        }
    }
}

#[cfg(all(test, feature = "std"))]
crate::testing_util::generate_tests! {
    Dipole
    filename: dipole
    params: { moment: vector![1.0, 2.0, 3.0] }
    rtols: {
        static: 2e-10,
        static_small: 2e-10,
        translate: 2e-10,
        rotate: 2e-10,
    }
    p95_rtols: {
        static: 2e-10,
        static_small: 2e-10,
        translate: 2e-10,
        rotate: 2e-10,
    }
    f32_rtols: {
        static: 1e-6,
        static_small: 1e-6,
        translate: 1e-6,
        rotate: 1e-6,
    }
    f32_p95_rtols: {
        static: 1e-6,
        static_small: 1e-6,
        translate: 1e-6,
        rotate: 1e-6,
    }
}
