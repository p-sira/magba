/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use nalgebra::Vector3;

use crate::crate_utils::define_source;

define_source! {
    /// Uniformly magnetized spherical magnet.
    ///
    /// # References
    ///
    /// - Ortner, Michael, and Lucas Gabriel Coliado Bandeira. “Magpylib: A Free Python Package for Magnetic Field Computation.” SoftwareX 11 (January 1, 2020): 100466. <https://doi.org/10.1016/j.softx.2020.100466>.
    SphereMagnet
    field_fn: sphere_B
    args: {
        polarization: Vector3<T> = Vector3::z(),
        diameter: T = T::one();
            validate diameter > T::zero();
            error "Diameter cannot be negative.",
    }
    arg_display: "pol={}, d={}";
    arg_fmt: [format_vector3, format_float]
    docs: {
        new: {
            /// Construct a [SphereMagnet].
            ///
            /// # Examples
            ///
            /// ```
            /// # use magba::magnets::SphereMagnet;
            /// # use nalgebra::*;
            /// let magnet = SphereMagnet::new(
            ///     [0.0, 0.0, 0.0],              // position: Center of the sphere (m)
            ///     UnitQuaternion::identity(),   // orientation as unit quaternion
            ///     [0.0, 0.0, 1.0],              // polarization (T)
            ///     0.01,                         // diameter (m)
            /// );
            /// ```
        }
    }
}

#[cfg(all(test, feature = "std"))]
crate::testing_util::generate_tests! {
    SphereMagnet
    filename: sphere
    params: { polarization: vector![1.0, 2.0, 3.0], d: 0.1 }
    rtols: {
        static: 1e-10,
        static_small: 1e-10,
        translate: 1e-10,
        rotate: 1e-10,
    }
}

#[cfg(test)]
mod tests {
    use crate::magnets::SphereMagnet;

    #[test]
    #[should_panic]
    fn test_input_validation() {
        let _: SphereMagnet = SphereMagnet::default().with_diameter(-1.0_f64);
    }
}
