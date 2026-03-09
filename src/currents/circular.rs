/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use crate::crate_utils::define_source;

define_source! {
    /// Circular current source.
    ///
    /// # References
    ///
    /// - Ortner, Michael, and Lucas Gabriel Coliado Bandeira. “Magpylib: A Free Python Package for Magnetic Field Computation.” SoftwareX 11 (January 1, 2020): 100466. <https://doi.org/10.1016/j.softx.2020.100466>.
    CircularCurrent
    field_fn: circular_B
    args: {
        diameter: T = T::one();
            validate diameter > T::zero();
            error "Diameter must be positive.",
        current: T = T::one(),
    }
    arg_display: "d={}, I={}";
    arg_fmt: [format_float, format_float]

    docs: {
        new: {
            /// Create a new circular current source.
            ///
            /// # Parameters
            ///
            /// - `position`: Position in units (m)
            /// - `orientation`: Orientation as unit quaternion
            /// - `diameter`: Loop diameter (m)
            /// - `current`: Electrical current (A)
        }
    }
}

#[cfg(all(test, feature = "std"))]
crate::testing_util::generate_tests! {
    CircularCurrent
    filename: circularcurrent
    params: { diameter: 1.0, current: 1.0 }
    rtols: {
        static: 2e-10,
        static_small: 2e-10,
        translate: 2e-10,
        rotate: 2e-10,
    }
}
