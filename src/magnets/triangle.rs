/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use nalgebra::Vector3;

use crate::crate_utils::define_source;

define_source! {
    /// Triangular surface with homogeneous magnetic surface charge.
    ///
    /// The charge is proportional to the projection of the polarization vectors onto the
    /// triangle surface. The order of the triangle vertices defines the sign of the
    /// surface normal vector (right-hand-rule).
    ///
    /// # References
    ///
    /// - Guptasarma, D., and B. Singh. "New scheme for computing the magnetic field resulting from a uniformly magnetized arbitrary polyhedron." Geophysics 64.1 (1999): 70-74.
    /// - Ortner, Michael, and Lucas Gabriel Coliado Bandeira. “Magpylib: A Free Python Package for Magnetic Field Computation.” SoftwareX 11 (January 1, 2020): 100466. <https://doi.org/10.1016/j.softx.2020.100466>.
    TriangleMagnet
    field_fn: triangle_B
    args: {
        polarization: Vector3<T> = Vector3::z(),
        vertices: @val [Vector3<T>; 3] = [Vector3::x(), Vector3::y(), Vector3::zeros()],
    }
    arg_display: "pol={}, vertices={}";
    arg_fmt: [format_vector3, format_vertices]
    docs: {
        new: {
            /// Construct a [TriangleMagnet].
            ///
            /// # Examples
            ///
            /// ```
            /// # use magba::magnets::TriangleMagnet;
            /// # use nalgebra::{UnitQuaternion, vector};
            /// let magnet = TriangleMagnet::new(
            ///     [0.0, 0.0, 0.0],              // position (m)
            ///     UnitQuaternion::identity(),   // orientation
            ///     [0.0, 0.0, 1.0],              // polarization (T)
            ///     [                             // vertices (m)
            ///         vector![-0.1, -0.1, -0.1],
            ///         vector![0.1, -0.1, 0.1],
            ///         vector![0.0, 0.2, 0.0],
            ///     ],
            /// );
            /// ```
        }
    }
}

#[cfg(all(test, feature = "std"))]
crate::testing_util::generate_tests! {
    TriangleMagnet
    filename: triangle
    params: {
        polarization: vector![1.0, 2.0, 3.0],
        vertices: [vector![-0.1, -0.1, -0.1], vector![0.1, -0.1, 0.1], vector![0.0, 0.2, 0.0]]
    }
    rtols: {
        static: 1e-10,
        static_small: 1e-10,
        translate: 1e-10,
        rotate: 1e-10,
    }
    p95_rtols: {
        static: 1e-10,
        static_small: 1e-10,
        translate: 1e-10,
        rotate: 1e-10,
    }
    f32_rtols: {
        static: 1e-3,
        static_small: 1e-3,
        translate: 1e-3,
        rotate: 2e-3,
    }
    f32_p95_rtols: {
        static: 1e-3,
        static_small: 1e-3,
        translate: 1e-3,
        rotate: 1e-3,
    }
}
