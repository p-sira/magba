/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use nalgebra::Vector3;

use crate::crate_utils::define_source;

define_source! {
    /// Tetrahedron with homogeneous magnetic surface charge.
    TetrahedronMagnet
    field_fn: tetrahedron_B
    args: {
        polarization: Vector3<T> = Vector3::z(),
        vertices: @val [Vector3<T>; 4] = [Vector3::zeros(), Vector3::x(), Vector3::y(), Vector3::z()],
    }
    arg_display: "pol={}, vertices={}";
    arg_fmt: [format_vector3, format_vertices_4]
    docs: {
        new: {
            /// Construct a [TetrahedronMagnet].
            ///
            /// # Examples
            ///
            /// ```
            /// # use magba::magnets::TetrahedronMagnet;
            /// # use nalgebra::{UnitQuaternion, vector};
            /// let magnet = TetrahedronMagnet::new(
            ///     [0.0, 0.0, 0.0],              // position (m)
            ///     UnitQuaternion::identity(),   // orientation
            ///     [0.0, 0.0, 1.0],              // polarization (T)
            ///     [                             // vertices (m)
            ///         vector![-0.1, -0.1, -0.1],
            ///         vector![0.1, -0.1, -0.1],
            ///         vector![0.0, 0.1, -0.1],
            ///         vector![0.0, 0.0, 0.1],
            ///     ],
            /// );
            /// ```
        }
    }
}

#[cfg(all(test, feature = "std"))]
crate::testing_util::generate_tests! {
    TetrahedronMagnet
    filename: tetrahedron
    params: {
        polarization: vector![1.0, 2.0, 3.0],
        vertices: [
            vector![-0.1, -0.1, -0.1],
            vector![0.1, -0.1, -0.1],
            vector![0.0, 0.1, -0.1],
            vector![0.0, 0.0, 0.1]
        ]
    }
    rtols: {
        static: 2e-9,
        static_small: 2e-9,
        translate: 2e-9,
        rotate: 2e-9,
    }
    f32_rtols: {
        static: 5e-2,
        static_small: 2e-1,
        translate: 5e-2,
        rotate: 5e-2,
    }
}
