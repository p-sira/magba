/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use nalgebra::Vector3;

use crate::crate_utils::define_source;

define_source! {
    /// A single triangular current sheet with homogeneous surface current density.
    TriangleCurrent
    field_fn: triangle_current_B
    args: {
        current_density: Vector3<T> = Vector3::zeros(),
        vertices: @val [Vector3<T>; 3] = [Vector3::x(), Vector3::y(), Vector3::zeros()],
    }
    arg_display: "current_density={}, vertices={}";
    arg_fmt: [format_vector3, format_vertices]
    docs: {
        new: {
            /// Construct a [TriangleCurrent].
            ///
            /// # Examples
            ///
            /// ```
            /// # use magba::currents::TriangleCurrent;
            /// # use nalgebra::{UnitQuaternion, vector};
            /// let sheet = TriangleCurrent::new(
            ///     [0.0, 0.0, 0.0],              // position (m)
            ///     UnitQuaternion::identity(),   // orientation
            ///     [1.0, 2.0, 3.0],              // current density (A/m)
            ///     [                             // vertices (m)
            ///         vector![0.0, 0.0, 0.0],
            ///         vector![1.0, 0.0, 0.0],
            ///         vector![0.0, 1.0, 0.0],
            ///     ],
            /// );
            /// ```
        }
    }
}

#[cfg(all(test, feature = "std"))]
crate::testing_util::generate_tests! {
    TriangleCurrent
    filename: trianglecurrent
    params: {
        current_density: vector![1.0, 2.0, 3.0],
        vertices: [
            vector![-0.1, -0.1, -0.1],
            vector![0.1, -0.1, -0.1],
            vector![0.0, 0.1, -0.1],
        ],
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
        static: 2e-2,
        static_small: 5e-2,
        translate: 2e-2,
        rotate: 2e-2,
    }
    f32_p95_rtols: {
        static: 1e-4,
        static_small: 1e-4,
        translate: 1e-4,
        rotate: 1e-4,
    }
}
