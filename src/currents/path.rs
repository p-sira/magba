/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use alloc::vec::Vec;
use nalgebra::Vector3;

use crate::crate_utils::define_source;

define_source! {
    /// A current path modeling a sequence of straight current-carrying wire segments.
    PathCurrent
    field_fn: path_current_B
    args: {
        current: T = T::zero(),
        vertices: @ref Vec<Vector3<T>> = Vec::new(),
    }
    arg_display: "current={}, vertices count={}";
    arg_fmt: [format_float, format_vertices_count]
    docs: {
        new: {
            /// Construct a [PathCurrent].
            ///
            /// # Examples
            ///
            /// ```
            /// # use magba::currents::PathCurrent;
            /// # use nalgebra::{UnitQuaternion, vector, Vector3};
            /// let vertices = vec![vector![-0.1, -0.1, -0.1], vector![0.1, -0.1, -0.1], vector![0.0, 0.1, -0.1], vector![0.0, 0.0, 0.1]];
            ///
            /// let path_current = PathCurrent::new(
            ///     [0.0, 0.0, 0.0],              // position (m)
            ///     UnitQuaternion::identity(),   // orientation as unit quaternion
            ///     100.0,                        // current (A)
            ///     vertices,
            /// );
            /// ```
        }
    }
}

#[cfg(all(test, feature = "std"))]
crate::testing_util::generate_tests! {
    PathCurrent
    filename: polyline
    params: {
        current: 100.0,
        vertices: vec![vector![-0.1, -0.1, -0.1], vector![0.1, -0.1, -0.1], vector![0.0, 0.1, -0.1], vector![0.0, 0.0, 0.1]],
    }
    rtols: {
        static: 1e-14,
        static_small: 1e-9,
        translate: 1e-9,
        rotate: 1e-14,
    }
    p95_rtols: {
        static: 1e-14,
        static_small: 1e-9,
        translate: 1e-9,
        rotate: 1e-14,
    }
    f32_rtols: {
        static: 5e-2,
        static_small: 0.2,
        translate: 5e-2,
        rotate: 1e-2,
    }
    f32_p95_rtols: {
        static: 1e-3,
        static_small: 1e-3,
        translate: 1e-3,
        rotate: 1e-3,
    }
}
