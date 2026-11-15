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
    relative_complexity: |source| source.vertices.len().saturating_sub(1);
    args: {
        current: @val T = T::zero(),
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
        current: 1.0,
        vertices: vec![vector![-0.1, -0.1, -0.1], vector![0.1, -0.1, -0.1], vector![0.0, 0.1, -0.1], vector![0.0, 0.0, 0.1]],
    }
    // The rationalized endpoint expression differs from the MagPyLib grid by
    // ~5e-14 at a nearly collinear observer; independent wire integrals cover it.
    rtols: {
        static: 1e-13,
        static_small: 1e-9,
        translate: 1e-9,
        rotate: 3e-14,
    }
    p95_rtols: {
        static: 1e-14,
        static_small: 1e-9,
        translate: 1e-9,
        rotate: 1e-14,
    }
    f32_rtols: {
        static: 1e-5,
        static_small: 1e-5,
        translate: 1e-5,
        rotate: 1e-5,
    }
    f32_p95_rtols: {
        static: 2e-6,
        static_small: 2e-6,
        translate: 2e-6,
        rotate: 2e-6,
    }
}

#[cfg(test)]
mod complexity_tests {
    use super::*;
    use crate::base::Source;
    use nalgebra::vector;

    #[test]
    fn complexity_tracks_segment_count() {
        let mut path = PathCurrent::<f64>::default();
        assert_eq!(path.relative_complexity(), 1);

        path.set_vertices(vec![
            vector![0.0, 0.0, 0.0],
            vector![1.0, 0.0, 0.0],
            vector![1.0, 1.0, 0.0],
            vector![1.0, 1.0, 1.0],
        ]);
        assert_eq!(path.relative_complexity(), 3);
    }
}
