/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use alloc::vec::Vec;
use nalgebra::Vector3;

use crate::crate_utils::define_source;

define_source! {
    /// Triangular mesh with homogeneous magnetic surface charge.
    MeshMagnet
    field_fn: mesh_B
    args: {
        polarization: Vector3<T> = Vector3::z(),
        vertices: @ref Vec<Vector3<T>> = Vec::new(),
        faces: @ref Vec<[usize; 3]> = Vec::new(),
    }
    arg_display: "pol={}, vertices={}, faces={}";
    arg_fmt: [format_vector3, format_vec_len, format_vec_len]
    docs: {
        new: {
            /// Construct a [MeshMagnet].
            ///
            /// # Examples
            ///
            /// ```
            /// # use magba::magnets::MeshMagnet;
            /// # use nalgebra::{UnitQuaternion, vector};
            /// let vertices = vec![vector![-0.1, -0.1, -0.1], vector![0.1, -0.1, -0.1], vector![0.0, 0.1, -0.1], vector![0.0, 0.0, 0.1]];
            /// let faces = vec![[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2]];
            ///
            /// let mesh = MeshMagnet::new(
            ///     [0.0, 0.0, 0.0],              // position (m)
            ///     UnitQuaternion::identity(),   // orientation as unit quaternion
            ///     [0.0, 0.0, 1.0],              // polarization (T)
            ///     vertices,
            ///     faces,
            /// );
            /// ```
        }
    }
}

#[cfg(all(test, feature = "std"))]
crate::testing_util::generate_tests! {
    MeshMagnet
    filename: triangularmesh
    params: {
        polarization: vector![1.0, 2.0, 3.0],
        vertices: vec![vector![-0.1, -0.1, -0.1], vector![0.1, -0.1, -0.1], vector![0.0, 0.1, -0.1], vector![0.0, 0.0, 0.1]],
        faces: vec![[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2]]
    }
    rtols: {
        static: 2e-10,
        static_small: 1e-9,
        translate: 1e-9,
        rotate: 1e-10,
    }
    f32_rtols: {
        static: 5e-2,
        static_small: 0.2,
        translate: 5e-2,
        rotate: 1e-2,
    }
}
