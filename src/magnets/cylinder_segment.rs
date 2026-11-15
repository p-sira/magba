/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use nalgebra::{Vector3, Vector5};

use crate::crate_utils::define_source;

define_source! {
    /// Uniformly magnetized cylindrical ring segment.
    ///
    /// The local z-axis is the cylinder axis. The azimuth angles are measured in degrees
    /// counterclockwise from the positive x-axis. The origin is the center of the full
    /// cylinder from which the segment is cut.
    ///
    /// The `dimensions` are `[inner_radius, outer_radius, height, start_angle, end_angle]`.
    /// Radii and height are in meters; angles are in degrees. An inner radius of zero
    /// produces a solid sector, and a 360° opening produces a full cylinder or tube.
    ///
    /// # References
    ///
    /// - Slanovc, F., *Journal of Magnetism and Magnetic Materials* 559 (2022), 169482.
    /// - Ortner, Michael, and Lucas Gabriel Coliado Bandeira. “Magpylib: A Free Python Package for Magnetic Field Computation.” SoftwareX 11 (2020): 100466. <https://doi.org/10.1016/j.softx.2020.100466>.
    #[cfg_attr(feature = "graphics-plotly", doc = include_str!("graphics/cylinder_segment.md"))]
    CylinderSegmentMagnet
    field_fn: cylinder_segment_B
    relative_complexity: |_source| 20;
    args: {
        polarization: Vector3<T> = Vector3::z(),
        dimensions: Vector5<T> = Vector5::new(T::zero(), T::one(), T::one(), T::zero(), T::from(90.0).unwrap());
            validate dimensions.iter().all(|&v| num_traits::Float::is_finite(v))
                && dimensions[0] >= T::zero()
                && dimensions[1] > dimensions[0]
                && dimensions[2] > T::zero()
                && dimensions[4] > dimensions[3]
                && dimensions[4] - dimensions[3] <= T::from(360.0).unwrap();
            error "Cylinder segment dimensions require 0 <= r1 < r2, h > 0, and 0 < phi2 - phi1 <= 360 degrees."
    }
    arg_display: "pol={}, dim={}";
    arg_fmt: [format_vector3, format_vector5]
    docs: {
        new: {
            /// Construct a [CylinderSegmentMagnet].
            ///
            /// # Examples
            ///
            /// ```
            /// # use magba::magnets::CylinderSegmentMagnet;
            /// # use nalgebra::UnitQuaternion;
            /// let magnet = CylinderSegmentMagnet::new(
            ///     [0.0, 0.0, 0.0],              // position (m)
            ///     UnitQuaternion::identity(),   // orientation as unit quaternion
            ///     [0.0, 0.0, 1.0],              // polarization (T)
            ///     [0.01, 0.02, 0.01, 0.0, 90.0], // radii (m), height (m), angles (degrees)
            /// );
            /// ```
        }
    }
}

#[cfg(all(test, feature = "std", not(feature = "test_force_fail")))]
crate::testing_util::generate_tests! {
    CylinderSegmentMagnet
    filename: cylindersegment
    params: {
        polarization: vector![1.0, 2.0, 3.0],
        dimensions: nalgebra::vector![0.01, 0.03, 0.02, -30.0, 120.0]
    }
    rtols: {
        static: 3e-5,
        static_small: 3e-5,
        translate: 3e-5,
        rotate: 3e-5,
    }
    p95_rtols: {
        static: 3e-6,
        static_small: 3e-6,
        translate: 3e-6,
        rotate: 3e-6,
    }
    f32_rtols: {
        static: 5e-2,
        static_small: 5e-2,
        translate: 5e-2,
        rotate: 5e-2,
    }
    f32_p95_rtols: {
        static: 5e-3,
        static_small: 5e-3,
        translate: 5e-3,
        rotate: 5e-3,
    }
}

#[cfg(all(test, feature = "std"))]
mod tests {
    use super::*;

    #[test]
    fn display_formats_all_dimensions_with_requested_precision() {
        let magnet = CylinderSegmentMagnet::<f64>::default()
            .with_dimensions([0.01, 0.03, 0.02, -30.0, 120.0]);
        assert_eq!(
            format!("{magnet}"),
            "CylinderSegmentMagnet (pol=[0.0, 0.0, 1.0], dim=[0.01, 0.03, 0.02, -30.0, 120.0]) at pos=[0.0, 0.0, 0.0], rot=[0.0, 0.0, 0.0]"
        );
        assert_eq!(
            format!("{magnet:.2}"),
            "CylinderSegmentMagnet (pol=[0.00, 0.00, 1.00], dim=[0.01, 0.03, 0.02, -30.00, 120.00]) at pos=[0.0, 0.0, 0.0], rot=[0.0, 0.0, 0.0]"
        );
    }

    #[test]
    fn rejects_invalid_dimensions() {
        let mut magnet = CylinderSegmentMagnet::<f64>::default();
        for dims in [
            [-1.0, 2.0, 1.0, 0.0, 90.0],
            [2.0, 2.0, 1.0, 0.0, 90.0],
            [0.0, 2.0, 0.0, 0.0, 90.0],
            [0.0, 2.0, 1.0, 90.0, 0.0],
            [0.0, 2.0, 1.0, 0.0, 361.0],
            [0.0, f64::INFINITY, 1.0, 0.0, 90.0],
        ] {
            let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                magnet.set_dimensions(dims);
            }));
            assert!(result.is_err(), "accepted invalid dimensions {dims:?}");
        }
        assert_eq!(
            magnet.dimensions(),
            CylinderSegmentMagnet::<f64>::default().dimensions()
        );
    }
}
