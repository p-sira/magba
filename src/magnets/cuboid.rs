/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use crate::magnets::define_magnet;
use nalgebra::Vector3;

define_magnet! {
    /// Uniformly magnetized cuboid magnet.
    ///
    /// # Fields
    /// 
    /// - `position`: Center of the cuboid (m)
    /// - `orientation`: Orientation as unit quaternion
    /// - `polarization`: Polarization vector (T)
    /// - `dimensions`: Cuboid side lengths (m)
    ///
    /// # Examples
    /// 
    /// ```
    /// # use magba::CuboidMagnet;
    /// # use nalgebra::UnitQuaternion;
    /// let magnet = CuboidMagnet::new(
    ///     [0.0, 0.0, 0.0],              // position (m)
    ///     UnitQuaternion::identity(),   // orientation
    ///     [0.0, 0.0, 1.0],              // polarization (T)
    ///     [0.01, 0.01, 0.02],           // dimensions (m)
    /// );
    /// ```
    /// 
    /// # References
    /// 
    /// - Ortner, Michael, and Lucas Gabriel Coliado Bandeira. “Magpylib: A Free Python Package for Magnetic Field Computation.” SoftwareX 11 (January 1, 2020): 100466. <https://doi.org/10.1016/j.softx.2020.100466>.
    CuboidMagnet
    field_fn: cuboid_B
    args: {
        polarization:Vector3<T> = Vector3::z(),
        dimensions:Vector3<T> = Vector3::from_element(T::one());
            where dimensions.iter().all(|&elem| elem >= T::zero()); else "Dimensions must be non-negative."
    }
    arg_display: "pol={}, dim={}";
    arg_fmt: [format_vector3, format_vector3]
}

#[cfg(test)]
crate::testing_util::generate_tests! {
    CuboidMagnet
    filename: cuboid
    params: { polarization: vector![1.0, 2.0, 3.0], dimensions: vector![0.1, 0.2, 0.3]}
    rtols: {
        static: 2e-10,
        static_small: 2e-10,
        translate: 2e-10,
        rotate: 2e-10,
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::UnitQuaternion;

    use crate::magnets::CuboidMagnet;

    #[test]
    #[should_panic]
    fn test_input_validation() {
        CuboidMagnet::new(
            [0.0; 3],
            UnitQuaternion::identity(),
            [0.0; 3],
            [0.0, -1.0, -2.0],
        );
    }
}
