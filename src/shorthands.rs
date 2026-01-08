/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use nalgebra::UnitQuaternion;

use crate::{CuboidMagnet, CylinderMagnet, Dipole, Float};

/// Construct a uniformly magnetized cylindrical magnet using a shorthand interface.
///
/// This function is a convenience wrapper around [`CylinderMagnet::new`] that
/// accepts plain arrays and a rotation vector for improved ergonomics.
///
/// # Parameters
/// - `position`: Center of the cylinder in Cartesian coordinates (m)
/// - `orientation`: Rotation vector representation of the orientation (rad)
/// - `polarization`: Uniform polarization vector (T)
/// - `diameter`: Cylinder diameter (m)
/// - `height`: Cylinder height (m)
///
/// # Examples
/// ```
/// use magba::shorthands::cylinder;
///
/// let magnet = cylinder(
///     [0.0, 0.0, 0.0], // position (m)
///     [0.0, 0.0, 0.0], // orientation (rad)
///     [0.0, 0.0, 1.0], // polarization (T)
///     0.01,            // diameter (m)
///     0.02,            // height (m)
/// );
/// ```
pub fn cylinder<T: Float>(
    position: [T; 3],
    orientation: [T; 3],
    polarization: [T; 3],
    diameter: T,
    height: T,
) -> CylinderMagnet<T> {
    CylinderMagnet::new(
        position.into(),
        UnitQuaternion::from_scaled_axis(orientation.into()),
        polarization.into(),
        diameter,
        height,
    )
}

/// Construct a uniformly magnetized cuboid magnet using a shorthand interface.
///
/// This function is a convenience wrapper around [`CuboidMagnet::new`] that
/// accepts plain arrays and a rotation vector for improved ergonomics.
///
/// # Parameters
/// - `position`: Center of the cuboid in Cartesian coordinates (m)
/// - `orientation`: Rotation vector representation of the orientation (rad)
/// - `polarization`: Uniform polarization vector (T)
/// - `dimensions`: Cuboid dimensions along the x, y, and z axes (m)
///
/// # Examples
/// ```
/// use magba::shorthands::cuboid;
///
/// let magnet = cuboid(
///     [0.0, 0.0, 0.0], // position (m)
///     [0.0, 0.0, 0.0], // orientation (identity rotation)
///     [1.0, 0.0, 0.0], // polarization (T)
///     [0.01, 0.02, 0.03], // dimensions (m)
/// );
/// ```
pub fn cuboid<T: Float>(
    position: [T; 3],
    orientation: [T; 3],
    polarization: [T; 3],
    dimensions: [T; 3],
) -> CuboidMagnet<T> {
    CuboidMagnet::new(
        position.into(),
        UnitQuaternion::from_scaled_axis(orientation.into()),
        polarization.into(),
        dimensions.into(),
    )
}

/// Construct a magnetic dipole using a shorthand interface.
///
/// This function is a convenience wrapper around [`Dipole::new`] that
/// accepts plain arrays and a rotation vector for improved ergonomics.
///
/// # Parameters
/// - `position`: Dipole position in Cartesian coordinates (m)
/// - `orientation`: Rotation vector representation of the orientation (rad)
/// - `moment`: Magnetic dipole moment vector (A·m²)
///
/// # Examples
/// ```
/// use magba::shorthands::dipole;
///
/// let dipole = dipole(
///     [0.0, 0.0, 0.0], // position (m)
///     [0.0, 0.0, 0.0], // orientation (identity rotation)
///     [0.0, 0.0, 1.0], // magnetic moment (A·m²)
/// );
/// ```
pub fn dipole<T: Float>(position: [T; 3], orientation: [T; 3], moment: [T; 3]) -> Dipole<T> {
    Dipole::new(
        position.into(),
        UnitQuaternion::from_scaled_axis(orientation.into()),
        moment.into(),
    )
}
