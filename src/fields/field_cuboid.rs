//! Analytical B-field computation for cuboid (rectangular prism) magnets.
//!
//! Based on Yang (1990), Engel-Herbert (2005), Camacho (2013), Cichon (2019), and MagpyLib.

use std::iter::Sum;

use nalgebra::{Matrix3, Point3, RealField, UnitQuaternion, Vector3};
use numeric_literals::replace_float_literals;

use crate::compute_in_local;
#[cfg(feature = "parallel")]
use rayon::prelude::*;

/// Compute B-field of a homogeneous cuboid magnet at point (x, y, z) in the local frame.
///
/// # Arguments
/// - `point`: Observer position (m)
/// - `dimensions`: Cuboid side lengths (m)
/// - `polarization`: Polarization vector (T)
///
/// # Returns
/// - B-field vector (T) at point (x, y, z)
///
/// # References
/// - Ortner, Michael, and Lucas Gabriel Coliado Bandeira. “Magpylib: A Free Python Package for Magnetic Field Computation.” SoftwareX 11 (January 1, 2020): 100466. https://doi.org/10.1016/j.softx.2020.100466.
#[allow(non_snake_case)]
#[inline]
#[replace_float_literals(T::from_f64(literal).unwrap())]
pub fn local_cuboid_B<T: RealField + Copy>(
    point: &Point3<T>,
    dimensions: &Vector3<T>,
    polarization: &Vector3<T>,
) -> Vector3<T> {
    let (mut x, mut y, mut z) = (point.x, point.y, point.z);
    let abc = dimensions / 2.0;
    let (a, b, c) = (abc.x, abc.y, abc.z);

    // Handle special cases
    let RTOL_SURFACE = 1e-12;
    let dists = point.map(|v| v.abs()) - abc;
    let tols = abc * RTOL_SURFACE;

    let is_on_surf = [
        dists[0].abs() < tols[0],
        dists[1].abs() < tols[1],
        dists[2].abs() < tols[2],
    ];

    let is_inside = [dists[0] < tols[0], dists[1] < tols[1], dists[2] < tols[2]];
    // Return zero if the point is on the edge
    // A point is on the edge when it's on the orthogonal surfaces and inside the adjacent surface
    let is_on_edge_x = is_on_surf[1] && is_on_surf[2] && is_inside[0];
    let is_on_edge_y = is_on_surf[0] && is_on_surf[2] && is_inside[1];
    let is_on_edge_z = is_on_surf[0] && is_on_surf[1] && is_inside[2];

    if is_on_edge_x || is_on_edge_y || is_on_edge_z {
        return Vector3::zeros();
    }

    let mut qsign = Matrix3::from_element(1.0);
    // Avoid indeterminate form by mapping to the bottQ4 counterpart, according to MagpyLib
    // and apply sign flips
    if x < 0.0 {
        x = -x;
        qsign.component_mul_assign(&Matrix3::from([
            [1.0, -1.0, -1.0],
            [-1.0, 1.0, 1.0],
            [-1.0, 1.0, 1.0],
        ]));
    }
    if y > 0.0 {
        y = -y;
        qsign.component_mul_assign(&Matrix3::from([
            [1.0, -1.0, 1.0],
            [-1.0, 1.0, -1.0],
            [1.0, -1.0, 1.0],
        ]));
    }
    if z > 0.0 {
        z = -z;
        qsign.component_mul_assign(&Matrix3::from([
            [1.0, 1.0, -1.0],
            [1.0, 1.0, -1.0],
            [-1.0, -1.0, 1.0],
        ]));
    }

    let xma = x - a;
    let xpa = x + a;
    let ymb = y - b;
    let ypb = y + b;
    let zmc = z - c;
    let zpc = z + c;

    let xma2 = xma * xma;
    let xpa2 = xpa * xpa;
    let ymb2 = ymb * ymb;
    let ypb2 = ypb * ypb;
    let zmc2 = zmc * zmc;
    let zpc2 = zpc * zpc;

    let mmm = (xma2 + ymb2 + zmc2).sqrt();
    let pmp = (xpa2 + ymb2 + zpc2).sqrt();
    let pmm = (xpa2 + ymb2 + zmc2).sqrt();
    let mmp = (xma2 + ymb2 + zpc2).sqrt();
    let mpm = (xma2 + ypb2 + zmc2).sqrt();
    let ppp = (xpa2 + ypb2 + zpc2).sqrt();
    let ppm = (xpa2 + ypb2 + zmc2).sqrt();
    let mpp = (xma2 + ypb2 + zpc2).sqrt();

    let ff2x = ((xma + mmm) * (xpa + ppm) * (xpa + pmp) * (xma + mpp)).ln()
        - ((xpa + pmm) * (xma + mpm) * (xma + mmp) * (xpa + ppp)).ln();

    let ff2y = ((-ymb + mmm) * (-ypb + ppm) * (-ymb + pmp) * (-ypb + mpp)).ln()
        - ((-ymb + pmm) * (-ypb + mpm) * (ymb - mmp) * (ypb - ppp)).ln();

    let ff2z = ((-zmc + mmm) * (-zmc + ppm) * (-zpc + pmp) * (-zpc + mpp)).ln()
        - ((-zmc + pmm) * (zmc - mpm) * (-zpc + mmp) * (zpc - ppp)).ln();

    let ff1x =
        (ymb * zmc).atan2(xma * mmm) - (ymb * zmc).atan2(xpa * pmm) - (ypb * zmc).atan2(xma * mpm)
            + (ypb * zmc).atan2(xpa * ppm)
            - (ymb * zpc).atan2(xma * mmp)
            + (ymb * zpc).atan2(xpa * pmp)
            + (ypb * zpc).atan2(xma * mpp)
            - (ypb * zpc).atan2(xpa * ppp);
    let ff1y =
        (xma * zmc).atan2(ymb * mmm) - (xpa * zmc).atan2(ymb * pmm) - (xma * zmc).atan2(ypb * mpm)
            + (xpa * zmc).atan2(ypb * ppm)
            - (xma * zpc).atan2(ymb * mmp)
            + (xpa * zpc).atan2(ymb * pmp)
            + (xma * zpc).atan2(ypb * mpp)
            - (xpa * zpc).atan2(ypb * ppp);
    let ff1z =
        (xma * ymb).atan2(zmc * mmm) - (xpa * ymb).atan2(zmc * pmm) - (xma * ypb).atan2(zmc * mpm)
            + (xpa * ypb).atan2(zmc * ppm)
            - (xma * ymb).atan2(zpc * mmp)
            + (xpa * ymb).atan2(zpc * pmp)
            + (xma * ypb).atan2(zpc * mpp)
            - (xpa * ypb).atan2(zpc * ppp);

    let (pol_x, pol_y, pol_z) = (polarization.x, polarization.y, polarization.z);
    // Contributions from x-polarization
    let bx_pol_x = pol_x * ff1x * qsign[(0, 0)];
    let by_pol_x = pol_x * ff2z * qsign[(0, 1)];
    let bz_pol_x = pol_x * ff2y * qsign[(0, 2)];
    // Contributions from y-polarization
    let bx_pol_y = pol_y * ff2z * qsign[(1, 0)];
    let by_pol_y = pol_y * ff1y * qsign[(1, 1)];
    let bz_pol_y = -pol_y * ff2x * qsign[(1, 2)];
    // Contributions from z-polarization
    let bx_pol_z = pol_z * ff2y * qsign[(2, 0)];
    let by_pol_z = -pol_z * ff2x * qsign[(2, 1)];
    let bz_pol_z = pol_z * ff1z * qsign[(2, 2)];

    // Sum all contributions
    let bx_tot = bx_pol_x + bx_pol_y + bx_pol_z;
    let by_tot = by_pol_x + by_pol_y + by_pol_z;
    let bz_tot = bz_pol_x + bz_pol_y + bz_pol_z;

    Vector3::new(bx_tot, by_tot, bz_tot) / (4.0 * T::pi())
}

/// Compute B-field of a homogeneous cuboid magnet at point (x, y, z).
///
/// # Arguments
/// - `point`: Observer positions (m)
/// - `position`: Magnet position (m)
/// - `orientation`: Magnet orientation in unit quaternion
/// - `dimensions`: Cuboid side lengths (m)
/// - `polarization`: Polarization vector (T)
///
/// # Returns
/// - B-field vector (T) at point (x, y, z)
///
/// # References
/// - Ortner, Michael, and Lucas Gabriel Coliado Bandeira. “Magpylib: A Free Python Package for Magnetic Field Computation.” SoftwareX 11 (January 1, 2020): 100466. https://doi.org/10.1016/j.softx.2020.100466.
#[allow(non_snake_case)]
#[inline]
pub fn global_cuboid_B<T: RealField + Copy>(
    point: &Point3<T>,
    position: &Point3<T>,
    orientation: &UnitQuaternion<T>,
    dimensions: &Vector3<T>,
    polarization: &Vector3<T>,
) -> Vector3<T> {
    compute_in_local!(
        local_cuboid_B,
        point,
        position,
        orientation,
        (dimensions, polarization),
    )
}

/// Compute B-field at points in global frame for a single cuboid magnet.
///
/// # Arguments
/// - `points`: Observer positions (m)
/// - `position`: Magnet position (m)
/// - `orientation`: Magnet orientation in unit quaternion
/// - `dimensions`: Cuboid side lengths (m)
/// - `polarization`: Polarization vector (T)
///
/// # Returns
/// - B-field vectors at each observer (T)
#[allow(non_snake_case)]
pub fn cuboid_B<T: RealField + Copy>(
    points: &[Point3<T>],
    position: &Point3<T>,
    orientation: &UnitQuaternion<T>,
    dimensions: &Vector3<T>,
    polarization: &Vector3<T>,
) -> Vec<Vector3<T>> {
    #[cfg(feature = "parallel")]
    if points.len() > 60 {
        return points
            .par_iter()
            .map(|p| global_cuboid_B(p, position, orientation, dimensions, polarization))
            .collect();
    }

    // If small number of points or not using parallel feature
    points
        .iter()
        .map(|p| global_cuboid_B(p, position, orientation, dimensions, polarization))
        .collect()
}

/// Compute net B-field at each given point in global frame for multiple cuboid magnets.
///
/// # Arguments
/// - `points`: Observer positions in global frame (m)
/// - `positions`: Magnet positions (m)
/// - `orientations`: Magnet orientations as unit quaternions
/// - `dimensions`: Cuboid side lengths (m)
/// - `polarizations`: Polarization vectors (T)
///
/// # Returns
/// - Net B-field vectors at each observer (T)
#[allow(non_snake_case)]
pub fn sum_multiple_cuboid_B<T: RealField + Copy + Sum>(
    points: &[Point3<T>],
    positions: &[Point3<T>],
    orientations: &[UnitQuaternion<T>],
    dimensions: &[Vector3<T>],
    polarizations: &[Vector3<T>],
) -> Vec<Vector3<T>> {
    if positions.len() != orientations.len()
        || positions.len() != dimensions.len()
        || positions.len() != polarizations.len()
    {
        panic!("Lengths of input vectors must be equal.");
    }

    #[cfg(feature = "parallel")]
    {
        let vectors = positions
            .par_iter()
            .zip(orientations)
            .zip(dimensions)
            .zip(polarizations)
            .map(|(((position, orientation), dim), pol)| {
                cuboid_B(points, position, orientation, dim, pol)
            })
            .collect::<Vec<Vec<_>>>();

        let net_vectors: Vec<Vector3<_>> = (0..points.len())
            .map(|i| vectors.iter().map(|v| v[i]).sum())
            .collect();
        net_vectors
    }

    #[cfg(not(feature = "parallel"))]
    {
        let mut net_vectors = vec![Vector3::zeros(); points.len()];

        positions
            .iter()
            .zip(orientations)
            .zip(dimensions)
            .zip(polarizations)
            .map(|(((position, orientation), dim), pol)| {
                cuboid_B(points, position, orientation, dim, pol)
            })
            .for_each(|field_vectors| {
                net_vectors
                    .iter_mut()
                    .zip(field_vectors)
                    .for_each(|(net_vector, field_vector)| *net_vector += field_vector)
            });

        net_vectors
    }
}
