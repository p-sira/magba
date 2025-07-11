//! Analytical B-field computation for cuboid (rectangular prism) magnets.
//!
//! Based on Yang (1990), Engel-Herbert (2005), Camacho (2013), Cichon (2019), and MagpyLib.

use nalgebra::{Matrix3, Point3, UnitQuaternion, Vector3};
use std::f64::consts::PI;

use crate::compute_in_local;
use crate::geometry::{global_vectors, local_points};
#[cfg(feature = "parallel")]
use rayon::prelude::*;

/// Compute B-field of a homogeneously magnetized cuboid at multiple points in the local frame.
///
/// # Arguments
/// - `points`: Observer positions (x, y, z) in local Cartesian coordinates (m)
/// - `dimensions`: Cuboid side lengths (a, b, c) (m)
/// - `polarizations`: Polarization vectors (T)
///
/// # Returns
/// - `Ok(Vec<Vector3<f64>>)` - B-field vectors at each observer point (T)
/// - `Err(&'static str)` - If computation fails
///
/// # References
/// - Yang: Superconductor Science and Technology 3(12):591 (1990)
/// - Engel-Herbert: J. Appl. Phys. 97(7):074504 (2005)
/// - Camacho: Rev. Mex. Fis. E 59 (2013) 8-17
/// - Cichon: IEEE Sensors Journal, vol. 19, no. 7, April 1, 2019, p.2509
#[allow(non_snake_case)]
#[inline]
pub fn local_cuboid_B(
    point: &Point3<f64>,
    dimensions: &Vector3<f64>,
    polarization: &Vector3<f64>,
) -> Vector3<f64> {
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

    Vector3::new(bx_tot, by_tot, bz_tot) / (4.0 * PI)
}

#[allow(non_snake_case)]
#[inline]
pub fn local_cuboid_B_vec(
    points: &[Point3<f64>],
    dimensions: &Vector3<f64>,
    polarization: &Vector3<f64>,
) -> Vec<Vector3<f64>> {
    #[cfg(feature = "parallel")]
    if points.len() > 60 {
        return points
            .par_iter()
            .map(|p| local_cuboid_B(p, dimensions, polarization))
            .collect();
    }

    // If small number of points or not using parallel feature
    points
        .iter()
        .map(|p| local_cuboid_B(p, dimensions, polarization))
        .collect()
}

/// Compute B-field at points in global frame for a single cuboid magnet.
///
/// # Arguments
/// - `points`: Points in global frame (m)
/// - `position`: Magnet position (m)
/// - `orientation`: Magnet orientation
/// - `dimensions`: Cuboid side lengths (a, b, c) (m)
/// - `polarization`: Polarization vector (T)
///
/// # Returns
/// - `Ok(Vec<Vector3<f64>>)` - B-field vectors (T)
/// - `Err(&'static str)` - If computation fails
#[allow(non_snake_case)]
pub fn cuboid_B(
    points: &[Point3<f64>],
    position: &Point3<f64>,
    orientation: &UnitQuaternion<f64>,
    dimensions: &Vector3<f64>,
    polarization: &Vector3<f64>,
) -> Vec<Vector3<f64>> {
    compute_in_local!(
        local_cuboid_B_vec,
        &points,
        (&dimensions, &polarization),
        &position,
        &orientation
    )
}
