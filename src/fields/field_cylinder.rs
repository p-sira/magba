/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Analytical B-field computation for cylindrical magnets.

use ellip::{ellipe, ellipk};
use nalgebra::{Point3, UnitQuaternion, Vector3};
use std::f64::consts::PI;

use crate::geometry::{cart2cyl, global_vectors, local_points, vec_cyl2cart};
use crate::special::cel;
use crate::{compute_in_local, util};
#[cfg(feature = "parallel")]
use rayon::prelude::*;

/// Compute B-field of a cylindrical magnet with unit axial (z-axis) polarization
/// at point *(r, z)* in cylindrical CS.
///
/// *z0*: Dimensionless quantity of half the height over radius
///
/// Translated from MagpyLib, which is based on Derby & Olbert, 2010.
#[allow(non_snake_case)]
#[inline]
pub fn unit_axial_cyl_B_cyl(r: f64, z: f64, z0: f64) -> Result<Vector3<f64>, &'static str> {
    let (zp, zm) = (z + z0, z - z0);
    let (rp, rm) = (1.0 + r, 1.0 - r);

    let (zp2, zm2) = (zp * zp, zm * zm);
    let (rp2, rm2) = (rp * rp, rm * rm);

    let sq0 = (zm2 + rp2).sqrt();
    let sq1 = (zp2 + rp2).sqrt();

    let kp = ((zp2 + rm2) / (zp2 + rp2)).sqrt();
    let km = ((zm2 + rm2) / (zm2 + rp2)).sqrt();

    let gamma = rm / rp;
    let gamma2 = gamma * gamma;

    let br = (cel(kp, 1.0, 1.0, -1.0)? / sq1 - cel(km, 1.0, 1.0, -1.0)? / sq0) / PI;
    let bz = (zp * cel(kp, gamma2, 1.0, gamma)? / sq1 - zm * cel(km, gamma2, 1.0, gamma)? / sq0)
        / (rp * PI);
    // bphi = 0
    Ok(Vector3::new(br, 0.0, bz))
}

/// Compute B-field of a cylindrical magnet with unit diametrial (r-axis) polarization
/// at point *(r, phi, z)* in cylindrical CS.
///
/// *z0*: Dimensionless quantity of half the height over radius
///
/// Translated from MagpyLib, which is based on Caciagli et al., 2018.
#[allow(non_snake_case)]
#[inline]
pub fn unit_diametric_cyl_B_cyl(
    r: f64,
    phi: f64,
    z: f64,
    z0: f64,
) -> Result<Vector3<f64>, &'static str> {
    let (zp, zm) = (z + z0, z - z0);
    let (zp2, zm2) = (zp * zp, zm * zm);
    let r2 = r * r;

    if r < 5e-2 {
        // Taylor series approximation for small r
        let (zp4, zm4) = (zp2 * zp2, zm2 * zm2);
        let (zpp, zmm) = (zp2 + 1.0, zm2 + 1.0);
        let (zpp2, zmm2) = (zpp * zpp, zmm * zmm);
        let (zpp3, zmm3) = (zpp2 * zpp, zmm2 * zmm);
        let (zpp4, zmm4) = (zpp3 * zpp, zmm3 * zmm);
        let (zpp5, zmm5) = (zpp4 * zpp, zmm4 * zmm);
        let (sqrt_p, sqrt_m) = (zpp.sqrt(), zmm.sqrt());
        let (frac1, frac2) = (zp / sqrt_p, zm / sqrt_m);

        let r3 = r2 * r;
        let r4 = r3 * r;
        let r5 = r4 * r;

        let term1 = frac1 - frac2;
        let term2 = (frac1 / zpp2 - frac2 / zmm2) * r2 / 8.0;
        let term3 =
            ((3.0 - 4.0 * zp2) * frac1 / zpp4 - (3.0 - 4.0 * zm2) * frac2 / zmm4) / 64.0 * r4;

        let br = -phi.cos() / 4.0 * (term1 + 9.0 * term2 + 25.0 * term3);
        let bphi = phi.sin() / 4.0 * (term1 + 3.0 * term2 + 5.0 * term3);
        let bz = -phi.cos() / 4.0
            * (r * (1.0 / zpp / sqrt_p - 1.0 / zmm / sqrt_m)
                + 3.0 / 8.0
                    * r3
                    * ((1.0 - 4.0 * zp2) / zpp3 / sqrt_p - (1.0 - 4.0 * zm2) / zmm3 / sqrt_m)
                + 15.0 / 64.0
                    * r5
                    * ((1.0 - 12.0 * zp2 + 8.0 * zp4) / zpp5 / sqrt_p
                        - (1.0 - 12.0 * zm2 + 8.0 * zm4) / zmm5 / sqrt_m));
        return Ok(Vector3::new(br, bphi, bz));
    }

    // General case
    let (rp, rm) = (r + 1.0, r - 1.0);
    let (rp2, rm2) = (rp * rp, rm * rm);

    let (ap2, am2) = (zp2 + rm2, zm2 + rm2);
    let (ap, am) = (ap2.sqrt(), am2.sqrt());

    let (argp, argm) = (-4.0 * r / ap2, -4.0 * r / am2);

    // Branch special case r=r0
    let (argc, one_over_rm) = if rm == 0.0 {
        (1e16, 0.0)
    } else {
        (-4.0 * r / rm2, 1.0 / rm)
    };

    // Compute elliptics
    let (ellk_p, ellk_m) = (ellipk(argp)?, ellipk(argm)?);
    let (elle_p, elle_m) = (ellipe(argp)?, ellipe(argm)?);
    let (ellpi_p, ellpi_m) = (
        cel((1.0 - argp).sqrt(), 1.0 - argc, 1.0, 1.0)?,
        cel((1.0 - argm).sqrt(), 1.0 - argc, 1.0, 1.0)?,
    );

    // Compute the fields
    let br = -phi.cos() / (4.0 * PI * r2)
        * (-zm * am * elle_m + zp * ap * elle_p + zm / am * (2.0 + zm2) * ellk_m
            - zp / ap * (2.0 + zp2) * ellk_p
            + (zm / am * ellpi_m - zp / ap * ellpi_p) * rp * (r2 + 1.0) * one_over_rm);

    let bphi = phi.sin() / (4.0 * PI * r2)
        * (zm * am * elle_m - zp * ap * elle_p - zm / am * (2.0 + zm2 + 2.0 * r2) * ellk_m
            + zp / ap * (2.0 + zp2 + 2.0 * r2) * ellk_p
            + zm / am * rp2 * ellpi_m
            - zp / ap * rp2 * ellpi_p);

    let bz = -phi.cos() / (2.0 * PI * r)
        * (am * elle_m - ap * elle_p - (1.0 + zm2 + r2) / am * ellk_m
            + (1.0 + zp2 + r2) / ap * ellk_p);

    Ok(Vector3::new(br, bphi, bz))
}

/// Compute B-field of a cylindrical magnet
/// at point *(r, phi, z)* in cylindrical CS.
///
/// Zero vector is returned if the point is close to the cylindrical magnet's edge (rim).
///
/// Translated from MagpyLib.
#[allow(non_snake_case)]
#[inline]
pub fn cyl_B_cyl(
    r: f64,
    phi: f64,
    z: f64,
    radius: f64,
    height: f64,
    pol_r: f64,
    pol_z: f64,
) -> Result<Vector3<f64>, &'static str> {
    // Scale invariance
    let r = r / radius;
    let z = z / radius;
    let z0 = (height / 2.0) / radius;

    // Check if point is on Cylinder edge
    if util::is_close(r, 1.0, 1e-15) && util::is_close(z.abs(), z0, 1e-15) {
        return Ok(Vector3::zeros());
    }

    // M = Mz + Mr (Caciagli et al., 2018)
    let mut b = Vector3::zeros();
    if pol_z != 0.0 {
        let b_axial_cyl =
            pol_z * unit_axial_cyl_B_cyl(r, z, z0).expect("fn cyl_b_cyl: cannot compute axial B.");
        b += b_axial_cyl;
    }

    if pol_r != 0.0 {
        let b_diametric_cyl = pol_r
            * unit_diametric_cyl_B_cyl(r, phi, z, z0)
                .expect("fn cyl_b_cyl: cannot compute diametric B.");
        b += b_diametric_cyl;
    }

    Ok(b)
}

/// Compute B-field at point *(x, y, z)* of a cylindrical magnet in local frame.
///
/// Translated from MagpyLib.
///
/// ```
/// use magba::fields::field_cylinder;
/// use nalgebra::{Point3, Vector3};
///
/// let b = field_cylinder::local_cyl_B(&Point3::new(1.0, -1.0, 0.0), 1.0, 2.0, &Vector3::new(1.0, 2.0, 3.0)).expect("Invalid b calculation");
/// assert_eq! (b, Vector3::new(-0.36846056628423773, -0.10171405289381394, -0.3300649209932216));
///
/// let b = field_cylinder::local_cyl_B(&Point3::new(1.0, 1.0, 1.0), 0.5, 2.0, &Vector3::new(3.0, 2.0, -1.0)).expect("Invalid b calculation");
/// assert_eq! (b, Vector3::new(0.05331225054004448, 0.07895873346514143, 0.10406997810600024));
///
/// let b = field_cylinder::local_cyl_B(&Point3::new(0.0, 0.0, 0.0), 1.5, 3.0, &Vector3::new(1.0, 1.0, 1.0)).expect("Invalid b calculation");
/// assert_eq! (b, Vector3::new(0.6464466094067263, 0.6464466094067263, 0.7071067811865476));
/// ```
#[allow(non_snake_case)]
#[inline]
pub fn local_cyl_B(
    point: &Point3<f64>,
    radius: f64,
    height: f64,
    pol: &Vector3<f64>,
) -> Result<Vector3<f64>, &'static str> {
    let (r, phi) = cart2cyl(point.x, point.y);
    let (pol_r, theta) = cart2cyl(pol.x, pol.y);

    let b_cyl = cyl_B_cyl(r, phi - theta, point.z, radius, height, pol_r, pol.z)?;

    let (bx, by) = vec_cyl2cart(b_cyl.x, b_cyl.y, phi);
    // Check if point is in the magnet
    if r <= radius && point.z.abs() <= height / 2.0 {
        return Ok(Vector3::new(bx + pol.x, by + pol.y, b_cyl.z));
    }

    Ok(Vector3::new(bx, by, b_cyl.z))
}

/// Compute B-field of a cylindrical magnet at multiple points in local frame.
///
/// Serialized approach is used if the number of points is small.
/// Otherwise, the calculation is parallel (need `parallel` feature).
#[allow(non_snake_case)]
#[inline]
pub fn local_cyl_B_vec(
    points: &[Point3<f64>],
    radius: f64,
    height: f64,
    pol: &Vector3<f64>,
) -> Result<Vec<Vector3<f64>>, &'static str> {
    #[cfg(feature = "parallel")]
    if points.len() > 20 {
        return Ok(points
            .par_iter()
            .map(|p| local_cyl_B(p, radius, height, pol).unwrap())
            .collect());
    }

    // If small number of points or not using parallel feature
    Ok(points
        .iter()
        .map(|p| local_cyl_B(p, radius, height, pol).unwrap())
        .collect())
}

/// Compute B-field of a cylindrical magnet at multiple points.
#[allow(non_snake_case)]
pub fn cyl_B(
    points: &[Point3<f64>],
    position: &Point3<f64>,
    orientation: &UnitQuaternion<f64>,
    radius: f64,
    height: f64,
    pol: &Vector3<f64>,
) -> Result<Vec<Vector3<f64>>, &'static str> {
    Ok(compute_in_local!(
        local_cyl_B_vec,
        &points,
        (radius, height, &pol),
        &position,
        &orientation
    ))
}

/// Compute net B-field of multiple cylindrical magnets at multiple points.
///
/// The B-field contributions of each cylindrical magnet at every points are summed.
#[allow(non_snake_case)]
pub fn sum_multiple_cyl_B(
    points: &[Point3<f64>],
    positions: &[Point3<f64>],
    orientations: &[UnitQuaternion<f64>],
    radii: &[f64],
    heights: &[f64],
    pols: &[Vector3<f64>],
) -> Result<Vec<Vector3<f64>>, &'static str> {
    if positions.len() != orientations.len()
        || positions.len() != radii.len()
        || positions.len() != heights.len()
        || positions.len() != pols.len()
    {
        return Err("fn sum_multiple_cyl_b: Length of input vectors must be equal.");
    }

    #[cfg(feature = "parallel")]
    let vectors = positions
        .par_iter()
        .zip(orientations)
        .zip(radii)
        .zip(heights)
        .zip(pols)
        .map(|((((position, orientation), radius), height), pol)| {
            cyl_B(points, position, orientation, *radius, *height, pol)
        })
        .collect::<Result<Vec<Vec<_>>, _>>()?;

    #[cfg(not(feature = "parallel"))]
    let vectors = positions
        .iter()
        .zip(orientations)
        .zip(radii)
        .zip(heights)
        .zip(pols)
        .map(|((((position, orientation), radius), height), pol)| {
            cyl_B(points, position, orientation, *radius, *height, pol)
        })
        .collect::<Result<Vec<Vec<_>>, _>>()?;

    let net_vector: Vec<Vector3<f64>> = (0..points.len())
        .map(|i| vectors.iter().map(|v| v[i]).sum())
        .collect();
    Ok(net_vector)
}
