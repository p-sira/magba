/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Analytical B-field computation for cuboid magnets.

use nalgebra::{Point3, RealField, UnitQuaternion, Vector3, vector};
use numeric_literals::replace_float_literals;

use crate::{
    base::coordinate::compute_in_local,
    crate_utils::impl_parallel_sum,
};

/// Computes B-field of a homogeneous cuboid magnet at point (x, y, z) in the local frame.
///
/// # Arguments
///
/// - `point`: Observer position (m)
/// - `dimensions`: Cuboid side lengths (m)
/// - `polarization`: Polarization vector (T)
///
/// # Returns
///
/// - B-field vector (T) at point (x, y, z)
///
/// # References
///
/// - Ortner, Michael, and Lucas Gabriel Coliado Bandeira. “Magpylib: A Free Python Package for Magnetic Field Computation.” SoftwareX 11 (January 1, 2020): 100466. <https://doi.org/10.1016/j.softx.2020.100466>.
#[allow(non_snake_case)]
#[inline]
#[replace_float_literals(T::from_f64(literal).unwrap())]
pub fn local_cuboid_B<T: RealField + Copy>(
    point: Point3<T>,
    polarization: Vector3<T>,
    dimensions: Vector3<T>,
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

    let (pol_x, pol_y, pol_z) = (polarization.x, polarization.y, polarization.z);
    if pol_x == 0.0 && pol_y == 0.0 && pol_z == 0.0 {
        return Vector3::zeros();
    }

    let flip_x = x < 0.0;
    if flip_x {
        x = -x;
    }
    let flip_y = y > 0.0;
    if flip_y {
        y = -y;
    }
    let flip_z = z > 0.0;
    if flip_z {
        z = -z;
    }

    let q01 = if flip_x ^ flip_y { -1.0 } else { 1.0 };
    let q02 = if flip_x ^ flip_z { -1.0 } else { 1.0 };
    let q12 = if flip_y ^ flip_z { -1.0 } else { 1.0 };

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

    let ff2x = if pol_y != 0.0 || pol_z != 0.0 {
        let num = (xma + mmm) * (xpa + ppm) * (xpa + pmp) * (xma + mpp);
        let den = (xpa + pmm) * (xma + mpm) * (xma + mmp) * (xpa + ppp);
        (num / den).ln()
    } else {
        0.0
    };

    let ff2y = if pol_x != 0.0 || pol_z != 0.0 {
        let num = (-ymb + mmm) * (-ypb + ppm) * (-ymb + pmp) * (-ypb + mpp);
        let den = (-ymb + pmm) * (-ypb + mpm) * (ymb - mmp) * (ypb - ppp);
        (num / den).ln()
    } else {
        0.0
    };

    let ff2z = if pol_x != 0.0 || pol_y != 0.0 {
        let num = (-zmc + mmm) * (-zmc + ppm) * (-zpc + pmp) * (-zpc + mpp);
        let den = (-zmc + pmm) * (zmc - mpm) * (-zpc + mmp) * (zpc - ppp);
        (num / den).ln()
    } else {
        0.0
    };

    let (bx_pol_x, by_pol_x, bz_pol_x) = if pol_x != 0.0 {
        let ff1x = (ymb * zmc).atan2(xma * mmm) - (ymb * zmc).atan2(xpa * pmm)
            - (ypb * zmc).atan2(xma * mpm)
            + (ypb * zmc).atan2(xpa * ppm)
            - (ymb * zpc).atan2(xma * mmp)
            + (ymb * zpc).atan2(xpa * pmp)
            + (ypb * zpc).atan2(xma * mpp)
            - (ypb * zpc).atan2(xpa * ppp);
        (pol_x * ff1x, pol_x * ff2z * q01, pol_x * ff2y * q02)
    } else {
        (0.0, 0.0, 0.0)
    };

    let (bx_pol_y, by_pol_y, bz_pol_y) = if pol_y != 0.0 {
        let ff1y = (xma * zmc).atan2(ymb * mmm) - (xpa * zmc).atan2(ymb * pmm)
            - (xma * zmc).atan2(ypb * mpm)
            + (xpa * zmc).atan2(ypb * ppm)
            - (xma * zpc).atan2(ymb * mmp)
            + (xpa * zpc).atan2(ymb * pmp)
            + (xma * zpc).atan2(ypb * mpp)
            - (xpa * zpc).atan2(ypb * ppp);
        (pol_y * ff2z * q01, pol_y * ff1y, -pol_y * ff2x * q12)
    } else {
        (0.0, 0.0, 0.0)
    };

    let (bx_pol_z, by_pol_z, bz_pol_z) = if pol_z != 0.0 {
        let ff1z = (xma * ymb).atan2(zmc * mmm) - (xpa * ymb).atan2(zmc * pmm)
            - (xma * ypb).atan2(zmc * mpm)
            + (xpa * ypb).atan2(zmc * ppm)
            - (xma * ymb).atan2(zpc * mmp)
            + (xpa * ymb).atan2(zpc * pmp)
            + (xma * ypb).atan2(zpc * mpp)
            - (xpa * ypb).atan2(zpc * ppp);
        (pol_z * ff2y * q02, -pol_z * ff2x * q12, pol_z * ff1z)
    } else {
        (0.0, 0.0, 0.0)
    };

    // Sum all contributions
    let bx_tot = bx_pol_x + bx_pol_y + bx_pol_z;
    let by_tot = by_pol_x + by_pol_y + by_pol_z;
    let bz_tot = bz_pol_x + bz_pol_y + bz_pol_z;

    vector![bx_tot, by_tot, bz_tot] / (4.0 * T::pi())
}

/// Computes B-field of a homogeneous cuboid magnet at point (x, y, z).
///
/// # Arguments
///
/// - `point`: Observer positions (m)
/// - `position`: Magnet position (m)
/// - `orientation`: Magnet orientation in unit quaternion
/// - `polarization`: Polarization vector (T)
/// - `dimensions`: Cuboid side lengths (m)
///
/// # Examples
///
/// ```
/// # use approx::assert_relative_eq;
/// # use magba::fields::cuboid_B;
/// # use nalgebra::*;
/// let b_field = cuboid_B(
///     point![5.0, 6.0, 7.0],
///     point![1.0, 2.0, 3.0],
///     UnitQuaternion::from_scaled_axis(
///         [1.0471975511965976, 0.6283185307179586, 0.4487989505128276].into(),
///     ),
///     vector![0.45, 0.3, 0.15],
///     vector![1.0, 2.0, 3.0],
/// );
/// let expected = vector![0.0007246145093594572, 0.0008956704674508121, 0.0010056854402183814];
/// assert_relative_eq!(b_field, expected, epsilon = 5e-14);
/// ```
///
/// # References
///
/// - Ortner, Michael, and Lucas Gabriel Coliado Bandeira. “Magpylib: A Free Python Package for Magnetic Field Computation.” SoftwareX 11 (January 1, 2020): 100466. <https://doi.org/10.1016/j.softx.2020.100466>.
#[allow(non_snake_case)]
#[inline]
pub fn cuboid_B<T: RealField + Copy>(
    point: Point3<T>,
    position: Point3<T>,
    orientation: UnitQuaternion<T>,
    polarization: Vector3<T>,
    dimensions: Vector3<T>,
) -> Vector3<T> {
    compute_in_local!(
        local_cuboid_B,
        point,
        position,
        orientation,
        (polarization, dimensions),
    )
}

/// Computes B-field at points in global frame for a single cuboid magnet.
///
/// # Arguments
///
/// - `points`: Observer positions (m)
/// - `position`: Magnet position (m)
/// - `orientation`: Magnet orientation in unit quaternion
/// - `polarization`: Polarization vector (T)
/// - `dimensions`: Cuboid side lengths (m)
/// - `out`: Mutable slice to store the B-field vectors at each observer (T)
///
/// # Examples
///
/// ```
/// # use approx::assert_relative_eq;
/// # use magba::fields::cuboid_B_batch;
/// # use nalgebra::*;
/// let mut out = [Vector3::zeros(); 3];
/// cuboid_B_batch(
///     &[
///         point![5.0, 6.0, 7.0],
///         point![4.0, 3.0, 2.0],
///         point![0.5, 0.25, 0.125],
///     ],
///     point![1.0, 2.0, 3.0],
///     UnitQuaternion::from_scaled_axis(
///         [1.0471975511965976, 0.6283185307179586, 0.4487989505128276].into(),
///     ),
///     vector![0.45, 0.3, 0.15],
///     vector![1.0, 2.0, 3.0],
///     &mut out,
/// );
///
/// let expected_fields = [
///     vector![
///         0.0007246145093594572,
///         0.0008956704674508121,
///         0.0010056854402183814
///     ],
///     vector![
///         0.007318657264531047,
///         0.0013309418462993756,
///         -0.006614791491997044
///     ],
///     vector![
///         -0.002912635045339925,
///         0.003374408702355898,
///         0.009246801593396508
///     ],
/// ];
///
/// out.iter()
///     .zip(expected_fields.iter())
///     .for_each(|(actual, expected)| assert_relative_eq!(actual, expected, epsilon = 5e-14));
/// ```
///
/// # References
///
/// - Ortner, Michael, and Lucas Gabriel Coliado Bandeira. “Magpylib: A Free Python Package for Magnetic Field Computation.” SoftwareX 11 (January 1, 2020): 100466. <https://doi.org/10.1016/j.softx.2020.100466>.
#[allow(non_snake_case)]
pub fn cuboid_B_batch<T: RealField + Copy>(
    points: &[Point3<T>],
    position: Point3<T>,
    orientation: UnitQuaternion<T>,
    polarization: Vector3<T>,
    dimensions: Vector3<T>,
    out: &mut [Vector3<T>],
) {
    assert_eq!(
        out.len(),
        points.len(),
        "Output slice length must match input vectors length."
    );
    let inv_orientation = orientation.inverse();

    #[cfg(feature = "rayon")]
    {
        if points.len() > 50 {
            use rayon::prelude::*;
            out.par_iter_mut()
                .zip(points.par_iter())
                .for_each(|(o, p)| {
                    let local_point = inv_orientation * Point3::from(p.coords - position.coords);
                    let local_b = local_cuboid_B(local_point, polarization, dimensions);
                    *o = orientation * local_b;
                });
            return;
        }
    }

    out.iter_mut()
        .zip(points.iter())
        .for_each(|(o, p)| {
            let local_point = inv_orientation * Point3::from(p.coords - position.coords);
            let local_b = local_cuboid_B(local_point, polarization, dimensions);
            *o = orientation * local_b;
        });
}

/// Computes net B-field at each given point in global frame for multiple cuboid magnets.
///
/// # Arguments
///
/// - `points`: Observer positions in global frame (m)
/// - `positions`: Magnet positions (m)
/// - `orientations`: Magnet orientations as unit quaternions
/// - `polarizations`: Polarization vectors (T)
/// - `dimensions`: Cuboid side lengths (m)
/// - `out`: Mutable slice to store the net B-field vectors at each observer (T)
///
/// # References
///
/// - Ortner, Michael, and Lucas Gabriel Coliado Bandeira. “Magpylib: A Free Python Package for Magnetic Field Computation.” SoftwareX 11 (January 1, 2020): 100466. <https://doi.org/10.1016/j.softx.2020.100466>.
#[allow(non_snake_case)]
pub fn sum_multiple_cuboid_B<T: RealField + Copy>(
    points: &[Point3<T>],
    positions: &[Point3<T>],
    orientations: &[UnitQuaternion<T>],
    polarizations: &[Vector3<T>],
    dimensions: &[Vector3<T>],
    out: &mut [Vector3<T>],
) {
    impl_parallel_sum!(
        out,
        points,
        60,
        [positions, orientations, polarizations, dimensions],
        |pos, p, o, pol, dim| cuboid_B(*pos, *p, *o, *pol, *dim)
    )
}

#[cfg(test)]
mod tests {
    use nalgebra::{point, vector};

    use super::*;

    #[test]
    fn test_sum_multiple_cuboid_b() {
        use crate::testing_util::impl_test_sum_multiple;
        let points = &[
            point![5.0, 6.0, 7.0],
            point![4.0, 3.0, 2.0],
            point![0.5, 0.25, 0.125],
        ];
        let positions = &[point![1.0, 2.0, 3.0], point![0.0, 0.0, 0.0]];
        let orientations = &[
            UnitQuaternion::from_scaled_axis(vector![1.0, 0.6, 0.4]),
            UnitQuaternion::identity(),
        ];
        let polarizations = &[vector![0.45, 0.3, 0.15], vector![1.0, 2.0, 3.0]];
        let dimensions = &[vector![1.0, 2.0, 3.0], vector![2.0, 2.0, 2.0]];

        impl_test_sum_multiple!(
            sum_multiple_cuboid_B,
            5e-14,
            points,
            positions,
            orientations,
            (polarizations, dimensions),
            |p, pos, ori, pol, dim| cuboid_B(p, pos, ori, pol, dim)
        );
    }

    #[test]
    fn test_cuboid_edge_and_batch() {
        // Point on the cuboid edge: y = 1.0, z = 1.0, x = 0.5 (semi-dim = 1.0)
        let b_edge = local_cuboid_B(
            point![0.5, 1.0, 1.0],
            vector![0.0, 0.0, 1.0],
            vector![2.0, 2.0, 2.0],
        );
        assert_eq!(b_edge, Vector3::zeros());

        // Batch with > 50 points to trigger Rayon threshold
        let points = vec![point![0.0, 0.0, 5.0]; 60];
        let mut out = vec![Vector3::zeros(); 60];
        cuboid_B_batch(
            &points,
            Point3::origin(),
            UnitQuaternion::identity(),
            vector![0.0, 0.0, 1.0],
            vector![1.0, 1.0, 1.0],
            &mut out,
        );
        assert_eq!(out.len(), 60);
        assert!(out[0].z != 0.0);
    }
}
