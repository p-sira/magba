/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Analytical B-field computation for cuboid magnets.
//!
//! <div class="warning">⚠️ Unstable feature. May subject to changes.</div>

use nalgebra::{Matrix3, Point3, RealField, UnitQuaternion, Vector3, vector};
use numeric_literals::replace_float_literals;

use crate::{
    crate_util::{impl_parallel, impl_parallel_sum},
    geometry::compute_in_local,
};

/// Compute B-field of a homogeneous cuboid magnet at point (x, y, z) in the local frame.
///
/// <div class="warning">⚠️ Unstable feature. May subject to changes.</div>
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
    point: &Point3<T>,
    polarization: &Vector3<T>,
    dimensions: &Vector3<T>,
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

    vector![bx_tot, by_tot, bz_tot] / (4.0 * T::pi())
}

/// Compute B-field of a homogeneous cuboid magnet at point (x, y, z).
///
/// <div class="warning">⚠️ Unstable feature. May subject to changes.</div>
///
/// Wrapper of [local_cuboid_B].
#[allow(non_snake_case)]
#[inline]
pub fn global_cuboid_B<T: RealField + Copy>(
    point: &Point3<T>,
    position: &Point3<T>,
    orientation: &UnitQuaternion<T>,
    polarization: &Vector3<T>,
    dimensions: &Vector3<T>,
) -> Vector3<T> {
    compute_in_local!(
        local_cuboid_B,
        point,
        position,
        orientation,
        (polarization, dimensions),
    )
}

/// Compute B-field at points in global frame for a single cuboid magnet.
///
/// # Arguments
/// 
/// - `points`: Observer positions (m)
/// - `position`: Magnet position (m)
/// - `orientation`: Magnet orientation in unit quaternion
/// - `polarization`: Polarization vector (T)
/// - `dimensions`: Cuboid side lengths (m)
///
/// # Returns
/// 
/// - B-field vectors at each observer (T)
///
/// # References
/// 
/// - Ortner, Michael, and Lucas Gabriel Coliado Bandeira. “Magpylib: A Free Python Package for Magnetic Field Computation.” SoftwareX 11 (January 1, 2020): 100466. <https://doi.org/10.1016/j.softx.2020.100466>.
#[allow(non_snake_case)]
pub fn cuboid_B<T: RealField + Copy>(
    points: &[Point3<T>],
    position: &Point3<T>,
    orientation: &UnitQuaternion<T>,
    polarization: &Vector3<T>,
    dimensions: &Vector3<T>,
    out: &mut [Vector3<T>],
) {
    impl_parallel!(
        out,
        global_cuboid_B,
        60,
        points,
        position,
        orientation,
        polarization,
        dimensions,
    )
}

/// Compute net B-field at each given point in global frame for multiple cuboid magnets.
///
/// # Arguments
/// 
/// - `points`: Observer positions in global frame (m)
/// - `positions`: Magnet positions (m)
/// - `orientations`: Magnet orientations as unit quaternions
/// - `polarizations`: Polarization vectors (T)
/// - `dimensions`: Cuboid side lengths (m)
///
/// # Returns
/// 
/// - Net B-field vectors at each observer (T)
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
        |pos, p, o, pol, dim| global_cuboid_B(pos, p, o, pol, dim)
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{assert_close_vec, testing_util::quat_from_rotvec};
    use nalgebra::{point, vector};

    #[test]
    fn static_cases() {
        let mut out = [Vector3::zeros(); 1];
        cuboid_B(
            &[point![5.0, 6.0, 7.0]],
            &point![1.0, 2.0, 3.0],
            &quat_from_rotvec(1.0471975511965976, 0.6283185307179586, 0.4487989505128276),
            &vector![0.45, 0.3, 0.15],
            &vector![1.0, 2.0, 3.0],
            &mut out,
        );
        assert_close_vec!(
            out[0],
            vector![
                0.0007246145093594572,
                0.0008956704674508121,
                0.0010056854402183814
            ],
            5e-14
        );
    }
}
