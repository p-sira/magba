/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Analytical B-field computation for a current path (a sequence of straight wire segments).

use nalgebra::{Point3, UnitQuaternion, Vector3};
use num_traits::Float as NumFloat;
use numeric_literals::replace_float_literals;

use crate::{
    base::{Float, coordinate::compute_in_local},
    crate_utils::{impl_parallel, impl_parallel_sum},
};

/// Computes B-field of a current path (sequence of straight wire segments) at point in local frame.
///
/// # Arguments
///
/// - `point`: Observer position (m)
/// - `current`: Current (A)
/// - `vertices`: Vertices defining the current path `[P1, P2, ...]` in local coords (m)
///
/// # Returns
///
/// - B-field vector (T) at point (x, y, z)
///
/// # References
///
/// - Ortner, Michael, and Lucas Gabriel Coliado Bandeira. “Magpylib: A Free Python Package for Magnetic Field Computation.” SoftwareX 11 (January 1, 2020): 100466. <https://doi.org/10.1016/j.softx.2020.100466>.
#[inline]
#[allow(non_snake_case)]
#[replace_float_literals(T::from_f64(literal).unwrap())]
pub fn local_path_current_B<T: Float>(
    point: Point3<T>,
    current: T,
    vertices: &[Vector3<T>],
) -> Vector3<T> {
    let mut b_total = Vector3::zeros();
    let n = vertices.len();
    if n < 2 {
        return b_total;
    }

    let po = Vector3::from(point.coords);

    for i in 0..n - 1 {
        let p1 = vertices[i];
        let p2 = vertices[i + 1];

        let p12 = p1 - p2;
        let norm_12 = p12.norm();
        if norm_12 == 0.0 {
            continue;
        }

        // Normalize points to make dimensionless as in magpylib (avoid precision issues)
        let p1_n = p1 / norm_12;
        let p2_n = p2 / norm_12;
        let po_n = po / norm_12;

        let p12_n = p1_n - p2_n;
        let t = (po_n - p1_n).dot(&p12_n);
        let p4_n = p1_n + p12_n * t;

        let po_p4 = po_n - p4_n;
        let norm_o4 = po_p4.norm();

        if norm_o4 < 1e-15 {
            continue;
        }

        let cros = (p2_n - p1_n).cross(&po_p4);
        let norm_cros = cros.norm();
        if norm_cros == 0.0 {
            continue;
        }
        let e_b = cros / norm_cros;

        let norm_o1 = (po_n - p1_n).norm();
        let norm_o2 = (po_n - p2_n).norm();
        let norm_41 = (p4_n - p1_n).norm();
        let norm_42 = (p4_n - p2_n).norm();

        let sin_th1 = norm_41 / norm_o1;
        let sin_th2 = norm_42 / norm_o2;

        let delta_sin = if norm_41 > 1.0 && norm_41 > norm_42 {
            NumFloat::abs(sin_th1 - sin_th2)
        } else if norm_42 > 1.0 && norm_42 > norm_41 {
            NumFloat::abs(sin_th2 - sin_th1)
        } else {
            NumFloat::abs(sin_th1 + sin_th2)
        };

        let b_mag = (delta_sin / norm_o4) * (1.0 / norm_12) * current * T::mu0_4pi();
        b_total += e_b * b_mag;
    }

    if b_total.x.is_nan() || b_total.y.is_nan() || b_total.z.is_nan() {
        Vector3::zeros()
    } else {
        b_total
    }
}

/// Computes B-field of a current path at point (x, y, z).
///
/// # Arguments
///
/// - `point`: Observer position (m)
/// - `position`: Element position (m)
/// - `orientation`: Element orientation in unit quaternion
/// - `current`: Current (A)
/// - `vertices`: Vertices defining the current path in local coords (m)
///
/// # Returns
///
/// - B-field vector (T) at point (x, y, z)
#[inline]
#[allow(non_snake_case)]
pub fn path_current_B<T: Float>(
    point: Point3<T>,
    position: Point3<T>,
    orientation: UnitQuaternion<T>,
    current: T,
    vertices: &[Vector3<T>],
) -> Vector3<T> {
    compute_in_local!(
        local_path_current_B,
        point,
        position,
        orientation,
        (current, vertices),
    )
}

/// Computes B-field at points in global frame for a current path.
///
/// # Arguments
///
/// - `points`: Observer positions (m)
/// - `position`: Element position (m)
/// - `orientation`: Element orientation in unit quaternion
/// - `current`: Current (A)
/// - `vertices`: Vertices defining the current path in local coords (m)
/// - `out`: Mutable slice to store the B-field vectors at each observer (T)
#[allow(non_snake_case)]
pub fn path_current_B_batch<T: Float>(
    points: &[Point3<T>],
    position: Point3<T>,
    orientation: UnitQuaternion<T>,
    current: T,
    vertices: &[Vector3<T>],
    out: &mut [Vector3<T>],
) {
    impl_parallel!(
        path_current_B,
        rayon_threshold: 200,
        input: points,
        output: out,
        args: [position, orientation, current, vertices]
    )
}

/// Computes B-field at each given points in global frame for multiple current paths.
///
/// # Arguments
///
/// - `points`: Observer positions (m)
/// - `positions`: Element positions (m)
/// - `orientations`: Element orientations in unit quaternion
/// - `currents`: Currents (A)
/// - `vertices_list`: List of vertices defining the current path in local coords (m)
/// - `out`: Mutable slice to store the net B-field vectors at each observer (T)
#[allow(non_snake_case)]
pub fn sum_multiple_path_current_B<T: Float>(
    points: &[Point3<T>],
    positions: &[Point3<T>],
    orientations: &[UnitQuaternion<T>],
    currents: &[T],
    vertices_list: &[alloc::vec::Vec<Vector3<T>>],
    out: &mut [Vector3<T>],
) {
    impl_parallel_sum!(
        out,
        points,
        60,
        [positions, orientations, currents, vertices_list],
        |pos, p, o, curr, vert| path_current_B(*pos, *p, *o, *curr, vert)
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{point, vector};

    #[test]
    #[allow(non_snake_case)]
    fn test_sum_multiple_path_current_B() {
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
        let currents = &[100.0, 200.0];
        let vertices_list = &[
            vec![
                vector![0.0, 0.0, 0.0],
                vector![0.0, 0.0, 1.0],
                vector![1.0, 0.0, 0.0],
            ],
            vec![
                vector![0.0, 0.0, 0.0],
                vector![0.0, 1.0, 0.0],
                vector![0.0, 0.0, 1.0],
            ],
        ];

        impl_test_sum_multiple!(
            sum_multiple_path_current_B,
            1e-15,
            points,
            positions,
            orientations,
            (currents, vertices_list),
            |p, pos, ori, curr, vert| path_current_B(p, pos, ori, curr, &vert)
        );
    }
}
