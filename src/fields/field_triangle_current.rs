/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Analytical B-field computation for a homogeneously charged triangular current sheet.

use nalgebra::{Point3, UnitQuaternion, Vector3};
use num_traits::Float as NumFloat;
use numeric_literals::replace_float_literals;

use crate::{
    base::{Float, coordinate::compute_in_local},
    crate_utils::{impl_parallel, impl_parallel_sum},
};

/// Computes B-field of an elementar current sheet in the local frame.
#[inline]
#[allow(non_snake_case)]
#[replace_float_literals(T::from_f64(literal).unwrap())]
fn elementar_current_sheet_Hfield<T: Float>(
    x: T,
    y: T,
    mut z: T,
    u1: T,
    u2: T,
    v2: T,
    ju: T,
    jv: T,
) -> Vector3<T> {
    if NumFloat::abs(z) < 1e-15 {
        z = if z < 0.0 { -1e-15 } else { 1e-15 };
    }

    let sqrt1 = NumFloat::sqrt((x * x) + (y * y) + (z * z));
    let sqrt2 = NumFloat::sqrt((u1 * u1) - 2.0 * u1 * x + (x * x) + (y * y) + (z * z));
    let sqrt3 = NumFloat::sqrt(
        (u2 * u2) - 2.0 * u2 * x + (v2 * v2) - 2.0 * v2 * y + (x * x) + (y * y) + (z * z),
    );
    let sqrt4 = NumFloat::sqrt((u1 * u1) - 2.0 * u1 * u2 + (u2 * u2) + (v2 * v2));
    let sqrt5 = NumFloat::sqrt((u2 * u2) + (v2 * v2));

    let mut H = Vector3::zeros();

    H.x = (NumFloat::atan((-u2 * ((y * y) + (z * z)) + v2 * x * y) / (v2 * z * sqrt1))
        + NumFloat::atan((v2 * y * (u1 - x) - (u1 - u2) * ((y * y) + (z * z))) / (v2 * z * sqrt2))
        - NumFloat::atan(
            (-u2 * ((y * y) + (z * z)) - (v2 * v2) * x + v2 * y * (u2 + x)) / (v2 * z * sqrt3),
        )
        - NumFloat::atan(
            (-u1 * ((v2 * v2) - 2.0 * v2 * y + (y * y) + (z * z))
                + u2 * ((y * y) + (z * z))
                + (v2 * v2) * x
                - v2 * y * (u2 + x))
                / (v2 * z * sqrt3),
        ))
        / (u1 * v2 * z);

    H.y = H.x; // Set to H.x temporarily

    H.z = -(ju * NumFloat::atanh(x / sqrt1) + ju * NumFloat::atanh((u1 - x) / sqrt2)
        - (ju * (u1 - u2) - jv * v2)
            * NumFloat::atanh(((u1 * u1) - u1 * (u2 + x) + u2 * x + v2 * y) / (sqrt4 * sqrt2))
            / sqrt4
        + (ju * (u1 - u2) - jv * v2)
            * NumFloat::atanh(
                (u1 * (u2 - x) - (u2 * u2) + u2 * x + v2 * (-v2 + y)) / (sqrt4 * sqrt3),
            )
            / sqrt4
        + (ju * u2 + jv * v2) * NumFloat::atanh((-u2 * x - v2 * y) / (sqrt5 * sqrt1)) / sqrt5
        - (ju * u2 + jv * v2)
            * NumFloat::atanh(((u2 * u2) - u2 * x + v2 * (v2 - y)) / (sqrt5 * sqrt3))
            / sqrt5)
        / (u1 * v2);

    let H0 = H.x * jv * z * u1 * v2 / (4.0 * T::pi());
    let H1 = -H.y * ju * z * u1 * v2 / (4.0 * T::pi());
    let H2 = H.z * u1 * v2 / (4.0 * T::pi());

    Vector3::new(H0, H1, H2)
}

/// Computes B-field of a triangular current sheet at point in local frame.
#[inline]
#[allow(non_snake_case)]
#[replace_float_literals(T::from_f64(literal).unwrap())]
pub fn local_triangle_current_B<T: Float>(
    point: Point3<T>,
    current_density: Vector3<T>,
    vertices: &[Vector3<T>; 3],
) -> Vector3<T> {
    let translation = vertices[0];
    let v1 = vertices[1] - translation;
    let v2 = vertices[2] - translation;

    let u1 = v1.norm();
    if u1 < 1e-15 {
        return Vector3::zeros();
    }

    let ex = v1 / u1;
    let cross = ex.cross(&v2);
    let n_norm = cross.norm();
    if n_norm < 1e-15 {
        return Vector3::zeros();
    }
    let ez = cross / n_norm;
    let ey = ez.cross(&ex);

    let point_trans = point.coords - translation;
    let x = point_trans.dot(&ex);
    let y = point_trans.dot(&ey);
    let z = point_trans.dot(&ez);

    let u2 = v2.dot(&ex);
    let v_2 = v2.dot(&ey);

    let ju = current_density.dot(&ex);
    let jv = current_density.dot(&ey);

    let H_local = elementar_current_sheet_Hfield(x, y, z, u1, u2, v_2, ju, jv);

    // Transform back H
    let H_global = ex * H_local.x + ey * H_local.y + ez * H_local.z;

    // B = H * mu0
    let B = H_global * (4e-7 * T::pi());

    if B.x.is_nan() || B.y.is_nan() || B.z.is_nan() {
        Vector3::zeros()
    } else {
        B
    }
}

/// Computes B-field of a triangular current sheet at point (x, y, z).
#[inline]
#[allow(non_snake_case)]
pub fn triangle_current_B<T: Float>(
    point: Point3<T>,
    position: Point3<T>,
    orientation: UnitQuaternion<T>,
    current_density: Vector3<T>,
    vertices: [Vector3<T>; 3],
) -> Vector3<T> {
    compute_in_local!(
        local_triangle_current_B,
        point,
        position,
        orientation,
        (current_density, &vertices),
    )
}

/// Computes B-field at points in global frame for a triangular current sheet.
#[allow(non_snake_case)]
pub fn triangle_current_B_batch<T: Float>(
    points: &[Point3<T>],
    position: Point3<T>,
    orientation: UnitQuaternion<T>,
    current_density: Vector3<T>,
    vertices: [Vector3<T>; 3],
    out: &mut [Vector3<T>],
) {
    impl_parallel!(
        triangle_current_B,
        rayon_threshold: 100,
        input: points,
        output: out,
        args: [position, orientation, current_density, vertices]
    )
}

/// Computes B-field at each given points in global frame for multiple triangles.
#[allow(non_snake_case)]
pub fn sum_multiple_triangle_current_B<T: Float>(
    points: &[Point3<T>],
    positions: &[Point3<T>],
    orientations: &[UnitQuaternion<T>],
    current_densities: &[Vector3<T>],
    vertices_list: &[[Vector3<T>; 3]],
    out: &mut [Vector3<T>],
) {
    impl_parallel_sum!(
        out,
        points,
        60,
        [positions, orientations, current_densities, vertices_list],
        |pos, p, o, pol, vert| triangle_current_B(*pos, *p, *o, *pol, *vert)
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{point, vector};

    #[test]
    fn test_sum_multiple_triangle_current_b() {
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
        let current_densities = &[vector![0.45, 0.3, 0.15], vector![1.0, 2.0, 3.0]];
        let vertices_list = &[
            [
                vector![0.0, 0.0, 0.0],
                vector![0.0, 0.0, 1.0],
                vector![1.0, 0.0, 0.0],
            ],
            [
                vector![0.0, 0.0, 0.0],
                vector![0.0, 1.0, 0.0],
                vector![0.0, 0.0, 1.0],
            ],
        ];

        impl_test_sum_multiple!(
            sum_multiple_triangle_current_B,
            1e-15,
            points,
            positions,
            orientations,
            (current_densities, vertices_list),
            |p, pos, ori, pol, vert| triangle_current_B(p, pos, ori, pol, vert)
        );
    }
}
