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

#[derive(Clone, Copy)]
pub(crate) struct PrecomputedTriangleCurrent<T: Float> {
    translation: Vector3<T>,
    ex: Vector3<T>,
    ey: Vector3<T>,
    ez: Vector3<T>,
    u1: T,
    u2: T,
    v2: T,
    u1_2: T,
    u2_2: T,
    v2_2: T,
    sqrt4: T,
    sqrt5: T,
    ju: T,
    jv: T,
    ju_u1_u2_jv_v2_over_sqrt4: T,
    ju_u2_jv_v2_over_sqrt5: T,
    factor: T,
    u1_v2: T,
}

#[allow(non_snake_case)]
impl<T: Float> PrecomputedTriangleCurrent<T> {
    #[replace_float_literals(T::from_f64(literal).unwrap())]
    pub fn new(current_density: Vector3<T>, vertices: &[Vector3<T>; 3]) -> Option<Self> {
        if current_density == Vector3::zeros() {
            return None;
        }

        let translation = vertices[0];
        let v1 = vertices[1] - translation;
        let v2_vec = vertices[2] - translation;

        let u1 = v1.norm();
        if u1 < 1e-15 {
            return None;
        }

        let ex = v1 / u1;
        let cross = ex.cross(&v2_vec);
        let n_norm = cross.norm();
        if n_norm < 1e-15 {
            return None;
        }
        let ez = cross / n_norm;
        let ey = ez.cross(&ex);

        let u2 = v2_vec.dot(&ex);
        let v2 = v2_vec.dot(&ey);

        let ju = current_density.dot(&ex);
        let jv = current_density.dot(&ey);

        let u1_2 = u1 * u1;
        let u2_2 = u2 * u2;
        let v2_2 = v2 * v2;

        let sqrt4 = NumFloat::sqrt(u1_2 - 2.0 * u1 * u2 + u2_2 + v2_2);
        let sqrt5 = NumFloat::sqrt(u2_2 + v2_2);

        let ju_u1_u2_jv_v2 = ju * (u1 - u2) - jv * v2;
        let ju_u2_jv_v2 = ju * u2 + jv * v2;

        let ju_u1_u2_jv_v2_over_sqrt4 = ju_u1_u2_jv_v2 / sqrt4;
        let ju_u2_jv_v2_over_sqrt5 = ju_u2_jv_v2 / sqrt5;

        let factor = u1 * v2 * T::mu0_4pi();
        let u1_v2 = u1 * v2;

        Some(Self {
            translation,
            ex,
            ey,
            ez,
            u1,
            u2,
            v2,
            u1_2,
            u2_2,
            v2_2,
            sqrt4,
            sqrt5,
            ju,
            jv,
            ju_u1_u2_jv_v2_over_sqrt4,
            ju_u2_jv_v2_over_sqrt5,
            factor,
            u1_v2,
        })
    }

    #[inline]
    #[replace_float_literals(T::from_f64(literal).unwrap())]
    pub fn compute_B(&self, point: Point3<T>) -> Vector3<T> {
        let point_trans = point.coords - self.translation;
        let x = point_trans.dot(&self.ex);
        let y = point_trans.dot(&self.ey);
        let mut z = point_trans.dot(&self.ez);

        if NumFloat::abs(z) < 1e-15 {
            z = if z < 0.0 { -1e-15 } else { 1e-15 };
        }

        let y_2 = y * y;
        let z_2 = z * z;
        let yz2 = y_2 + z_2;
        let x_2 = x * x;
        let r2 = x_2 + yz2;

        let sqrt1 = NumFloat::sqrt(r2);
        let sqrt2 = NumFloat::sqrt(self.u1_2 - 2.0 * self.u1 * x + r2);
        let sqrt3 = NumFloat::sqrt(self.u2_2 - 2.0 * self.u2 * x + self.v2_2 - 2.0 * self.v2 * y + r2);

        let v2_z = self.v2 * z;

        let H_x = (NumFloat::atan((-self.u2 * yz2 + self.v2 * x * y) / (v2_z * sqrt1))
            + NumFloat::atan((self.v2 * y * (self.u1 - x) - (self.u1 - self.u2) * yz2) / (v2_z * sqrt2))
            - NumFloat::atan((-self.u2 * yz2 - self.v2_2 * x + self.v2 * y * (self.u2 + x)) / (v2_z * sqrt3))
            - NumFloat::atan(
                (-self.u1 * (self.v2_2 - 2.0 * self.v2 * y + yz2) + self.u2 * yz2 + self.v2_2 * x - self.v2 * y * (self.u2 + x))
                    / (v2_z * sqrt3),
            ))
            / (self.u1 * v2_z);

        let H_z = -(self.ju * NumFloat::atanh(x / sqrt1) + self.ju * NumFloat::atanh((self.u1 - x) / sqrt2)
            - self.ju_u1_u2_jv_v2_over_sqrt4
                * NumFloat::atanh((self.u1_2 - self.u1 * (self.u2 + x) + self.u2 * x + self.v2 * y) / (self.sqrt4 * sqrt2))
            + self.ju_u1_u2_jv_v2_over_sqrt4
                * NumFloat::atanh((self.u1 * (self.u2 - x) - self.u2_2 + self.u2 * x + self.v2 * (-self.v2 + y)) / (self.sqrt4 * sqrt3))
            + self.ju_u2_jv_v2_over_sqrt5 * NumFloat::atanh((-self.u2 * x - self.v2 * y) / (self.sqrt5 * sqrt1))
            - self.ju_u2_jv_v2_over_sqrt5 * NumFloat::atanh((self.u2_2 - self.u2 * x + self.v2 * (self.v2 - y)) / (self.sqrt5 * sqrt3)))
            / self.u1_v2;

        let factor_z = self.factor * z;
        let B_local_x = H_x * self.jv * factor_z;
        let B_local_y = -H_x * self.ju * factor_z;
        let B_local_z = H_z * self.factor;

        let B = self.ex * B_local_x + self.ey * B_local_y + self.ez * B_local_z;
        if B.x.is_nan() || B.y.is_nan() || B.z.is_nan() {
            Vector3::zeros()
        } else {
            B
        }
    }
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
    match PrecomputedTriangleCurrent::new(current_density, vertices) {
        Some(pre) => pre.compute_B(point),
        None => Vector3::zeros(),
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
    let pre = match PrecomputedTriangleCurrent::new(current_density, &vertices) {
        Some(p) => p,
        None => {
            out.fill(Vector3::zeros());
            return;
        }
    };
    let inv_orientation = orientation.inverse();
    impl_parallel!(
        rayon_threshold: 100,
        input: points,
        output: out,
        |p| {
            let local_point = inv_orientation * Point3::from(p.coords - position.coords);
            let local_b = pre.compute_B(local_point);
            orientation * local_b
        }
    );
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

    #[test]
    fn test_triangle_current_edge_cases_and_batch() {
        let verts = [
            vector![0.0, 0.0, 0.0],
            vector![1.0, 0.0, 0.0],
            vector![0.0, 1.0, 0.0],
        ];

        // Small negative z (|z| < 1e-15, z < 0)
        let b_neg_z =
            local_triangle_current_B(point![0.2, 0.2, -1e-16], vector![1.0, 0.0, 0.0], &verts);
        assert!(b_neg_z.z.is_finite());

        // Zero current density
        let b_zero_curr =
            local_triangle_current_B(point![0.2, 0.2, 1.0], vector![0.0, 0.0, 0.0], &verts);
        assert_eq!(b_zero_curr, Vector3::zeros());

        // Duplicate vertices 0 and 1 (u1 < 1e-15)
        let b_dup_v = local_triangle_current_B(
            point![0.2, 0.2, 1.0],
            vector![1.0, 0.0, 0.0],
            &[
                vector![0.0, 0.0, 0.0],
                vector![0.0, 0.0, 0.0],
                vector![0.0, 1.0, 0.0],
            ],
        );
        assert_eq!(b_dup_v, Vector3::zeros());

        // Collinear vertices (n_norm < 1e-15)
        let b_collinear = local_triangle_current_B(
            point![0.2, 0.2, 1.0],
            vector![1.0, 0.0, 0.0],
            &[
                vector![0.0, 0.0, 0.0],
                vector![1.0, 0.0, 0.0],
                vector![2.0, 0.0, 0.0],
            ],
        );
        assert_eq!(b_collinear, Vector3::zeros());

        // Batch with > 100 points for Rayon threshold
        let points = vec![point![0.0, 0.0, 5.0]; 120];
        let mut out = vec![Vector3::zeros(); 120];
        triangle_current_B_batch(
            &points,
            Point3::origin(),
            UnitQuaternion::identity(),
            vector![1.0, 0.0, 0.0],
            verts,
            &mut out,
        );
        assert_eq!(out.len(), 120);
        assert!(out[0].z != 0.0);
    }
}
