//! Shared Coulomb surface integral for triangular magnets and current sheets.

use crate::base::Float;
use nalgebra::{Point3, Vector3};
use num_traits::Float as NumFloat;

#[derive(Clone, Copy)]
pub(super) struct TriangleSurface<T: Float> {
    vertices: [Vector3<T>; 3],
    directions: [Vector3<T>; 3],
    lengths: [T; 3],
    area_vector: Vector3<T>,
    pub normal: Vector3<T>,
}

impl<T: Float> TriangleSurface<T> {
    pub fn new(vertices: [Vector3<T>; 3]) -> Option<Self> {
        let edges = [
            vertices[1] - vertices[0],
            vertices[2] - vertices[1],
            vertices[0] - vertices[2],
        ];
        let area_vector = edges[0].cross(&(vertices[2] - vertices[0]));
        let area = area_vector.norm();
        if area == T::zero() {
            return None;
        }
        let lengths = edges.map(|edge| edge.norm());
        Some(Self {
            vertices,
            lengths,
            directions: core::array::from_fn(|i| edges[i] / lengths[i]),
            area_vector,
            normal: area_vector / area,
        })
    }

    /// Oriented solid angle without the edge integrals needed for a field.
    #[inline]
    pub fn solid_angle(&self, point: Point3<T>) -> T {
        let r = self.vertices.map(|v| v - point.coords);
        let distances = r.map(|v| v.norm());
        self.solid_angle_from_vectors(&r, &distances)
    }

    #[inline]
    #[num_lazy_proc::generic_float(T)]
    fn solid_angle_from_vectors(&self, r: &[Vector3<T>; 3], distances: &[T; 3]) -> T {
        // Use source edges for the determinant. Cross products of distant
        // observer-to-vertex vectors discard the triangle's small area.
        let numerator = -r[0].dot(&self.area_vector);
        let numerator = if numerator == T::zero() {
            T::zero()
        } else {
            numerator
        };
        let denominator = distances[0] * distances[1] * distances[2]
            + r[2].dot(&r[1]) * distances[0]
            + r[2].dot(&r[0]) * distances[1]
            + r[1].dot(&r[0]) * distances[2];
        2.0 * NumFloat::atan2(numerator, denominator)
    }

    /// Integral of R/|R|³ divided by 4π, and the oriented solid angle.
    #[inline]
    #[num_lazy_proc::generic_float(T)]
    pub fn field_with_solid_angle(&self, point: Point3<T>) -> (Vector3<T>, T) {
        let r = self.vertices.map(|v| v - point.coords);
        let distances = r.map(|v| v.norm());
        let omega = self.solid_angle_from_vectors(&r, &distances);
        let mut edge_integral = Vector3::zeros();
        for i in 0..3 {
            let length = self.lengths[i];
            let direction = self.directions[i];
            let sum = distances[i] + distances[(i + 1) % 3];
            let ratio = length / sum;
            let integral = if ratio > 0.5 {
                // Near an edge, sum - length loses the squared perpendicular
                // distance. Endpoint asinh values retain that distance.
                let along = r[i].dot(&direction);
                let perpendicular = r[i].cross(&direction).norm();
                if perpendicular == T::zero() && (along > T::zero() || along + length < T::zero()) {
                    NumFloat::ln_1p(2.0 * length / (sum - length))
                } else {
                    NumFloat::asinh((along + length) / perpendicular)
                        - NumFloat::asinh(along / perpendicular)
                }
            } else {
                // log((sum + length)/(sum - length)), without rounding the
                // ratio to 1 in the far field.
                NumFloat::ln_1p(2.0 * length / (sum - length))
            };
            edge_integral += direction * integral;
        }
        let field = (self.normal * omega - self.normal.cross(&edge_integral)) / (4.0 * PI);
        (field, omega)
    }
}
