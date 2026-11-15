/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Magnetic flux density of homogeneously polarized annular cylinder sectors.
//!
//! The field is obtained from the magnetic surface-charge representation
//!
//! `B = J (inside) + (1 / 4π) ∫ (J · n) (p - q) / |p - q|³ dS`.
//!
//! Integrating the axial coordinate analytically on the cylindrical and radial
//! walls, and the radius analytically on the end caps, leaves only smooth,
//! one-dimensional integrals. Adaptive quadrature handles the remaining
//! integrals without the many separate elliptic-integral singularity cases of
//! the Slanovc formulation used by MagPyLib.

use nalgebra::{Point3, SVector, UnitQuaternion, Vector3, vector};
use num_traits::Float as NumFloat;

use crate::{
    base::{Float, coordinate::compute_in_local},
    crate_utils::impl_parallel,
    fields::field_cylinder::local_cylinder_B,
};

const PI: f64 = core::f64::consts::PI;
const TWO_PI: f64 = 2.0 * PI;
const INV_FOUR_PI: f64 = 1.0 / (4.0 * PI);

#[inline]
fn positive_remainder(value: f64, period: f64) -> f64 {
    let remainder = value - period * NumFloat::floor(value / period);
    if remainder >= period { 0.0 } else { remainder }
}

/// Simpson quadrature with error control on each Cartesian component.
///
/// The input functions are analytic in one integration variable except at
/// segment boundary points, which are handled before integration.
#[inline]
fn simpson(a: f64, b: f64, fa: Vector3<f64>, fm: Vector3<f64>, fb: Vector3<f64>) -> Vector3<f64> {
    (fa + fm * 4.0 + fb) * ((b - a) / 6.0)
}

#[derive(Clone, Copy)]
struct SimpsonInterval {
    a: f64,
    b: f64,
    fa: Vector3<f64>,
    fm: Vector3<f64>,
    fb: Vector3<f64>,
    estimate: Vector3<f64>,
}

impl SimpsonInterval {
    #[inline]
    fn new(a: f64, b: f64, fa: Vector3<f64>, fm: Vector3<f64>, fb: Vector3<f64>) -> Self {
        Self {
            a,
            b,
            fa,
            fm,
            fb,
            estimate: simpson(a, b, fa, fm, fb),
        }
    }
}

fn refine<F: Fn(f64) -> Vector3<f64>>(
    f: &F,
    interval: SimpsonInterval,
    tolerance: f64,
    depth: u32,
) -> Vector3<f64> {
    let SimpsonInterval {
        a,
        b,
        fa,
        fm,
        fb,
        estimate,
    } = interval;
    if !estimate.iter().all(|component| component.is_finite()) {
        return Vector3::repeat(f64::NAN);
    }
    let mid = (a + b) * 0.5;
    let fl = f((a + mid) * 0.5);
    let fr = f((mid + b) * 0.5);
    let left = SimpsonInterval::new(a, mid, fa, fl, fm);
    let right = SimpsonInterval::new(mid, b, fm, fr, fb);
    let finer = left.estimate + right.estimate;
    let delta = finer - estimate;
    let error = delta.x.abs().max(delta.y.abs()).max(delta.z.abs());
    // Once the change falls below f64 arithmetic noise in the local integral,
    // further subdivisions cannot improve the answer. This matters close to
    // the curved walls, where cancellation otherwise exhausts recursion.
    let scale = finer.x.abs().max(finer.y.abs()).max(finer.z.abs());
    if error <= 15.0 * tolerance.max(5e-14 * scale) {
        return finer + delta / 15.0;
    }
    // Do not report an unconverged estimate as a field. This also bounds work
    // for non-finite inputs and intervals below floating-point resolution.
    if depth == 0 || (a + mid) * 0.5 == a || (mid + b) * 0.5 == b {
        return Vector3::repeat(f64::NAN);
    }
    refine(f, left, tolerance * 0.5, depth - 1) + refine(f, right, tolerance * 0.5, depth - 1)
}

fn integrate<F: Fn(f64) -> Vector3<f64>>(f: &F, a: f64, b: f64) -> Vector3<f64> {
    if a >= b {
        return Vector3::zeros();
    }
    let mid = (a + b) * 0.5;
    let fa = f(a);
    let fm = f(mid);
    let fb = f(b);
    refine(f, SimpsonInterval::new(a, b, fa, fm, fb), 2e-12, 48)
}

/// Resolve a narrow surface kernel before applying the quadrature error test.
/// Each side of the nearest point is divided into geometrically growing panels,
/// starting at the distance to the surface in the integration coordinate.
fn integrate_near<F: Fn(f64) -> Vector3<f64>>(
    f: &F,
    a: f64,
    b: f64,
    nearest: f64,
    width: f64,
) -> Vector3<f64> {
    let nearest = nearest.clamp(a, b);
    let mut total = Vector3::zeros();
    for (end, direction) in [(a, -1.0), (b, 1.0)] {
        let length = (end - nearest).abs();
        let mut offset = 0.0;
        let mut step = width.max(32.0 * f64::EPSILON * (1.0 + nearest.abs()));
        while offset < length {
            let next = (offset + step).min(length);
            let left = nearest + direction * offset;
            let right = nearest + direction * next;
            total += integrate(f, left.min(right), left.max(right));
            offset = next;
            step *= 2.0;
        }
    }
    total
}

/// Integrate around the closest periodic copy of the observer's azimuth.
fn integrate_angle<F: Fn(f64) -> Vector3<f64>>(
    f: &F,
    start: f64,
    end: f64,
    azimuth: f64,
    width: f64,
) -> Vector3<f64> {
    let split = start + positive_remainder(azimuth - start, TWO_PI);
    // When the observer is outside the sector, the closest azimuth can lie
    // just below start instead of near the other end of the circle.
    let nearest = if split - end > start - (split - TWO_PI) {
        split - TWO_PI
    } else {
        split
    };
    // Use offsets from the azimuth so a tiny angular displacement is not
    // rounded away by adding it to an angle of order one.
    integrate_near(f, start - nearest, end - nearest, 0.0, width)
}

// Eight-point Gauss-Legendre rule. At more than eight bounding radii away,
// the volume dipole kernel is smooth and this rule resolves its spatial
// variation without subtracting nearly equal charged-surface contributions.
const GAUSS: [(f64, f64); 8] = [
    (-0.9602898564975363, 0.1012285362903763),
    (-0.7966664774136267, 0.2223810344533745),
    (-0.525532409916329, 0.3137066458778873),
    (-0.1834346424956498, 0.362683783378362),
    (0.1834346424956498, 0.362683783378362),
    (0.525532409916329, 0.3137066458778873),
    (0.7966664774136267, 0.2223810344533745),
    (0.9602898564975363, 0.1012285362903763),
];

fn distant_field(
    point: Vector3<f64>,
    polarization: Vector3<f64>,
    inner: f64,
    half_height: f64,
    start: f64,
    span: f64,
) -> Vector3<f64> {
    let distance = NumFloat::hypot(NumFloat::hypot(point.x, point.y), point.z);
    let observer = point / distance;
    let radial_half = (1.0 - inner) * 0.5;
    let radial_mid = inner + radial_half;
    let panels = NumFloat::ceil(span / (PI * 0.5)) as usize;
    let angle_half = span / (2.0 * panels as f64);
    let mut total = Vector3::zeros();
    for panel in 0..panels {
        let angle_mid = start + (2 * panel + 1) as f64 * angle_half;
        for (na, wa) in GAUSS {
            let angle = angle_mid + angle_half * na;
            let co = NumFloat::cos(angle);
            let si = NumFloat::sin(angle);
            for (nr, wr) in GAUSS {
                let rho = radial_mid + radial_half * nr;
                for (nz, wz) in GAUSS {
                    let offset =
                        observer - vector![rho * co, rho * si, half_height * nz] / distance;
                    let squared = offset.norm_squared();
                    let kernel = (offset * (3.0 * polarization.dot(&offset) / squared)
                        - polarization)
                        / (squared * NumFloat::sqrt(squared));
                    total += kernel * (wa * wr * wz * rho);
                }
            }
        }
    }
    total
        * (INV_FOUR_PI * radial_half * angle_half * (half_height / distance) / distance / distance)
}

/// Analytic integral over the height of a wall, for fixed transverse offset.
/// Returns the coefficients of `(x-x', y-y', z-z') / distance³`.
#[inline]
fn integrated_height(distance_sq: f64, z: f64, half_height: f64) -> (f64, f64) {
    let low = z + half_height;
    let high = z - half_height;
    let d_low = NumFloat::sqrt(distance_sq + low * low);
    let d_high = NumFloat::sqrt(distance_sq + high * high);

    // Rationalization prevents losing all precision when the observer is
    // directly above/below a wall (distance_sq -> 0).
    let horizontal = if low * high > 0.0 {
        let numerator = low * low - high * high;
        let denominator = d_low * d_high * (low.abs() * d_high + high.abs() * d_low);
        low.signum() * numerator / denominator
    } else {
        (low / d_low - high / d_high) / distance_sq
    };
    let vertical = 1.0 / d_high - 1.0 / d_low;
    (horizontal, vertical)
}

/// The radial antiderivatives of `ρ (p-q) / |p-q|³` on a horizontal end cap.
/// Here `along = x cosθ + y sinθ`, `across = y cosθ - x sinθ`, and `w=z-z_cap`.
#[inline]
fn integrated_radius(
    inner: f64,
    outer: f64,
    along: f64,
    across: f64,
    w: f64,
    cos_theta: f64,
    sin_theta: f64,
) -> Vector3<f64> {
    let s1 = inner - along;
    let s2 = outer - along;
    let transverse_sq = across * across + w * w;
    let d1 = NumFloat::sqrt(s1 * s1 + transverse_sq);
    let d2 = NumFloat::sqrt(s2 * s2 + transverse_sq);

    // A = integral ρ / distance³ dρ. Rationalize the difference of
    // s/sqrt(s²+d²) if both limits lie on the same side of the observer.
    let a = if s1 * s2 > 0.0 {
        let denom = d1 * d2 * (s2 * d1 + s1 * d2);
        along * (s2 * s2 - s1 * s1) / denom + 1.0 / d1 - 1.0 / d2
    } else {
        along * (s2 / d2 - s1 / d1) / transverse_sq + 1.0 / d1 - 1.0 / d2
    };

    // C = integral ρ(along-ρ) / distance³ dρ.
    // The asinh difference is expressed as logarithms on equal-sign intervals
    // so it remains well-defined when the transverse distance is zero.
    let asinh_difference = if s1 >= 0.0 && s2 > 0.0 {
        NumFloat::ln((s2 + d2) / (s1 + d1))
    } else if s1 < 0.0 && s2 <= 0.0 {
        NumFloat::ln((-s1 + d1) / (-s2 + d2))
    } else {
        let transverse = NumFloat::sqrt(transverse_sq);
        NumFloat::asinh(s2 / transverse) - NumFloat::asinh(s1 / transverse)
    };
    let c = outer / d2 - inner / d1 - asinh_difference;

    vector![
        cos_theta * c - sin_theta * across * a,
        sin_theta * c + cos_theta * across * a,
        w * a,
    ]
}

#[inline]
fn on_boundary(value: f64, bound: f64, tolerance: f64) -> bool {
    (value - bound).abs() <= tolerance
}

/// Computes the B-field (T) of a cylinder segment at a point in local coordinates.
///
/// `dimensions = [inner radius, outer radius, height, start angle, end angle]`;
/// lengths are in metres, angles in degrees, and polarization in tesla.
/// The axis passes through the origin, parallel to +z; the section begins at
/// the plane spanned by +x and +z. On the segment surface, B is set to zero,
/// following MagPyLib's convention for the undefined field at discontinuities.
///
/// The surface integrals are exact reductions to one dimension; adaptive
/// quadrature controls the residual integration error. A complete circular
/// cylinder or ring uses the existing elliptic-integral kernel nearby. Far
/// outside the bounding sphere, volume dipole quadrature avoids cancellation.
/// If adaptive quadrature cannot converge within floating-point resolution or
/// its recursion limit, the result contains NaN components.
///
/// # References
/// - Slanovc, F., *Journal of Magnetism and Magnetic Materials* **559** (2022), 169482.
/// - MagPyLib `field_BH_cylinder_segment.py`,
///   <https://github.com/magpylib/magpylib/blob/main/src/magpylib/_src/fields/field_BH_cylinder_segment.py>.
#[allow(non_snake_case)]
pub fn local_cylinder_segment_B<T: Float>(
    point: Point3<T>,
    polarization: Vector3<T>,
    dimensions: SVector<T, 5>,
) -> Vector3<T> {
    if polarization == Vector3::zeros() {
        return Vector3::zeros();
    }

    let inner_radius = dimensions[0];
    let outer_radius = dimensions[1];
    let height = dimensions[2];
    let angular_span = dimensions[4] - dimensions[3];
    if angular_span <= T::zero() || outer_radius <= T::zero() || height <= T::zero() {
        return Vector3::zeros();
    }

    // Work with dimensionless f64 geometry even for f32 callers: it avoids
    // overflow in high powers of dimensional distances and preserves accuracy
    // for very small or very large magnets without requiring allocation.
    let scale = outer_radius.to_f64().unwrap();
    let inner = inner_radius.to_f64().unwrap() / scale;
    let outer = 1.0;
    let h = height.to_f64().unwrap() / scale;
    let half_h = 0.5 * h;
    let x = point.x.to_f64().unwrap() / scale;
    let y = point.y.to_f64().unwrap() / scale;
    let z = point.z.to_f64().unwrap() / scale;
    let r = NumFloat::sqrt(x * x + y * y);
    let azimuth = NumFloat::atan2(y, x);
    let (cos_azimuth, sin_azimuth) = if r > 0.0 { (x / r, y / r) } else { (1.0, 0.0) };
    let start_deg = dimensions[3].to_f64().unwrap();
    let span = angular_span.to_f64().unwrap() * (PI / 180.0);
    let start = positive_remainder(start_deg, 360.0) * (PI / 180.0);
    let end = start + span;
    let relative = positive_remainder(azimuth - start, TWO_PI);
    let full = angular_span >= T::from(360.0).unwrap();
    let inside_angle = full || relative <= span || (inner == 0.0 && r == 0.0);
    let inside_radius = inner <= r && r <= outer;
    let inside_height = z.abs() <= half_h;
    let inside = inside_angle && inside_radius && inside_height;

    // Scale each surface tolerance by its own dimension, and keep opposite
    // boundary bands disjoint even for thin rings, caps, or angular sectors.
    let epsilon = (4.0 * <T as NumFloat>::epsilon().to_f64().unwrap()).max(32.0 * f64::EPSILON);
    let radial_tolerance = epsilon.min((outer - inner) * 0.25);
    let cap_tolerance = epsilon * half_h;
    let angle_tolerance = (epsilon * span).min(span * 0.25);
    let on_cap = on_boundary(z.abs(), half_h, cap_tolerance) && inside_radius && inside_angle;
    let on_cylinder = ((inner > 0.0 && on_boundary(r, inner, radial_tolerance * inner))
        || on_boundary(r, outer, radial_tolerance))
        && inside_height
        && inside_angle;
    let on_cut = !full
        && (r == 0.0 && inner == 0.0
            || relative.min(TWO_PI - relative) <= angle_tolerance
            || (relative - span).abs() <= angle_tolerance)
        && inside_radius
        && inside_height;
    if on_cap || on_cylinder || on_cut {
        return Vector3::zeros();
    }

    let jx = polarization.x.to_f64().unwrap();
    let jy = polarization.y.to_f64().unwrap();
    let jz = polarization.z.to_f64().unwrap();
    let magnitude = jx.abs().max(jy.abs()).max(jz.abs());
    let mx = jx / magnitude;
    let my = jy / magnitude;
    let mz = jz / magnitude;

    let local_point = vector![x, y, z];
    let normalized_polarization = vector![mx, my, mz];
    let to_output =
        |field: Vector3<f64>| field.map(|component| T::from(component * magnitude).unwrap());
    if NumFloat::hypot(r, z) > 8.0 * NumFloat::hypot(1.0, half_h) {
        return to_output(distant_field(
            local_point,
            normalized_polarization,
            inner,
            half_h,
            start,
            span,
        ));
    }

    // Check the segment boundary before using the cylinder kernel: its surface
    // convention differs, and the subtracted inner cylinder has artificial cap
    // surfaces inside the ring's hole.
    if full {
        let local_point = Point3::from(local_point);
        let mut field = local_cylinder_B(local_point, normalized_polarization, outer, h);
        if inner > 0.0 {
            field -= local_cylinder_B(local_point, normalized_polarization, inner, h);
        }
        return to_output(field);
    }

    let mut total = Vector3::zeros();
    if mx != 0.0 || my != 0.0 {
        // Exterior and interior cylindrical walls, normals +/- e_r.
        for (wall_radius, sign) in [(outer, 1.0), (inner, -1.0)] {
            if wall_radius == 0.0 {
                continue;
            }
            let wall = |offset: f64| {
                let co = NumFloat::cos(offset);
                let si = NumFloat::sin(offset);
                // r-R*cos(offset) loses the normal distance near a wall.
                let radial =
                    r - wall_radius + 2.0 * wall_radius * NumFloat::sin(offset * 0.5).powi(2);
                let tangential = -wall_radius * si;
                let distance_sq = radial * radial + tangential * tangential;
                let (horizontal, vertical) = integrated_height(distance_sq, z, half_h);
                let charge = sign
                    * wall_radius
                    * (mx * (cos_azimuth * co - sin_azimuth * si)
                        + my * (sin_azimuth * co + cos_azimuth * si));
                vector![
                    (radial * cos_azimuth - tangential * sin_azimuth) * horizontal,
                    (radial * sin_azimuth + tangential * cos_azimuth) * horizontal,
                    vertical,
                ] * charge
            };
            let gap = NumFloat::hypot(r - wall_radius, (z.abs() - half_h).max(0.0));
            let width = gap / NumFloat::sqrt(r * wall_radius).max(f64::MIN_POSITIVE);
            total += integrate_angle(&wall, start, end, azimuth, width);
        }

        // Two flat radial walls, normals -e_theta and +e_theta.
        for (theta, sign) in [(start, -1.0), (end, 1.0)] {
            let co = NumFloat::cos(theta);
            let si = NumFloat::sin(theta);
            let charge = sign * (-mx * si + my * co);
            if charge == 0.0 {
                continue;
            }
            let along = x * co + y * si;
            let across = y * co - x * si;
            let radial_wall = |offset: f64| {
                let (horizontal, vertical) =
                    integrated_height(offset * offset + across * across, z, half_h);
                vector![
                    (-offset * co - across * si) * horizontal,
                    (-offset * si + across * co) * horizontal,
                    vertical,
                ] * charge
            };
            let width = NumFloat::hypot(across, (z.abs() - half_h).max(0.0));
            total += integrate_near(&radial_wall, inner - along, outer - along, 0.0, width);
        }
    }

    if mz != 0.0 {
        // End caps have normals +/- e_z. Their radial integrals are analytic.
        for (cap_z, sign) in [(half_h, 1.0), (-half_h, -1.0)] {
            let cap = |offset: f64| {
                let co = NumFloat::cos(offset);
                let si = NumFloat::sin(offset);
                let along = r * co;
                let across = -r * si;
                let cos_theta = cos_azimuth * co - sin_azimuth * si;
                let sin_theta = sin_azimuth * co + cos_azimuth * si;
                integrated_radius(inner, outer, along, across, z - cap_z, cos_theta, sin_theta)
                    * (mz * sign)
            };
            let radial_gap = (inner - r).max(r - outer).max(0.0);
            let width = NumFloat::hypot(z - cap_z, radial_gap) / r.max(f64::MIN_POSITIVE);
            total += integrate_angle(&cap, start, end, azimuth, width);
        }
    }

    total *= magnitude * INV_FOUR_PI;
    if inside {
        total.x += jx;
        total.y += jy;
        total.z += jz;
    }
    vector![
        T::from(total.x).unwrap(),
        T::from(total.y).unwrap(),
        T::from(total.z).unwrap(),
    ]
}

/// Computes the B-field (T) of a cylinder segment in global coordinates.
///
/// `dimensions = [r_inner, r_outer, height, phi_start, phi_end]`, in metres
/// and degrees. The polarization is specified in the magnet's local frame.
///
/// # Arguments
/// - `point`: Observer position in global coordinates (m)
/// - `position`: Center of the full cylinder in global coordinates (m)
/// - `orientation`: Magnet orientation as a unit quaternion
/// - `polarization`: Local polarization vector (T)
/// - `dimensions`: Inner/outer radii (m), height (m), start/end azimuths (degrees)
///
/// # Examples
/// ```
/// # use approx::assert_relative_eq;
/// # use magba::fields::cylinder_segment_B;
/// # use nalgebra::{Point3, UnitQuaternion, point, vector};
/// let b = cylinder_segment_B(
///     point![0.03, 0.01, 0.03],
///     Point3::origin(),
///     UnitQuaternion::identity(),
///     vector![0.2, -0.3, 0.4],
///     vector![0.0, 0.02, 0.02, -20.0, 100.0],
/// );
/// assert_relative_eq!(
///     b,
///     vector![0.006546273267888944, 0.0054657542981008155, 0.007359158272544878],
///     epsilon = 5e-8,
/// );
/// ```
///
/// # Numerical method
/// The wall-height and cap-radius integrals have closed forms; the remaining
/// one-dimensional integrals use adaptive quadrature. This is slower than the
/// complete cylinder's elliptic-integral formulas, particularly very close
/// to sector edges, but avoids elliptic branch singularities. Distant observers
/// use Gauss-Legendre quadrature of the volume dipole kernel to avoid surface
/// cancellation. A failed adaptive quadrature returns NaN components.
///
/// # References
/// - Slanovc, F., *Journal of Magnetism and Magnetic Materials* **559** (2022), 169482.
/// - MagPyLib's CylinderSegment implementation:
///   <https://github.com/magpylib/magpylib/blob/main/src/magpylib/_src/fields/field_BH_cylinder_segment.py>.
#[allow(non_snake_case)]
#[inline]
pub fn cylinder_segment_B<T: Float>(
    point: Point3<T>,
    position: Point3<T>,
    orientation: UnitQuaternion<T>,
    polarization: Vector3<T>,
    dimensions: SVector<T, 5>,
) -> Vector3<T> {
    compute_in_local!(
        local_cylinder_segment_B,
        point,
        position,
        orientation,
        (polarization, dimensions),
    )
}

/// Computes the B-field (T) at multiple global observer positions.
///
/// Arguments are the same as [`cylinder_segment_B`], except that `points` is
/// the observer slice and `out` receives the B-field vectors in input order.
/// The implementation can parallelize large point sets when Rayon is enabled.
#[allow(non_snake_case)]
pub fn cylinder_segment_B_batch<T: Float>(
    points: &[Point3<T>],
    position: Point3<T>,
    orientation: UnitQuaternion<T>,
    polarization: Vector3<T>,
    dimensions: SVector<T, 5>,
    out: &mut [Vector3<T>],
) {
    let inv_orientation = orientation.inverse();
    impl_parallel!(
        rayon_threshold: 40,
        input: points,
        output: out,
        |p| {
            let local_point = inv_orientation * Point3::from(p.coords - position.coords);
            let local_b = local_cylinder_segment_B(local_point, polarization, dimensions);
            orientation * local_b
        }
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{point, vector};

    #[test]
    fn volume_and_surface_integrals_agree_at_far_field_transition() {
        let direction = vector![7.0, 1.0, 2.0].normalize();
        let pol = vector![0.2, -0.3, 0.4];
        for (inner, height, start, end) in [
            (0.0, 1.0, -90.0, 90.0),
            (0.99, 1.0, 170.0, 450.0),
            (0.2, 0.01, 30.0, 31.0),
            (0.2, 10.0, -60.0, 290.0),
        ] {
            let point = Point3::from(direction * (7.99 * NumFloat::hypot(1.0, height * 0.5)));
            let surface =
                local_cylinder_segment_B(point, pol, vector![inner, 1.0, height, start, end]);
            let volume = distant_field(
                point.coords,
                pol,
                inner,
                0.5 * height,
                start * PI / 180.0,
                (end - start) * PI / 180.0,
            );
            assert!(
                (surface - volume).norm() < 2e-8 * surface.norm(),
                "inner={inner}, height={height}: {surface:?} != {volume:?}"
            );
        }
    }

    #[test]
    fn small_segments_preserve_scale_invariance() {
        let pol = vector![0.4, -0.2, 0.7];
        let p = point![0.3, -0.7, 1.1];
        let reference = local_cylinder_segment_B(p, pol, vector![0.2, 1.0, 0.8, -70.0, 160.0]);
        assert!(reference.iter().all(|&v| v.is_finite()));

        for factor in [1e-9, 1e-12] {
            let scaled = local_cylinder_segment_B(
                Point3::from(p.coords * factor),
                pol,
                vector![0.2 * factor, factor, 0.8 * factor, -70.0, 160.0],
            );
            assert!(
                (scaled - reference).norm() < 1e-10 * reference.norm(),
                "factor={factor}, scaled={scaled:?}, reference={reference:?}"
            );
        }
    }

    #[test]
    fn axis_of_solid_sector_is_a_boundary() {
        let dims = vector![0.0, 1.0, 2.0, 30.0, 120.0];
        let pol = vector![0.3, -0.2, 0.5];
        // Radial cut faces meet at the z-axis. Following MagPyLib, the field
        // is zero on this geometric boundary, but finite outside the magnet.
        assert_eq!(
            local_cylinder_segment_B(point![0.0, 0.0, 0.0], pol, dims),
            Vector3::zeros(),
        );
        assert!(
            local_cylinder_segment_B(point![0.0, 0.0, 3.0], pol, dims)
                .iter()
                .all(|&v| v.is_finite())
        );
    }

    #[test]
    fn nearby_curved_wall() {
        let dims = vector![0.01, 0.02, 0.01, -30.0, 90.0];
        for delta in [-1e-6, 1e-6] {
            let r = 0.01 + delta;
            let theta = PI / 6.0;
            let point = point![r * NumFloat::cos(theta), r * NumFloat::sin(theta), 0.001];
            let field = local_cylinder_segment_B(point, vector![0.2, -0.3, 0.4], dims);
            assert!(field.iter().all(|&v| v.is_finite()));
        }
    }
}
