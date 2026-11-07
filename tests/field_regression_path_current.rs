#![cfg(feature = "std")]

//! Independent finite-wire Biot–Savart reference, including cancellation
//! resistant endpoint factors for observers beyond either segment endpoint.
use magba::fields::{path_current_B, path_current_B_batch};
use nalgebra::{Point3, UnitQuaternion, Vector3};

const MU0_4PI: f64 = 1.25663706127e-6 / (4.0 * std::f64::consts::PI);

fn reference(p: Point3<f64>, current: f64) -> Vector3<f64> {
    let r = p.x.hypot(p.y);
    let z = p.z;
    let angular = if z < 0.0 || z > 1.0 {
        // Rationalized difference of nearly equal endpoint cosines.
        let (near, far) = if z < 0.0 { (-z, 1.0 - z) } else { (z - 1.0, z) };
        let a = near.hypot(r);
        let b = far.hypot(r);
        r * r * (far * far - near * near) / (a * b * (far * a + near * b))
    } else {
        z / z.hypot(r) + (1.0 - z) / (1.0 - z).hypot(r)
    };
    let b = MU0_4PI * current * angular / (r * r);
    Vector3::new(-p.y * b, p.x * b, 0.0)
}

fn close(actual: Vector3<f64>, expected: Vector3<f64>, rel: f64, abs: f64) {
    let error = (actual - expected).norm();
    assert!(
        actual.iter().all(|x| x.is_finite()) && error <= abs + rel * expected.norm(),
        "actual={actual:?}, expected={expected:?}, error={error:e}"
    );
}

#[test]
fn signed_endpoint_factors_reversal_and_rotated_f64_batch() {
    let vertices = [Vector3::zeros(), Vector3::z()];
    let reverse = [vertices[1], vertices[0]];
    let position = Point3::new(0.3_f64, -0.2, 0.4);
    let orientation = UnitQuaternion::from_scaled_axis(Vector3::new(0.2, -0.4, 0.3));
    let local = [-20.0, -0.5, 0.5, 1.5, 21.0].map(|z| Point3::new(0.1, -0.07, z));
    let points = local.map(|p| Point3::from(position.coords + orientation * p.coords));
    let mut batch = [Vector3::zeros(); 5];
    path_current_B_batch(&points, position, orientation, 2.5, &vertices, &mut batch);

    for ((point, local), batched) in points.into_iter().zip(local).zip(batch) {
        let expected = orientation * reference(local, 2.5);
        close(batched, expected, 3e-8, 2e-18);
        for (wire, sign) in [(&vertices[..], 1.0), (&reverse[..], -1.0)] {
            close(
                path_current_B(point, position, orientation, 2.5, wire),
                expected * sign,
                3e-8,
                2e-18,
            );
        }
    }
}

#[test]
fn f32_endpoints_reversal_and_batch() {
    let vertices = [Vector3::zeros(), Vector3::z()];
    let reverse = [vertices[1], vertices[0]];
    let points = [-5.0_f32, -0.5, 0.5, 1.5, 6.0].map(|z| Point3::new(0.1, -0.07, z));
    let position = Point3::origin();
    let rotation = UnitQuaternion::identity();
    let mut batch = [Vector3::zeros(); 5];
    path_current_B_batch(&points, position, rotation, 2.5, &vertices, &mut batch);

    for (point, batched) in points.into_iter().zip(batch) {
        let expected = reference(point.cast::<f64>(), 2.5);
        close(batched.cast::<f64>(), expected, 8e-3, 2e-13);
        for (wire, sign) in [(&vertices[..], 1.0), (&reverse[..], -1.0)] {
            close(
                path_current_B(point, position, rotation, 2.5, wire).cast::<f64>(),
                expected * sign,
                8e-3,
                2e-13,
            );
        }
    }
}

#[test]
fn near_wire_and_cutoff_for_both_precisions() {
    let vertices = [Vector3::zeros(), Vector3::z()];
    let field64 = |p, v: &[Vector3<f64>]| {
        path_current_B(p, Point3::origin(), UnitQuaternion::identity(), 1.0, v)
    };
    let near = Point3::new(1e-12, 0.0, 0.5);
    close(field64(near, &vertices), reference(near, 1.0), 1e-11, 1e-9);
    assert_eq!(
        field64(Point3::new(1e-16, 0.0, 0.5), &vertices),
        Vector3::zeros()
    );

    let vertices32: [Vector3<f32>; 2] = [Vector3::zeros(), Vector3::z()];
    let position32 = Point3::origin();
    let rotation32 = UnitQuaternion::identity();
    let field32 = |p| path_current_B(p, position32, rotation32, 1.0, &vertices32);
    let near32 = Point3::new(1e-6_f32, 0.0, 0.5);
    close(
        field32(near32).cast::<f64>(),
        reference(near32.cast(), 1.0),
        1e-4,
        1e-9,
    );
    assert_eq!(field32(Point3::new(1e-16, 0.0, 0.5)), Vector3::zeros());
}
