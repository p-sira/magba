//! Independent checks of surface-charge integration and tetrahedral decomposition.
use magba::prelude::*;
use nalgebra::{Point3, UnitQuaternion, Vector3};

fn triangle_quadrature(
    point: Point3<f64>,
    vertices: [Vector3<f64>; 3],
    polarization: Vector3<f64>,
) -> Vector3<f64> {
    let n = 120;
    let a = vertices[1] - vertices[0];
    let b = vertices[2] - vertices[0];
    let weight =
        polarization.dot(&a.cross(&b)) / (2.0 * (n * n) as f64 * std::f64::consts::TAU * 2.0);
    let mut field = Vector3::zeros();
    for i in 0..n {
        for j in 0..n - i {
            for offset in [1.0 / 3.0, 2.0 / 3.0] {
                if offset > 0.5 && i + j == n - 1 {
                    continue;
                }
                let surface = vertices[0]
                    + a * ((i as f64 + offset) / n as f64)
                    + b * ((j as f64 + offset) / n as f64);
                let r = point.coords - surface;
                field += r * (weight / r.norm().powi(3));
            }
        }
    }
    field
}

#[test]
fn triangle_agrees_with_direct_surface_integration_at_multiple_scales() {
    let polarization = Vector3::new(0.3, -0.5, 1.2);
    let orientation = UnitQuaternion::from_scaled_axis(Vector3::new(0.4, -0.2, 0.7));
    for scale in [1.0, 1e-3, 1e-6] {
        let vertices = [
            Vector3::new(-0.7, -0.4, 0.1),
            Vector3::new(0.8, -0.3, 0.2),
            Vector3::new(0.1, 0.9, -0.1),
        ]
        .map(|v| v * scale);
        let position = Vector3::new(0.2, -0.4, 0.7) * scale;
        let magnet = TriangleMagnet::default()
            .with_vertices(vertices)
            .with_polarization(polarization)
            .with_position(position)
            .with_orientation(orientation);
        for point in [
            Point3::new(0.2, 0.1, 1.5),
            Point3::new(-0.3, 0.2, -1.2),
            Point3::new(1.3, -0.8, 0.1),
        ] {
            let point = point * scale;
            let expected = orientation * triangle_quadrature(point, vertices, polarization);
            let observer = Point3::from(orientation * point.coords + position);
            let actual = magnet.compute_B(observer);
            assert!(
                (actual - expected).norm() < expected.norm() * 3e-5,
                "scale={scale}, point={point:?}: {actual:?} != {expected:?}"
            );
            let mut reversed = vertices;
            reversed.swap(1, 2);
            assert!(
                (magnet.clone().with_vertices(reversed).compute_B(observer) + actual).norm()
                    < 1e-12
            );
        }
    }
}

#[test]
fn six_tetrahedra_reproduce_a_cuboid_inside_and_outside_at_multiple_scales() {
    let polarization = Vector3::new(0.3, -0.5, 1.2);
    let orientation = UnitQuaternion::from_scaled_axis(Vector3::new(0.4, -0.2, 0.7));
    for scale in [1.0, 1e-3, 1e-6] {
        let position = Vector3::new(0.2, -0.4, 0.7) * scale;
        let cube = CuboidMagnet::default()
            .with_dimensions([2.0 * scale; 3])
            .with_polarization(polarization)
            .with_position(position)
            .with_orientation(orientation);
        let tetrahedra: Vec<_> = [
            [0, 1, 2],
            [0, 2, 1],
            [1, 0, 2],
            [1, 2, 0],
            [2, 0, 1],
            [2, 1, 0],
        ]
        .into_iter()
        .map(|axes| {
            let start = Vector3::repeat(-scale);
            let mut a = start;
            a[axes[0]] = scale;
            let mut b = a;
            b[axes[1]] = scale;
            TetrahedronMagnet::default()
                .with_vertices([start, a, b, Vector3::repeat(scale)])
                .with_polarization(polarization)
                .with_position(position)
                .with_orientation(orientation)
        })
        .collect();
        for point in [
            Point3::new(0.13, -0.22, 0.37),
            Point3::new(2.1, -1.7, 0.8),
            Point3::new(10.0, 7.0, -8.0),
        ] {
            let observer = Point3::from(orientation * (point.coords * scale) + position);
            let expected = cube.compute_B(observer);
            let actual: Vector3<f64> = tetrahedra.iter().map(|t| t.compute_B(observer)).sum();
            assert!(
                (actual - expected).norm() < expected.norm() * 1e-9 + 1e-13,
                "scale={scale}: {actual:?} != {expected:?}"
            );
            for tetrahedron in &tetrahedra {
                let mut reversed = tetrahedron.vertices();
                reversed.swap(2, 3);
                assert!(
                    (tetrahedron
                        .clone()
                        .with_vertices(reversed)
                        .compute_B(observer)
                        - tetrahedron.compute_B(observer))
                    .norm()
                        < 1e-12
                );
            }
        }
    }
}
