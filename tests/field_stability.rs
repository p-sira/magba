//! Independent surface-charge and Biot–Savart quadrature at former grid failures.
//! Provenance and regeneration: data/field_stability_generator.py.
#![cfg(all(feature = "std", feature = "mesh"))]

use magba::{base::Float, base::mesh::TriMesh, prelude::*};
use nalgebra::{Point3, UnitQuaternion, Vector3};

fn source<T: Float + std::iter::Sum>(name: &str) -> Box<dyn Source<T>> {
    let f = |v: f64| T::from(v).unwrap();
    let position = [f(0.1), f(0.2), f(0.3)];
    let rotation = UnitQuaternion::from_scaled_axis(Vector3::new(
        f(core::f64::consts::PI / 7.0),
        f(core::f64::consts::PI / 6.0),
        f(core::f64::consts::PI / 5.0),
    ));
    let polarization = [f(1.0), f(2.0), f(3.0)];
    let vertices = [
        Vector3::new(f(-0.1), f(-0.1), f(-0.1)),
        Vector3::new(f(0.1), f(-0.1), f(-0.1)),
        Vector3::new(f(0.0), f(0.1), f(-0.1)),
        Vector3::new(f(0.0), f(0.0), f(0.1)),
    ];
    let mesh = || TriMesh::new(vertices, [[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2]]).unwrap();
    match name {
        "cylinder" => Box::new(CylinderMagnet::new(
            position,
            rotation,
            polarization,
            f(0.1),
            f(0.2),
        )),
        "triangle_current" => Box::new(TriangleCurrent::new(
            position,
            rotation,
            polarization,
            [vertices[0], vertices[1], vertices[2]],
        )),
        "sheet_current" => Box::new(SheetCurrent::new(
            position,
            rotation,
            vec![Vector3::from(polarization); 4],
            mesh(),
        )),
        "tetrahedron" => Box::new(TetrahedronMagnet::new(
            position,
            rotation,
            polarization,
            vertices,
        )),
        "mesh" => Box::new(MeshMagnet::new(position, rotation, polarization, mesh())),
        "triangle_magnet" => Box::new(TriangleMagnet::new(
            position,
            rotation,
            polarization,
            [
                vertices[0],
                Vector3::new(f(0.1), f(-0.1), f(0.1)),
                Vector3::new(f(0.0), f(0.2), f(0.0)),
            ],
        )),
        "path_current" => Box::new(PathCurrent::new(
            position,
            rotation,
            f(1.0),
            vertices.to_vec(),
        )),
        _ => panic!("unknown reference source {name}"),
    }
}

fn check<T: Float + std::iter::Sum>(
    name: &str,
    observer: Vector3<f64>,
    expected: Vector3<f64>,
    tolerance: f64,
) {
    let source = source::<T>(name);
    let point = Point3::from(observer.map(|v| T::from(v).unwrap()));
    let points = vec![point; 512];
    for got in std::iter::once(source.compute_B(point)).chain(source.compute_B_batch(&points)) {
        let got = got.map(|v| v.to_f64().unwrap());
        let error = (got - expected).norm() / expected.norm();
        assert!(
            got.iter().all(|v| v.is_finite()) && error < tolerance,
            "{name} {} at {observer:?}: relative error {error:e}, got {got:?}, expected {expected:?}",
            std::any::type_name::<T>()
        );
    }
}

#[test]
fn grid_stress_cases_match_independent_integrals() {
    let mut reader =
        csv::Reader::from_reader(include_bytes!("data/field_stability_reference.csv").as_slice());
    for row in reader.records() {
        let row = row.unwrap();
        let values: Vec<f64> = row.iter().skip(1).map(|s| s.parse().unwrap()).collect();
        let observer = Vector3::from_column_slice(&values[..3]);
        let expected = Vector3::from_column_slice(&values[3..]);
        let name = &row[0];
        let names: &[&str] = if name == "tetrahedron" {
            &["tetrahedron", "mesh"]
        } else {
            &[name]
        };
        for &name in names {
            check::<f64>(
                name,
                observer,
                expected,
                if name == "cylinder" { 1e-10 } else { 2e-12 },
            );
            check::<f32>(
                name,
                observer,
                expected,
                if name == "tetrahedron" || name == "mesh" {
                    5e-5
                } else {
                    2e-5
                },
            );
        }
    }
}
