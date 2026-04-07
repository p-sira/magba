/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use magba::base::mesh::TriMesh;
use magba::prelude::*;
use magba::testing_util::*;
use nalgebra::{Point3, Vector3, vector};
use reproducible::ReportBuilder;
use std::error::Error;
use std::f64::consts::PI;

fn create_magnet<T: magba::base::Float + std::str::FromStr>(name: &str) -> Box<dyn Source<T>>
where
    CylinderMagnet<T>: Source<T>,
    CuboidMagnet<T>: Source<T>,
    Dipole<T>: Source<T>,
    SphereMagnet<T>: Source<T>,
    TetrahedronMagnet<T>: Source<T>,
    TriangleMagnet<T>: Source<T>,
    MeshMagnet<T>: Source<T>,
{
    let f = |v: f64| T::from(v).unwrap();

    // Pose used in generate_tests! macro
    let pos = [f(0.1), f(0.2), f(0.3)];
    let rot = quat_from_rotvec(f(PI / 7.0), f(PI / 6.0), f(PI / 5.0));
    let pol = [f(1.0), f(2.0), f(3.0)];

    match name {
        "CylinderMagnet" => Box::new(CylinderMagnet::new(pos, rot, pol, f(0.1), f(0.2))),
        "CuboidMagnet" => Box::new(CuboidMagnet::new(pos, rot, pol, [f(0.1), f(0.2), f(0.3)])),
        "Dipole" => Box::new(Dipole::new(pos, rot, pol)),
        "SphereMagnet" => Box::new(SphereMagnet::new(pos, rot, pol, f(0.1))),
        "TetrahedronMagnet" => Box::new(TetrahedronMagnet::new(
            pos,
            rot,
            pol,
            [
                vector![f(-0.1), f(-0.1), f(-0.1)],
                vector![f(0.1), f(-0.1), f(-0.1)],
                vector![f(0.0), f(0.1), f(-0.1)],
                vector![f(0.0), f(0.0), f(0.1)],
            ],
        )),
        "TriangleMagnet" => Box::new(TriangleMagnet::new(
            pos,
            rot,
            pol,
            [
                vector![f(-0.1), f(-0.1), f(-0.1)],
                vector![f(0.1), f(-0.1), f(0.1)],
                vector![f(0.0), f(0.2), f(0.0)],
            ],
        )),
        "MeshMagnet" => {
            let mesh = TriMesh::<T>::new_unchecked(
                vec![
                    Vector3::new(f(-0.1), f(-0.1), f(-0.1)),
                    Vector3::new(f(0.1), f(-0.1), f(-0.1)),
                    Vector3::new(f(0.0), f(0.1), f(-0.1)),
                    Vector3::new(f(0.0), f(0.0), f(0.1)),
                ],
                vec![[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2]],
            );
            Box::new(MeshMagnet::new(pos, rot, pol, mesh))
        }
        _ => panic!("Unknown magnet: {}", name),
    }
}

fn add_magnet_accuracy<T: magba::base::Float + std::str::FromStr, S: Source<T> + ?Sized>(
    builder: ReportBuilder,
    name: &str,
    source: &S,
    ref_file: &str,
) -> Result<ReportBuilder, Box<dyn Error>> {
    let points_path = std::path::Path::new("./tests/test-data/points.csv");
    let ref_path = std::path::Path::new("./tests/test-data/").join(ref_file);

    if !points_path.exists() || !ref_path.exists() {
        return Err(format!("Test data not found for {}", name).into());
    }

    let expected = matrix_to_vector_vec(&load_matrix_from_csv::<f64>(&ref_path));
    let points_raw = load_matrix_from_csv::<f64>(&points_path);

    let points_t: Vec<Point3<T>> = points_raw
        .row_iter()
        .map(|row| {
            Point3::new(
                T::from(row[0]).unwrap(),
                T::from(row[1]).unwrap(),
                T::from(row[2]).unwrap(),
            )
        })
        .collect();

    let b_fields_t = source.compute_B_batch(&points_t);

    let errors: Vec<f64> = b_fields_t
        .iter()
        .zip(expected.iter())
        .map(|(a, b)| {
            let actual_f64 = Vector3::new(
                a.x.to_f64().unwrap(),
                a.y.to_f64().unwrap(),
                a.z.to_f64().unwrap(),
            );

            let diff = (actual_f64 - b).norm();
            let norm_b = b.norm();

            if norm_b > 1e-12 {
                diff / norm_b
            } else {
                diff
            }
        })
        .collect();

    let stats = reproducible::Stats::from_samples(&errors);
    Ok(builder.add_accuracy_row(reproducible::AccuracyReportRow {
        name: name.to_owned(),
        stats,
    }))
}

fn main() -> Result<(), Box<dyn Error>> {
    println!("# Magba Accuracy Report\n");

    let magnets = [
        ("CylinderMagnet", "cylinder.csv"),
        ("CuboidMagnet", "cuboid.csv"),
        ("Dipole", "dipole.csv"),
        ("SphereMagnet", "sphere.csv"),
        ("TetrahedronMagnet", "tetrahedron.csv"),
        ("TriangleMagnet", "triangle.csv"),
        ("MeshMagnet", "triangularmesh.csv"),
    ];

    // f64 report
    {
        println!("## Precision: f64 (Relative Error)\n");
        let mut builder = ReportBuilder::new();
        builder.render_config_mut().error_unit = "".to_owned();
        builder.render_config_mut().float_precision = 3;

        for &(name, ref_file) in &magnets {
            let magnet = create_magnet::<f64>(name);
            builder = add_magnet_accuracy(builder, name, &*magnet, ref_file)?;
        }
        println!("{}", builder.render_accuracy_markdown());
    }

    // f32 report
    {
        println!("\n## Precision: f32 (Relative Error)\n");
        let mut builder = ReportBuilder::new();
        builder.render_config_mut().error_unit = "".to_owned();
        builder.render_config_mut().float_precision = 3;

        for &(name, ref_file) in &magnets {
            let magnet = create_magnet::<f32>(name);
            builder = add_magnet_accuracy(builder, name, &*magnet, ref_file)?;
        }
        println!("{}", builder.render_accuracy_markdown());
    }

    Ok(())
}
