/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use magba::base::mesh::TriMesh;
use magba::prelude::*;
use magba::testing_util::*;
use nalgebra::{Point3, vector};
use reproducible::metrics;
use reproducible::prelude::*;
use std::error::Error;
use std::f64::consts::PI;

/// Extension trait to convert a `Source` into a closure compatible with `reproducible`'s batch API
trait SourceExt {
    fn as_batch_evaluator(self) -> Box<dyn Fn(&[Vec<f64>]) -> Vec<Vec<f64>> + Send + Sync>;
}

impl<T: magba::base::Float + std::str::FromStr> SourceExt for Box<dyn Source<T> + Send + Sync> {
    fn as_batch_evaluator(self) -> Box<dyn Fn(&[Vec<f64>]) -> Vec<Vec<f64>> + Send + Sync> {
        Box::new(move |inputs: &[Vec<f64>]| {
            let points: Vec<Point3<T>> = inputs
                .iter()
                .map(|i| {
                    Point3::new(
                        T::from(i[0]).unwrap(),
                        T::from(i[1]).unwrap(),
                        T::from(i[2]).unwrap(),
                    )
                })
                .collect();
            self.compute_B_batch(&points)
                .into_iter()
                .map(|v| {
                    vec![
                        v.x.to_f64().unwrap(),
                        v.y.to_f64().unwrap(),
                        v.z.to_f64().unwrap(),
                    ]
                })
                .collect()
        })
    }
}

fn create_magnet<T: magba::base::Float + std::str::FromStr>(
    name: &str,
) -> Box<dyn Source<T> + Send + Sync>
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

    let tetrahedron_vertices = [
        vector![f(-0.1), f(-0.1), f(-0.1)],
        vector![f(0.1), f(-0.1), f(-0.1)],
        vector![f(0.0), f(0.1), f(-0.1)],
        vector![f(0.0), f(0.0), f(0.1)],
    ];

    match name {
        "CylinderMagnet" => Box::new(CylinderMagnet::new(pos, rot, pol, f(0.1), f(0.2))),
        "CuboidMagnet" => Box::new(CuboidMagnet::new(pos, rot, pol, [f(0.1), f(0.2), f(0.3)])),
        "Dipole" => Box::new(Dipole::new(pos, rot, pol)),
        "SphereMagnet" => Box::new(SphereMagnet::new(pos, rot, pol, f(0.1))),
        "TetrahedronMagnet" => {
            Box::new(TetrahedronMagnet::new(pos, rot, pol, tetrahedron_vertices))
        }
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
                tetrahedron_vertices,
                vec![[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2]],
            );
            Box::new(MeshMagnet::new(pos, rot, pol, mesh))
        }
        _ => panic!("Unknown magnet: {}", name),
    }
}

fn add_magnet_accuracy<T: magba::base::Float + std::str::FromStr>(
    report: Report,
    name: &str,
    source: Box<dyn Source<T> + Send + Sync>,
    ref_file: &str,
) -> Result<Report, Box<dyn Error>> {
    let points_path = std::path::Path::new("./tests/test-data/points.csv");
    let ref_path = std::path::Path::new("./tests/test-data/").join(ref_file);

    if !points_path.exists() || !ref_path.exists() {
        return Err(format!("Test data not found for {}", name).into());
    }

    Ok(report.with_row(
        Row::new_batch(name, source.as_batch_evaluator())
            .with_split_csv_cases(points_path, ref_path)?,
    ))
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

    fn get_report() -> Report {
        Report::new()
            .with_float_precision(3)
            .with_metric(metrics::rel_err)
            .with_column(Column::accuracy("Median").with_stat(ColumnStat::Median))
            .with_column(Column::accuracy("Mean").with_stat(ColumnStat::Mean))
            .with_column(Column::accuracy("P95").with_stat(ColumnStat::P95))
            .with_column(Column::accuracy("Max").with_stat(ColumnStat::Max))
    }

    // f64 report
    {
        println!("## Relative Error: f64\n");
        let mut report = get_report();

        for &(name, ref_file) in &magnets {
            let magnet = create_magnet::<f64>(name);
            report = add_magnet_accuracy(report, name, magnet, ref_file)?;
        }
        println!("{}", report.render_markdown());
    }

    // f32 report
    {
        println!("\n## Relative Error: f32\n");
        let mut report = get_report();

        for &(name, ref_file) in &magnets {
            let magnet = create_magnet::<f32>(name);
            report = add_magnet_accuracy(report, name, magnet, ref_file)?;
        }
        println!("{}", report.render_markdown());
    }

    Ok(())
}
