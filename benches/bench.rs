use criterion::{Criterion, black_box, criterion_group, criterion_main};
use magba::base::Source;
use magba::currents::*;
use magba::magnets::*;
use nalgebra::{Point3, UnitQuaternion, point, vector};
use std::path::Path;

fn get_points_f64() -> Vec<Point3<f64>> {
    let path = Path::new("tests/test-data/points.csv");
    let file = std::fs::File::open(path).unwrap();
    let mut reader = csv::ReaderBuilder::new()
        .has_headers(false)
        .from_reader(file);
    let mut points = Vec::new();
    for result in reader.records() {
        let record = result.unwrap();
        let x: f64 = record[0].parse().unwrap();
        let y: f64 = record[1].parse().unwrap();
        let z: f64 = record[2].parse().unwrap();
        points.push(point![x, y, z]);
    }
    points
}

fn get_points_f32() -> Vec<Point3<f32>> {
    let path = Path::new("tests/test-data/points.csv");
    let file = std::fs::File::open(path).unwrap();
    let mut reader = csv::ReaderBuilder::new()
        .has_headers(false)
        .from_reader(file);
    let mut points = Vec::new();
    for result in reader.records() {
        let record = result.unwrap();
        let x: f32 = record[0].parse().unwrap();
        let y: f32 = record[1].parse().unwrap();
        let z: f32 = record[2].parse().unwrap();
        points.push(point![x, y, z]);
    }
    points
}

fn bench_field_functions(c: &mut Criterion) {
    // f64 bench
    {
        let points = get_points_f64();
        let pos = Point3::<f64>::origin();
        let ori = UnitQuaternion::<f64>::identity();
        let pol = vector![1.0, 0.0, 0.0];
        let dim = vector![0.01, 0.01, 0.01];
        let height = 0.02;
        let diameter = 0.02;
        let moment = vector![0.0, 0.0, 1e-3];
        let current = 1.0;

        let mut group = c.benchmark_group("fields/f64");
        group.throughput(criterion::Throughput::Elements(points.len() as u64));

        let circular = CircularCurrent::new(pos, ori, diameter, current);
        group.bench_function("CircularCurrent", |b| {
            b.iter(|| black_box(circular.compute_B_batch(&points)))
        });

        let cuboid = CuboidMagnet::new(pos, ori, pol, dim);
        group.bench_function("CuboidMagnet", |b| {
            b.iter(|| black_box(cuboid.compute_B_batch(&points)))
        });

        let cylinder = CylinderMagnet::new(pos, ori, pol, diameter, height);
        group.bench_function("CylinderMagnet", |b| {
            b.iter(|| black_box(cylinder.compute_B_batch(&points)))
        });

        let dipole = Dipole::new(pos, ori, moment);
        group.bench_function("Dipole", |b| {
            b.iter(|| black_box(dipole.compute_B_batch(&points)))
        });

        let sphere = SphereMagnet::new(pos, ori, pol, diameter);
        group.bench_function("SphereMagnet", |b| {
            b.iter(|| black_box(sphere.compute_B_batch(&points)))
        });

        let vertices_triangle = [
            vector![-0.1, -0.1, -0.1],
            vector![0.1, -0.1, 0.1],
            vector![0.0, 0.2, 0.0],
        ];
        let triangle = TriangleMagnet::new(pos, ori, pol, vertices_triangle);
        group.bench_function("TriangleMagnet", |b| {
            b.iter(|| black_box(triangle.compute_B_batch(&points)))
        });

        let vertices_tetra = [
            vector![-0.1, -0.1, -0.1],
            vector![0.1, -0.1, -0.1],
            vector![0.0, 0.1, -0.1],
            vector![0.0, 0.0, 0.1],
        ];
        let tetra = TetrahedronMagnet::new(pos, ori, pol, vertices_tetra);
        group.bench_function("TetrahedronMagnet", |b| {
            b.iter(|| black_box(tetra.compute_B_batch(&points)))
        });

        use magba::base::mesh::TriMesh;
        let vertices_mesh = vec![
            vector![-0.1, -0.1, -0.1],
            vector![0.1, -0.1, -0.1],
            vector![0.0, 0.1, -0.1],
            vector![0.0, 0.0, 0.1],
        ];
        let faces_mesh = vec![[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2]];
        let mesh = TriMesh::new(vertices_mesh, faces_mesh).unwrap();
        let mesh_magnet = MeshMagnet::new(pos, ori, pol, mesh);
        group.bench_function("MeshMagnet", |b| {
            b.iter(|| black_box(mesh_magnet.compute_B_batch(&points)))
        });

        group.finish();
    }

    // f32 bench
    {
        let points = get_points_f32();
        let pos = Point3::<f32>::origin();
        let ori = UnitQuaternion::<f32>::identity();
        let pol = vector![1.0f32, 0.0, 0.0];
        let dim = vector![0.01f32, 0.01, 0.01];
        let height = 0.02f32;
        let diameter = 0.02f32;
        let moment = vector![0.0f32, 0.0, 1e-3];
        let current = 1.0f32;

        let mut group = c.benchmark_group("fields/f32");
        group.throughput(criterion::Throughput::Elements(points.len() as u64));

        let circular = CircularCurrent::new(pos, ori, diameter, current);
        group.bench_function("CircularCurrent", |b| {
            b.iter(|| black_box(circular.compute_B_batch(&points)))
        });

        let cuboid = CuboidMagnet::new(pos, ori, pol, dim);
        group.bench_function("CuboidMagnet", |b| {
            b.iter(|| black_box(cuboid.compute_B_batch(&points)))
        });

        let cylinder = CylinderMagnet::new(pos, ori, pol, diameter, height);
        group.bench_function("CylinderMagnet", |b| {
            b.iter(|| black_box(cylinder.compute_B_batch(&points)))
        });

        let dipole = Dipole::new(pos, ori, moment);
        group.bench_function("Dipole", |b| {
            b.iter(|| black_box(dipole.compute_B_batch(&points)))
        });

        let sphere = SphereMagnet::new(pos, ori, pol, diameter);
        group.bench_function("SphereMagnet", |b| {
            b.iter(|| black_box(sphere.compute_B_batch(&points)))
        });

        let vertices_triangle = [
            vector![-0.1f32, -0.1, -0.1],
            vector![0.1f32, -0.1, 0.1],
            vector![0.0f32, 0.2, 0.0],
        ];
        let triangle = TriangleMagnet::new(pos, ori, pol, vertices_triangle);
        group.bench_function("TriangleMagnet", |b| {
            b.iter(|| black_box(triangle.compute_B_batch(&points)))
        });

        let vertices_tetra = [
            vector![-0.1f32, -0.1, -0.1],
            vector![0.1f32, -0.1, -0.1],
            vector![0.0f32, 0.1, -0.1],
            vector![0.0f32, 0.0, 0.1],
        ];
        let tetra = TetrahedronMagnet::new(pos, ori, pol, vertices_tetra);
        group.bench_function("TetrahedronMagnet", |b| {
            b.iter(|| black_box(tetra.compute_B_batch(&points)))
        });

        use magba::base::mesh::TriMesh;
        let vertices_mesh = vec![
            vector![-0.1f32, -0.1, -0.1],
            vector![0.1f32, -0.1, -0.1],
            vector![0.0f32, 0.1, -0.1],
            vector![0.0f32, 0.0, 0.1],
        ];
        let faces_mesh = vec![[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2]];
        let mesh = TriMesh::new(vertices_mesh, faces_mesh).unwrap();
        let mesh_magnet = MeshMagnet::new(pos, ori, pol, mesh);
        group.bench_function("MeshMagnet", |b| {
            b.iter(|| black_box(mesh_magnet.compute_B_batch(&points)))
        });

        group.finish();
    }
}

criterion_group!(benches, bench_field_functions);
criterion_main!(benches);
