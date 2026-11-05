//! Independent regression checks for batch surface-charge precomputation.
//!
//! Provenance: expected fields are evaluated here from the magnetostatic
//! surface-charge integral, using tensor-product Gauss-Legendre quadrature
//! mapped onto each triangle by the Duffy transform. This oracle uses neither
//! Magba's solid-angle/edge-logarithm formulas nor its batch/scalar evaluators.
//! It is distinct from the lattice-based quadrature in polyhedral_fields.rs.
//! The chosen observation points are separated from all faces, where the
//! smooth integrand converges rapidly; surface/edge limits are not oracle data.
//!
//! The tetrahedron/mesh reference adds the homogeneous polarization for
//! interior points (B = mu0*H + J). A face's outward direction is determined
//! geometrically from the opposite tetrahedron vertex, independently of
//! Magba's face indexing. Reversed triangle winding reverses the surface
//! charge; reversed tetrahedron vertex order must not change a closed body.

#![cfg(feature = "mesh")]

use magba::{
    base::mesh::TriMesh,
    fields::{
        mesh_B, mesh_B_batch, tetrahedron_B, tetrahedron_B_batch, triangle_B, triangle_B_batch,
    },
};
use nalgebra::{Point3, UnitQuaternion, Vector3, point, vector};

const QUAD_ORDER: usize = 96;
const ROTATION: [f64; 3] = [0.43, -0.71, 0.29];
const POLARIZATION: [f64; 3] = [0.74, -0.39, 1.13];
const SCALES: [f64; 3] = [1e-3, 1.0, 31.0];

fn legendre_and_derivative(order: usize, x: f64) -> (f64, f64) {
    let mut previous = 1.0;
    let mut current = x;
    for k in 2..=order {
        let next = ((2 * k - 1) as f64 * x * current - (k - 1) as f64 * previous) / k as f64;
        previous = current;
        current = next;
    }
    let derivative = order as f64 * (x * current - previous) / (x * x - 1.0);
    (current, derivative)
}

fn gauss_legendre_nodes(order: usize) -> Vec<(f64, f64)> {
    (0..order)
        .map(|i| {
            let mut root = (std::f64::consts::PI * (i as f64 + 0.75) / (order as f64 + 0.5)).cos();
            for _ in 0..20 {
                let (value, derivative) = legendre_and_derivative(order, root);
                let correction = value / derivative;
                root -= correction;
                if correction.abs() < 2e-15 {
                    break;
                }
            }
            let (_, derivative) = legendre_and_derivative(order, root);
            // Map [-1, 1] to [0, 1], including the factor 1/2 in the weight.
            let weight = 1.0 / ((1.0 - root * root) * derivative * derivative);
            ((root + 1.0) * 0.5, weight)
        })
        .collect()
}

/// Independent Coulomb integral of sigma = polarization dot outward normal.
/// The Duffy map is r(u,v) = a + u*(b-a) + (1-u)*v*(c-a).
fn charged_face_quadrature(
    observer: Point3<f64>,
    vertices: [Vector3<f64>; 3],
    polarization: Vector3<f64>,
    nodes: &[(f64, f64)],
) -> Vector3<f64> {
    let a = vertices[1] - vertices[0];
    let b = vertices[2] - vertices[0];
    let prefactor = polarization.dot(&a.cross(&b)) / (4.0 * std::f64::consts::PI);
    if prefactor == 0.0 {
        return Vector3::zeros();
    }
    let mut result = Vector3::zeros();
    for &(u, wu) in nodes {
        for &(v, wv) in nodes {
            let surface = vertices[0] + a * u + b * ((1.0 - u) * v);
            let displacement = observer.coords - surface;
            let radius_sq = displacement.norm_squared();
            let weight = wu * wv * (1.0 - u) * prefactor / (radius_sq * radius_sq.sqrt());
            result += displacement * weight;
        }
    }
    result
}

fn tetra_vertices(scale: f64) -> [Vector3<f64>; 4] {
    [
        vector![-0.34, -0.27, -0.19],
        vector![0.92, -0.11, 0.06],
        vector![-0.04, 1.27, 0.21],
        vector![0.15, 0.22, 0.83],
    ]
    .map(|v| v * scale)
}

fn triangle_vertices(scale: f64) -> [Vector3<f64>; 3] {
    [
        vector![-0.45, -0.27, 0.11],
        vector![0.67, -0.18, 0.24],
        vector![0.09, 0.83, -0.37],
    ]
    .map(|v| v * scale)
}

/// Construct the outward tetrahedron faces using the opposite vertex.
fn outward_faces(vertices: &[Vector3<f64>; 4]) -> [[usize; 3]; 4] {
    let mut faces = [[0, 1, 2], [0, 1, 3], [0, 2, 3], [1, 2, 3]];
    for face in &mut faces {
        let omitted = (0..4)
            .find(|index| !face.contains(index))
            .expect("each tetrahedron face omits one vertex");
        let [a, b, c] = *face;
        let normal = (vertices[b] - vertices[a]).cross(&(vertices[c] - vertices[a]));
        if normal.dot(&(vertices[omitted] - vertices[a])) > 0.0 {
            face.swap(1, 2);
        }
    }
    faces
}

fn closed_tetra_reference(
    observer: Point3<f64>,
    vertices: [Vector3<f64>; 4],
    polarization: Vector3<f64>,
    inside: bool,
    nodes: &[(f64, f64)],
) -> Vector3<f64> {
    let mut field = Vector3::zeros();
    for [a, b, c] in outward_faces(&vertices) {
        field += charged_face_quadrature(
            observer,
            [vertices[a], vertices[b], vertices[c]],
            polarization,
            nodes,
        );
    }
    if inside {
        field += polarization;
    }
    field
}

fn local_observers() -> [(Point3<f64>, bool); 6] {
    let t = tetra_vertices(1.0);
    [
        // Two convex combinations provide unambiguous interior points.
        (Point3::from((t[0] + t[1] + t[2] + t[3]) / 4.0), true),
        (
            Point3::from(t[0] * 0.1 + t[1] * 0.2 + t[2] * 0.3 + t[3] * 0.4),
            true,
        ),
        (point![1.7, 0.25, 0.44], false),
        (point![-1.3, 0.81, -0.67], false),
        (point![0.35, -0.74, -0.45], false),
        (point![0.84, 1.57, 1.45], false),
    ]
}

fn pose(scale: f64) -> (Point3<f64>, UnitQuaternion<f64>) {
    (
        point![0.21 * scale, -0.32 * scale, 0.53 * scale],
        UnitQuaternion::from_scaled_axis(vector![ROTATION[0], ROTATION[1], ROTATION[2]]),
    )
}

fn global_point(
    local: Point3<f64>,
    position: Point3<f64>,
    orientation: UnitQuaternion<f64>,
) -> Point3<f64> {
    position + orientation * local.coords
}

fn batch_observers(scale: f64, count: usize) -> Vec<Point3<f64>> {
    (0..count)
        .map(|i| {
            let x = (i * 37 % 397) as f64 / 397.0;
            let y = (i * 151 % 401) as f64 / 401.0;
            let z = (i * 271 % 409) as f64 / 409.0;
            point![
                (2.8 * x - 1.0) * scale,
                (2.7 * y - 0.9) * scale,
                (2.6 * z - 0.8) * scale
            ]
        })
        .collect()
}

fn assert_field_f64(actual: Vector3<f64>, expected: Vector3<f64>, context: &str) {
    let tolerance = 2e-9 + 2e-8 * expected.norm();
    assert!(
        (actual - expected).norm() <= tolerance,
        "{context}: actual={actual:?}, quadrature={expected:?}, delta={:?}, tolerance={tolerance}",
        actual - expected,
    );
}

fn assert_field_f32(actual: Vector3<f32>, expected: Vector3<f64>, context: &str) {
    let actual64 = actual.cast::<f64>();
    let tolerance = 2e-4 + 6e-4 * expected.norm();
    assert!(
        (actual64 - expected).norm() <= tolerance,
        "{context}: actual={actual:?}, quadrature={expected:?}, delta={:?}, tolerance={tolerance}",
        actual64 - expected,
    );
}

#[test]
fn rotated_anisotropic_triangle_batch_agrees_with_independent_quadrature() {
    let nodes = gauss_legendre_nodes(QUAD_ORDER);
    let polarization = vector![POLARIZATION[0], POLARIZATION[1], POLARIZATION[2]];
    let probes = [
        point![0.19, 0.07, 0.91],
        point![-0.61, 0.32, -0.84],
        point![1.27, -0.72, 0.28],
        point![0.31, 1.39, 0.58],
        point![-0.15, -0.91, 0.71],
    ];

    for scale in SCALES {
        let (position, orientation) = pose(scale);
        let vertices = triangle_vertices(scale);
        for reversed in [false, true] {
            let mut vertices = vertices;
            if reversed {
                vertices.swap(1, 2);
            }
            let local_points: Vec<_> = probes
                .iter()
                .map(|p| Point3::from(p.coords * scale))
                .chain(batch_observers(scale, 390))
                .collect();
            let global_points: Vec<_> = local_points
                .iter()
                .map(|&p| global_point(p, position, orientation))
                .collect();
            let mut result = vec![Vector3::zeros(); global_points.len()];
            triangle_B_batch(
                &global_points,
                position,
                orientation,
                polarization,
                vertices,
                &mut result,
            );

            for i in 0..probes.len() {
                let expected_local =
                    charged_face_quadrature(local_points[i], vertices, polarization, &nodes);
                let expected = orientation * expected_local;
                assert_field_f64(result[i], expected, "rotated f64 triangle");
            }
            for (&point, &actual) in global_points.iter().zip(&result) {
                let scalar = triangle_B(point, position, orientation, polarization, vertices);
                assert!(
                    (actual - scalar).norm() < 4e-13,
                    "triangle batch differs from scalar at scale={scale}, reversed={reversed}",
                );
            }

            let points32: Vec<_> = global_points.iter().map(|p| p.cast::<f32>()).collect();
            let mut result32 = vec![Vector3::zeros(); points32.len()];
            let position32 = position.cast::<f32>();
            let orientation32 = UnitQuaternion::from_scaled_axis(vector![
                ROTATION[0] as f32,
                ROTATION[1] as f32,
                ROTATION[2] as f32
            ]);
            let polarization32 = polarization.cast::<f32>();
            let vertices32 = vertices.map(|v| v.cast::<f32>());
            triangle_B_batch(
                &points32,
                position32,
                orientation32,
                polarization32,
                vertices32,
                &mut result32,
            );
            for i in 0..probes.len() {
                let expected = orientation
                    * charged_face_quadrature(local_points[i], vertices, polarization, &nodes);
                assert_field_f32(result32[i], expected, "rotated f32 triangle");
            }
            for (&point, &actual) in points32.iter().zip(&result32) {
                let scalar =
                    triangle_B(point, position32, orientation32, polarization32, vertices32);
                assert!(
                    (actual - scalar).norm() < 1e-5,
                    "f32 triangle batch differs from scalar at scale={scale}, reversed={reversed}",
                );
            }
        }
    }
}

#[test]
fn rotated_tetrahedron_and_mesh_batches_match_independent_surface_integral() {
    let nodes = gauss_legendre_nodes(QUAD_ORDER);
    let polarization = vector![POLARIZATION[0], POLARIZATION[1], POLARIZATION[2]];
    for scale in SCALES {
        let vertices = tetra_vertices(scale);
        let faces = outward_faces(&vertices);
        // The final face is zero-area; its physical surface charge is zero.
        let mesh = TriMesh::new_unchecked(vertices, faces.into_iter().chain([[0, 1, 1]]));
        let (position, orientation) = pose(scale);
        let mut probes: Vec<_> = local_observers()
            .into_iter()
            .map(|(p, inside)| (Point3::from(p.coords * scale), inside))
            .collect();
        probes.extend(batch_observers(scale, 1610).into_iter().map(|p| (p, false)));
        let points: Vec<_> = probes
            .iter()
            .map(|&(p, _)| global_point(p, position, orientation))
            .collect();
        let mut tetra_out = vec![Vector3::zeros(); points.len()];
        let mut mesh_out = vec![Vector3::zeros(); points.len()];
        tetrahedron_B_batch(
            &points,
            position,
            orientation,
            polarization,
            vertices,
            &mut tetra_out,
        );
        mesh_B_batch(
            &points,
            position,
            orientation,
            polarization,
            &mesh,
            &mut mesh_out,
        );
        for i in 0..local_observers().len() {
            let expected_local =
                closed_tetra_reference(probes[i].0, vertices, polarization, probes[i].1, &nodes);
            let expected = orientation * expected_local;
            assert_field_f64(tetra_out[i], expected, "f64 tetrahedron surface integral");
            assert_field_f64(mesh_out[i], expected, "f64 mesh surface integral");
        }
        for (index, &point) in points.iter().enumerate() {
            let expected_tetra =
                tetrahedron_B(point, position, orientation, polarization, vertices);
            let expected_mesh = mesh_B(point, position, orientation, polarization, &mesh);
            assert!(
                (tetra_out[index] - expected_tetra).norm() < 4e-13,
                "f64 tetra batch/scalar mismatch at scale={scale} index={index}",
            );
            assert!(
                (mesh_out[index] - expected_mesh).norm() < 4e-13,
                "f64 mesh batch/scalar mismatch at scale={scale} index={index}",
            );
        }

        let points32: Vec<_> = points.iter().map(|p| p.cast::<f32>()).collect();
        let vertices32 = vertices.map(|v| v.cast::<f32>());
        let mesh32 = TriMesh::new_unchecked(
            vertices32,
            outward_faces(&vertices).into_iter().chain([[0, 1, 1]]),
        );
        let position32 = position.cast::<f32>();
        let orientation32 = UnitQuaternion::from_scaled_axis(vector![
            ROTATION[0] as f32,
            ROTATION[1] as f32,
            ROTATION[2] as f32
        ]);
        let pol32 = polarization.cast::<f32>();
        let mut tetra32 = vec![Vector3::zeros(); points32.len()];
        let mut mesh32_out = vec![Vector3::zeros(); points32.len()];
        tetrahedron_B_batch(
            &points32,
            position32,
            orientation32,
            pol32,
            vertices32,
            &mut tetra32,
        );
        mesh_B_batch(
            &points32,
            position32,
            orientation32,
            pol32,
            &mesh32,
            &mut mesh32_out,
        );
        for i in 0..local_observers().len() {
            let expected = orientation
                * closed_tetra_reference(probes[i].0, vertices, polarization, probes[i].1, &nodes);
            assert_field_f32(tetra32[i], expected, "f32 tetrahedron surface integral");
            assert_field_f32(mesh32_out[i], expected, "f32 mesh surface integral");
        }
        for (index, &point) in points32.iter().enumerate() {
            let scalar_tetra = tetrahedron_B(point, position32, orientation32, pol32, vertices32);
            let scalar_mesh = mesh_B(point, position32, orientation32, pol32, &mesh32);
            assert!(
                (tetra32[index] - scalar_tetra).norm() < 3e-5,
                "f32 tetra batch/scalar mismatch at scale={scale} index={index}",
            );
            assert!(
                (mesh32_out[index] - scalar_mesh).norm() < 3e-5,
                "f32 mesh batch/scalar mismatch at scale={scale} index={index}",
            );
        }

        // Reversing a tetrahedron's vertex orientation must leave its B field
        // invariant. Test on both sides of the surface using the same batch.
        let mut reversed_vertices = vertices;
        reversed_vertices.swap(1, 2);
        let mut reversed = vec![Vector3::zeros(); points.len()];
        tetrahedron_B_batch(
            &points,
            position,
            orientation,
            polarization,
            reversed_vertices,
            &mut reversed,
        );
        for (index, (&actual, &expected)) in reversed.iter().zip(&tetra_out).enumerate() {
            assert!(
                (actual - expected).norm() < 3e-12,
                "tetra winding reversal at scale={scale} index={index}",
            );
        }
    }
}

#[test]
fn zero_area_and_reversed_open_mesh_face_preserve_physical_charge() {
    let nodes = gauss_legendre_nodes(QUAD_ORDER);
    let polarization = vector![POLARIZATION[0], POLARIZATION[1], POLARIZATION[2]];
    let vertices = triangle_vertices(0.2);
    let (position, orientation) = pose(0.2);
    let local = point![0.11, -0.17, 0.37];
    let observer = global_point(local, position, orientation);
    let mesh = TriMesh::new_unchecked(vertices, [[0, 1, 2], [0, 0, 1]]);
    let reversed = TriMesh::new_unchecked(vertices, [[0, 2, 1], [0, 0, 1]]);
    let mut field = [Vector3::zeros(); 1];
    let mut reversed_field = [Vector3::zeros(); 1];
    mesh_B_batch(
        &[observer],
        position,
        orientation,
        polarization,
        &mesh,
        &mut field,
    );
    mesh_B_batch(
        &[observer],
        position,
        orientation,
        polarization,
        &reversed,
        &mut reversed_field,
    );
    let expected = orientation * charged_face_quadrature(local, vertices, polarization, &nodes);
    assert_field_f64(field[0], expected, "open mesh face + degenerate face");
    assert_field_f64(reversed_field[0], -expected, "reversed open mesh face");
    assert!(
        (field[0] + reversed_field[0]).norm() < 1e-12,
        "reversing open face winding must reverse its signed surface charge",
    );

    // Repeated endpoints are exactly zero-area in floating-point arithmetic.
    let collinear = [vertices[0], vertices[1], vertices[1]];
    let mut zero = [vector![f64::NAN, f64::NAN, f64::NAN]; 1];
    triangle_B_batch(
        &[observer],
        position,
        orientation,
        polarization,
        collinear,
        &mut zero,
    );
    assert_eq!(zero[0], Vector3::zeros());
}
