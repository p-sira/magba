//! Independent Wolfram high-precision field references, in SI units.

#[cfg(feature = "mesh")]
use magba::base::mesh::TriMesh;
use magba::{base::Float, prelude::*};
use nalgebra::{Point3, UnitQuaternion, Vector3};

const SHIFT: [f64; 3] = [0.25, -0.375, 0.125];
const ROTATION: [f64; 3] = [0.37, -0.23, 0.51];

fn fixtures(name: &str, columns: &[&str]) -> Vec<(String, Vec<f64>)> {
    let path = format!(
        "{}/testing/data/high-precision/{name}.csv",
        env!("CARGO_MANIFEST_DIR")
    );
    let mut csv = csv::Reader::from_path(&path)
        .unwrap_or_else(|err| panic!("cannot read {path} (checkout testing submodule): {err}"));
    assert_eq!(
        csv.headers().unwrap().iter().collect::<Vec<_>>(),
        columns,
        "{path}: unexpected CSV columns"
    );
    let rows = csv
        .records()
        .map(|record| {
            let record = record.unwrap_or_else(|err| panic!("{path}: {err}"));
            let case = record[0].to_owned();
            assert!(!case.is_empty(), "{path}: empty case name");
            let values: Vec<f64> = record
                .iter()
                .skip(1)
                .zip(&columns[1..])
                .map(|(raw, column)| {
                    let value = raw
                        .parse::<f64>()
                        .unwrap_or_else(|err| panic!("{path}: {case} {column}={raw:?}: {err}"));
                    assert!(
                        value.is_finite(),
                        "{path}: {case} {column}={value} is nonfinite"
                    );
                    value
                })
                .collect();
            let case = if columns.get(1) == Some(&"shape") {
                format!("{case} shape={}", values[0])
            } else {
                case
            };
            (case, values)
        })
        .collect::<Vec<_>>();
    assert!(!rows.is_empty(), "{path}: no reference cases");
    rows
}

fn scalar<T: Float>(v: f64) -> T {
    num_traits::cast(v).expect("finite fixture value must fit target float")
}

fn vec3<T: Float>(v: &[f64]) -> Vector3<T> {
    Vector3::new(scalar(v[0]), scalar(v[1]), scalar(v[2]))
}

fn geometry<T: Float, const N: usize>(points: &[[f64; 3]; N]) -> [Vector3<T>; N] {
    points.map(|v| vec3(&v))
}

fn cuboid_f32_tolerance(row: &[f64]) -> (f64, f64) {
    let mut dimensions = [row[3], row[4], row[5]];
    dimensions.sort_by(f64::total_cmp);
    // Long, thin rods lose more f32 precision near their narrow edges.
    (
        if dimensions[2] >= 32.0 * dimensions[1] {
            1e-3
        } else {
            5e-5
        },
        1e-8,
    )
}

fn check<T: Float, S: Source<T>>(
    case: &str,
    precision: &str,
    make: impl Fn([T; 3], UnitQuaternion<T>) -> S,
    row: &[f64],
    at: usize,
    length_scale: f64,
    (rel, abs): (f64, f64),
) {
    assert!(
        length_scale.is_finite() && length_scale > 0.0,
        "{case}: invalid pose scale"
    );
    let local_point = Point3::from(vec3::<T>(&row[at..at + 3]));
    let local_field = vec3::<f64>(&row[at + 3..at + 6]);
    let reference_rotation = UnitQuaternion::from_scaled_axis(vec3::<f64>(&ROTATION));
    for (pose, moved, turned) in [
        ("identity", false, false),
        ("translated", true, false),
        ("rotated", false, true),
        ("translated_rotated", true, true),
    ] {
        let position = if moved {
            SHIFT.map(|component| scalar::<T>(component * length_scale))
        } else {
            [T::zero(); 3]
        };
        let rotation = if turned {
            UnitQuaternion::from_scaled_axis(vec3::<T>(&ROTATION))
        } else {
            UnitQuaternion::identity()
        };
        let world_point = rotation.transform_point(&local_point) + Vector3::from(position);
        let expected = if turned {
            reference_rotation * local_field
        } else {
            local_field
        };
        let scale = expected.iter().copied().map(f64::abs).fold(0.0, f64::max);
        let allowed = abs + rel * scale;
        let actual = make(position, rotation).compute_B(world_point);
        for (axis, (got, &want)) in ["bx", "by", "bz"]
            .into_iter()
            .zip(actual.iter().zip(expected.iter()))
        {
            let got = got.to_f64().unwrap();
            assert!(
                got.is_finite() && (got - want).abs() <= allowed,
                "{case} {pose} {precision} {axis}: got {got:.16e}, expected {want:.16e}, error {:.3e} > tolerance {allowed:.3e} T",
                (got - want).abs()
            );
        }
    }
}

// Build each geometry at four poses in both precisions, reusing the local Wolfram field.
macro_rules! source_test {
    ($(#[$attr:meta])* $test:ident, $file:literal, [$($column:literal),+], $at:expr,
     $scale:expr, $tol64:expr, $tol32:expr,
     |$t:ident, $row:ident, $pos:ident, $rot:ident| $make:expr) => {
        $(#[$attr])*
        #[test]
        fn $test() {
            for (case, values) in fixtures($file, &[$($column),+]) {
                {
                    type $t = f64;
                    let $row = values.as_slice();
                    check(&case, "f64", |$pos, $rot| $make, $row, $at, $scale, $tol64);
                }
                {
                    type $t = f32;
                    let $row = values.as_slice();
                    check(&case, "f32", |$pos, $rot| $make, $row, $at, $scale, $tol32);
                }
            }
        }
    };
}

const PATH: [[f64; 3]; 4] = [
    [-0.5, -0.25, -0.25],
    [0.5, -0.25, -0.25],
    [0.0, 0.5, -0.25],
    [0.0, 0.0, 0.5],
];
const PATH_ALT: [[f64; 3]; 4] = [
    [-0.75, -0.5, 0.25],
    [0.5, -0.25, 0.25],
    [0.75, 0.5, -0.25],
    [-0.25, 0.75, 0.5],
];
const CURRENT_TRIANGLE: [[f64; 3]; 3] =
    [[-0.5, -0.5, -0.25], [0.5, -0.5, -0.25], [0.0, 0.5, -0.25]];
const MAGNET_TRIANGLE: [[f64; 3]; 3] = [[-0.5, -0.5, -0.5], [0.5, -0.5, 0.5], [0.0, 0.5, 0.0]];
const TRIANGLE_ALT: [[f64; 3]; 3] = [
    [-0.75, -0.25, 1.0 / 3.0],
    [0.5, -0.5, 0.125],
    [0.25, 0.75, -1.0 / 3.0],
];
const TETRA: [[f64; 3]; 4] = [
    [-0.5, -0.5, -0.5],
    [0.5, -0.5, -0.5],
    [0.0, 0.5, -0.5],
    [0.0, 0.0, 0.5],
];
const TETRA_ALT: [[f64; 3]; 4] = [
    [-0.75, -0.5, -1.0 / 3.0],
    [0.5, -0.25, -0.5],
    [0.25, 0.75, -0.25],
    [-0.125, 0.125, 0.8],
];
#[cfg(feature = "mesh")]
const TETRA_FACES: [[usize; 3]; 4] = [[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2]];
#[cfg(feature = "mesh")]
const BIPYRAMID: [[f64; 3]; 5] = [
    [-0.75, -0.5, 0.0],
    [0.75, -0.5, 0.0],
    [0.0, 0.75, 0.0],
    [0.0, 0.0, 1.25],
    [0.0, 0.0, -0.75],
];
#[cfg(feature = "mesh")]
const BIPYRAMID_FACES: [[usize; 3]; 6] = [
    [0, 1, 3],
    [1, 2, 3],
    [0, 3, 2],
    [0, 4, 1],
    [1, 4, 2],
    [0, 2, 4],
];

fn selected<T: Float, const N: usize>(
    shape: f64,
    original: &[[f64; 3]; N],
    alternate: &[[f64; 3]; N],
) -> [Vector3<T>; N] {
    if shape == 0.0 {
        geometry(original)
    } else if shape == 1.0 {
        geometry(alternate)
    } else {
        panic!("unexpected shape {shape}; expected 0 or 1")
    }
}

#[cfg(feature = "mesh")]
fn poly_mesh<T: Float + std::iter::Sum>(shape: f64) -> TriMesh<T> {
    let (vertices, faces) = if shape == 2.0 {
        (geometry(&BIPYRAMID).to_vec(), BIPYRAMID_FACES.to_vec())
    } else {
        (
            selected(shape, &TETRA, &TETRA_ALT).to_vec(),
            TETRA_FACES.to_vec(),
        )
    };
    TriMesh::new(vertices, faces).unwrap()
}

source_test!(
    circular_current,
    "circular_current",
    [
        "case", "diameter", "current", "x", "y", "z", "bx", "by", "bz"
    ],
    2,
    v[0],
    (1e-12, 1e-19),
    (3e-5, 1e-12),
    |T, v, pos, rot| CircularCurrent::<T>::new(pos, rot, scalar(v[0]), scalar(v[1]))
);
source_test!(
    cuboid,
    "cuboid",
    [
        "case", "jx", "jy", "jz", "dx", "dy", "dz", "x", "y", "z", "bx", "by", "bz"
    ],
    6,
    v[3].min(v[4]).min(v[5]),
    (3e-12, 1e-15),
    cuboid_f32_tolerance(v),
    |T, v, pos, rot| CuboidMagnet::<T>::new(pos, rot, vec3(&v[..3]), vec3(&v[3..6]))
);
source_test!(
    dipole,
    "dipole",
    ["case", "mx", "my", "mz", "x", "y", "z", "bx", "by", "bz"],
    3,
    v[3].abs().max(v[4].abs()).max(v[5].abs()),
    (1e-12, 1e-22),
    (2e-6, 1e-13),
    |T, v, pos, rot| Dipole::<T>::new(pos, rot, vec3(&v[..3]))
);
source_test!(
    sphere,
    "sphere",
    [
        "case", "jx", "jy", "jz", "diameter", "x", "y", "z", "bx", "by", "bz"
    ],
    4,
    v[3],
    (1e-12, 1e-16),
    (2e-6, 1e-9),
    |T, v, pos, rot| SphereMagnet::<T>::new(pos, rot, vec3(&v[..3]), scalar(v[3]))
);
source_test!(
    cylinder,
    "cylinder",
    [
        "case", "jx", "jy", "jz", "diameter", "height", "x", "y", "z", "bx", "by", "bz"
    ],
    5,
    v[3].min(v[4]),
    (1e-12, 1e-14),
    (5e-5, 1e-8),
    |T, v, pos, rot| CylinderMagnet::<T>::new(pos, rot, vec3(&v[..3]), scalar(v[3]), scalar(v[4]))
);
source_test!(
    path_current,
    "path_current",
    ["case", "shape", "current", "x", "y", "z", "bx", "by", "bz"],
    2,
    1.0,
    (1e-12, 1e-19),
    (5e-5, 1e-12),
    |T, v, pos, rot| PathCurrent::<T>::new(
        pos,
        rot,
        scalar(v[1]),
        selected(v[0], &PATH, &PATH_ALT).to_vec()
    )
);
source_test!(
    triangle_current,
    "triangle_current",
    [
        "case", "shape", "kx", "ky", "kz", "x", "y", "z", "bx", "by", "bz"
    ],
    4,
    1.0,
    (1e-11, 1e-19),
    (2e-4, 1e-12),
    |T, v, pos, rot| TriangleCurrent::<T>::new(
        pos,
        rot,
        vec3(&v[1..4]),
        selected(v[0], &CURRENT_TRIANGLE, &TRIANGLE_ALT)
    )
);
source_test!(
    #[cfg(feature = "mesh")]
    sheet_current,
    "sheet_current",
    [
        "case", "shape", "kx", "ky", "kz", "x", "y", "z", "bx", "by", "bz"
    ],
    4,
    1.0,
    (1e-11, 1e-19),
    (5e-4, 1e-12),
    |T, v, pos, rot| {
        let mesh = poly_mesh::<T>(v[0]);
        SheetCurrent::<T>::new(pos, rot, vec![vec3(&v[1..4]); mesh.triangles().len()], mesh)
    }
);
source_test!(
    triangle_magnet,
    "triangle_magnet",
    [
        "case", "shape", "jx", "jy", "jz", "x", "y", "z", "bx", "by", "bz"
    ],
    4,
    1.0,
    (1e-11, 1e-15),
    (1e-4, 1e-8),
    |T, v, pos, rot| TriangleMagnet::<T>::new(
        pos,
        rot,
        vec3(&v[1..4]),
        selected(v[0], &MAGNET_TRIANGLE, &TRIANGLE_ALT)
    )
);
source_test!(
    tetrahedron_magnet,
    "tetrahedron_magnet",
    [
        "case", "shape", "jx", "jy", "jz", "x", "y", "z", "bx", "by", "bz"
    ],
    4,
    1.0,
    (1e-11, 1e-15),
    (1e-4, 1e-8),
    |T, v, pos, rot| TetrahedronMagnet::<T>::new(
        pos,
        rot,
        vec3(&v[1..4]),
        selected(v[0], &TETRA, &TETRA_ALT)
    )
);
source_test!(
    #[cfg(feature = "mesh")]
    mesh_magnet,
    "mesh_magnet",
    [
        "case", "shape", "jx", "jy", "jz", "x", "y", "z", "bx", "by", "bz"
    ],
    4,
    1.0,
    (1e-11, 1e-15),
    (1e-4, 1e-8),
    |T, v, pos, rot| MeshMagnet::<T>::new(pos, rot, vec3(&v[1..4]), poly_mesh::<T>(v[0]))
);
