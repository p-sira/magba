//! Independent Wolfram high-precision field references, in SI units.

#[cfg(feature = "mesh")]
use magba::base::mesh::TriMesh;
use magba::{base::Float, prelude::*};
use nalgebra::{Point3, UnitQuaternion, Vector3};

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
            let values = record
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

fn check<T: Float, S: Source<T>>(
    case: &str,
    precision: &str,
    source: S,
    row: &[f64],
    at: usize,
    (rel, abs): (f64, f64),
) {
    let expected = &row[at + 3..at + 6];
    let scale = expected.iter().copied().map(f64::abs).fold(0.0, f64::max);
    let allowed = abs + rel * scale;
    let actual = source.compute_B(Point3::from(vec3::<T>(&row[at..at + 3])));
    for (axis, (got, &want)) in ["bx", "by", "bz"]
        .into_iter()
        .zip(actual.iter().zip(expected))
    {
        let got = got.to_f64().unwrap();
        assert!(
            got.is_finite() && (got - want).abs() <= allowed,
            "{case} {precision} {axis}: got {got:.16e}, expected {want:.16e}, error {:.3e} > tolerance {allowed:.3e} T",
            (got - want).abs()
        );
    }
}

// $make constructs one source with T=f64 and again with T=f32, using the same row.
macro_rules! source_test {
    ($(#[$attr:meta])* $test:ident, $file:literal, [$($column:literal),+], $at:expr,
     $tol64:expr, $tol32:expr, |$t:ident, $row:ident| $make:expr) => {
        $(#[$attr])*
        #[test]
        fn $test() {
            for (case, values) in fixtures($file, &[$($column),+]) {
                {
                    type $t = f64;
                    let $row = values.as_slice();
                    check(&case, "f64", $make, $row, $at, $tol64);
                }
                {
                    type $t = f32;
                    let $row = values.as_slice();
                    check(&case, "f32", $make, $row, $at, $tol32);
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
const CURRENT_TRIANGLE: [[f64; 3]; 3] =
    [[-0.5, -0.5, -0.25], [0.5, -0.5, -0.25], [0.0, 0.5, -0.25]];
const MAGNET_TRIANGLE: [[f64; 3]; 3] = [[-0.5, -0.5, -0.5], [0.5, -0.5, 0.5], [0.0, 0.5, 0.0]];
const TETRA: [[f64; 3]; 4] = [
    [-0.5, -0.5, -0.5],
    [0.5, -0.5, -0.5],
    [0.0, 0.5, -0.5],
    [0.0, 0.0, 0.5],
];
#[cfg(feature = "mesh")]
const TETRA_FACES: [[usize; 3]; 4] = [[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2]];

#[cfg(feature = "mesh")]
fn tetra_mesh<T: Float + std::iter::Sum>() -> TriMesh<T> {
    TriMesh::new(geometry(&TETRA).to_vec(), TETRA_FACES.to_vec()).unwrap()
}

source_test!(
    circular_current,
    "circular_current",
    [
        "case", "diameter", "current", "x", "y", "z", "bx", "by", "bz"
    ],
    2,
    (1e-12, 1e-19),
    (3e-5, 1e-12),
    |T, v| CircularCurrent::<T>::new(
        [0.0; 3],
        UnitQuaternion::identity(),
        scalar(v[0]),
        scalar(v[1])
    )
);
source_test!(
    cuboid,
    "cuboid",
    [
        "case", "jx", "jy", "jz", "dx", "dy", "dz", "x", "y", "z", "bx", "by", "bz"
    ],
    6,
    (1e-12, 1e-15),
    (5e-5, 1e-8),
    |T, v| CuboidMagnet::<T>::new(
        [0.0; 3],
        UnitQuaternion::identity(),
        vec3(&v[..3]),
        vec3(&v[3..6])
    )
);
source_test!(
    dipole,
    "dipole",
    ["case", "mx", "my", "mz", "x", "y", "z", "bx", "by", "bz"],
    3,
    (1e-12, 1e-22),
    (2e-6, 1e-13),
    |T, v| Dipole::<T>::new([0.0; 3], UnitQuaternion::identity(), vec3(&v[..3]))
);
source_test!(
    sphere,
    "sphere",
    [
        "case", "jx", "jy", "jz", "diameter", "x", "y", "z", "bx", "by", "bz"
    ],
    4,
    (1e-12, 1e-16),
    (2e-6, 1e-9),
    |T, v| SphereMagnet::<T>::new(
        [0.0; 3],
        UnitQuaternion::identity(),
        vec3(&v[..3]),
        scalar(v[3])
    )
);
source_test!(
    cylinder,
    "cylinder",
    [
        "case", "jx", "jy", "jz", "diameter", "height", "x", "y", "z", "bx", "by", "bz"
    ],
    5,
    (1e-12, 1e-14),
    (5e-5, 1e-8),
    |T, v| CylinderMagnet::<T>::new(
        [0.0; 3],
        UnitQuaternion::identity(),
        vec3(&v[..3]),
        scalar(v[3]),
        scalar(v[4])
    )
);
source_test!(
    path_current,
    "path_current",
    ["case", "current", "x", "y", "z", "bx", "by", "bz"],
    1,
    (1e-12, 1e-19),
    (5e-5, 1e-12),
    |T, v| PathCurrent::<T>::new(
        [0.0; 3],
        UnitQuaternion::identity(),
        scalar(v[0]),
        geometry(&PATH).to_vec()
    )
);
source_test!(
    triangle_current,
    "triangle_current",
    ["case", "kx", "ky", "kz", "x", "y", "z", "bx", "by", "bz"],
    3,
    (1e-11, 1e-19),
    (2e-4, 1e-12),
    |T, v| TriangleCurrent::<T>::new(
        [0.0; 3],
        UnitQuaternion::identity(),
        vec3(&v[..3]),
        geometry(&CURRENT_TRIANGLE)
    )
);
source_test!(
    #[cfg(feature = "mesh")]
    sheet_current,
    "sheet_current",
    ["case", "kx", "ky", "kz", "x", "y", "z", "bx", "by", "bz"],
    3,
    (1e-11, 1e-19),
    (5e-4, 1e-12),
    |T, v| SheetCurrent::<T>::new(
        [0.0; 3],
        UnitQuaternion::identity(),
        vec![vec3(&v[..3]); 4],
        tetra_mesh::<T>()
    )
);
source_test!(
    triangle_magnet,
    "triangle_magnet",
    ["case", "jx", "jy", "jz", "x", "y", "z", "bx", "by", "bz"],
    3,
    (1e-11, 1e-15),
    (1e-4, 1e-8),
    |T, v| TriangleMagnet::<T>::new(
        [0.0; 3],
        UnitQuaternion::identity(),
        vec3(&v[..3]),
        geometry(&MAGNET_TRIANGLE)
    )
);
source_test!(
    tetrahedron_magnet,
    "tetrahedron_magnet",
    ["case", "jx", "jy", "jz", "x", "y", "z", "bx", "by", "bz"],
    3,
    (1e-11, 1e-15),
    (1e-4, 1e-8),
    |T, v| TetrahedronMagnet::<T>::new(
        [0.0; 3],
        UnitQuaternion::identity(),
        vec3(&v[..3]),
        geometry(&TETRA)
    )
);
source_test!(
    #[cfg(feature = "mesh")]
    mesh_magnet,
    "mesh_magnet",
    ["case", "jx", "jy", "jz", "x", "y", "z", "bx", "by", "bz"],
    3,
    (1e-11, 1e-15),
    (1e-4, 1e-8),
    |T, v| MeshMagnet::<T>::new(
        [0.0; 3],
        UnitQuaternion::identity(),
        vec3(&v[..3]),
        tetra_mesh::<T>()
    )
);
