//! MagPyLib 5.2.3 reference fields and geometric consistency for cylinder segments.

#![cfg(feature = "std")]

use approx::assert_relative_eq;
use magba::{
    fields::{cylinder_segment_B, cylinder_segment_B_batch},
    prelude::*,
};
use nalgebra::{Point3, UnitQuaternion, Vector3, point, vector};

#[test]
fn reference_fields_from_magpylib() {
    // (dimensions, observer, B), with J=(0.2, -0.3, 0.4) T.
    // Covers a solid sector, annular sector, wrapped azimuth range and full rings.
    let cases: [([f64; 5], [f64; 3], [f64; 3]); 10] = [
        (
            [0.0, 0.02, 0.02, -20.0, 100.0],
            [0.005, 0.003, 0.0],
            [
                0.14776698402587196,
                -0.20132151206974694,
                0.2832242334814179,
            ],
        ),
        (
            [0.0, 0.02, 0.02, -20.0, 100.0],
            [0.03, 0.01, 0.03],
            [
                0.006546273267888944,
                0.0054657542981008155,
                0.007359158272544878,
            ],
        ),
        (
            [0.01, 0.02, 0.01, -30.0, 90.0],
            [0.0, 0.0, 0.001],
            [
                -0.004364874953641248,
                0.0026080122127318016,
                -0.02707871045534397,
            ],
        ),
        (
            [0.01, 0.02, 0.01, -30.0, 90.0],
            [0.015, 0.003, 0.002],
            [0.156009814515993, -0.28227444201567903, 0.20917543734358962],
        ),
        (
            [0.01, 0.02, 0.01, -30.0, 90.0],
            [0.025, 0.01, 0.02],
            [
                0.005973860629933493,
                0.00635141571148056,
                0.007271483955310592,
            ],
        ),
        (
            [0.01, 0.02, 0.01, -30.0, 90.0],
            [0.0, 0.0, 0.0],
            [
                -0.000580209880232558,
                0.004738012265774639,
                -0.027290396065419883,
            ],
        ),
        (
            [0.003, 0.02, 0.02, 170.0, 290.0],
            [-0.011, -0.010, 0.002],
            [
                0.16070535195669253,
                -0.22114953143086635,
                0.2746261732265762,
            ],
        ),
        (
            [0.003, 0.02, 0.02, 170.0, 290.0],
            [0.01, 0.012, 0.03],
            [
                0.0010183894376343755,
                0.006145354987448535,
                0.0008207472778445995,
            ],
        ),
        (
            [0.0, 0.02, 0.02, 0.0, 360.0],
            [0.005, 0.003, 0.002],
            [0.1592071705812077, -0.22955985490224073, 0.1847166657803291],
        ),
        (
            [0.01, 0.02, 0.02, 0.0, 360.0],
            [0.015, 0.003, 0.002],
            [
                0.12473473374755506,
                -0.27022200455857603,
                0.2775327920600319,
            ],
        ),
    ];

    for (dimensions, observer, reference) in cases {
        let actual = cylinder_segment_B(
            Point3::from(observer),
            Point3::origin(),
            UnitQuaternion::identity(),
            vector![0.2, -0.3, 0.4],
            dimensions.into(),
        );
        assert_relative_eq!(
            actual,
            Vector3::from(reference),
            epsilon = 5e-8,
            max_relative = 5e-7
        );
    }
}

#[test]
fn independent_surface_charge_oracle_at_distant_point() {
    // Independent 192x192 Gauss-Legendre integration of all six charged surfaces.
    // The MagPyLib elliptic-antiderivative result differs by ~2e-5 relatively
    // here, where individual terms cancel strongly.
    let actual = cylinder_segment_B(
        point![-0.2777777777777778, 0.05555555555555558, -0.5],
        point![0.1, 0.2, 0.3],
        UnitQuaternion::from_scaled_axis(vector![
            core::f64::consts::PI / 7.0,
            core::f64::consts::PI / 6.0,
            core::f64::consts::PI / 5.0,
        ]),
        vector![1.0, 2.0, 3.0],
        vector![0.01, 0.03, 0.02, -30.0, 120.0],
    );
    let expected = vector![6.947248457665e-6, 1.250868199195e-6, 1.512474357605e-5];
    assert_relative_eq!(actual, expected, epsilon = 2e-12, max_relative = 1e-8);
}

#[test]
fn segment_boundaries_and_small_displacements() {
    let dimensions = vector![0.01, 0.02, 0.01, -30.0, 90.0];
    let pol = vector![0.2, -0.3, 0.4];
    let field = |x, y, z| {
        cylinder_segment_B(
            point![x, y, z],
            Point3::origin(),
            UnitQuaternion::identity(),
            pol,
            dimensions,
        )
    };
    let theta = 30.0_f64.to_radians();
    for (r, z) in [(0.01, 0.0), (0.02, 0.0), (0.015, 0.005), (0.015, -0.005)] {
        assert_eq!(field(r * theta.cos(), r * theta.sin(), z), Vector3::zeros());
    }
    assert_eq!(field(0.0, 0.015, 0.0), Vector3::zeros());
    assert_eq!(
        field(
            0.015 * (-30.0_f64).to_radians().cos(),
            0.015 * (-30.0_f64).to_radians().sin(),
            0.0
        ),
        Vector3::zeros()
    );
    for delta in [-1e-6, 1e-6] {
        let field_near = field(
            (0.01 + delta) * theta.cos(),
            (0.01 + delta) * theta.sin(),
            0.001,
        );
        assert!(field_near.iter().all(|v| v.is_finite()), "{field_near:?}");
    }
    assert_eq!(
        cylinder_segment_B(
            point![0.03, 0.03, 0.03],
            Point3::origin(),
            UnitQuaternion::identity(),
            Vector3::zeros(),
            dimensions
        ),
        Vector3::zeros()
    );
}

#[test]
fn pose_batch_and_source_collection() {
    let dimensions = vector![0.01, 0.02, 0.01, -30.0, 90.0];
    let pol = vector![0.2, -0.3, 0.4];
    let rotation = UnitQuaternion::from_scaled_axis(vector![0.37_f64, -0.23, 0.51]);
    let position = point![0.2, -0.1, 0.3];
    let local = point![0.025, 0.01, 0.02];
    let global = Point3::from(rotation * local.coords + position.coords);
    let raw = cylinder_segment_B(
        local,
        Point3::origin(),
        UnitQuaternion::identity(),
        pol,
        dimensions,
    );
    let magnet = CylinderSegmentMagnet::new(position, rotation, pol, dimensions);
    let expected = rotation * raw;
    assert_relative_eq!(magnet.compute_B(global), expected, epsilon = 5e-10);

    let points = vec![global; 48];
    let mut out = vec![Vector3::zeros(); points.len()];
    cylinder_segment_B_batch(&points, position, rotation, pol, dimensions, &mut out);
    for value in out {
        assert_relative_eq!(value, expected, epsilon = 5e-10);
    }
    let grouped: magba::collections::SourceComponent = magnet.into();
    assert_relative_eq!(grouped.compute_B(global), expected, epsilon = 5e-10);

    let f32_magnet = CylinderSegmentMagnet::<f32>::new(
        [0.0; 3],
        UnitQuaternion::identity(),
        [0.2, -0.3, 0.4],
        [0.01, 0.02, 0.01, -30.0, 90.0],
    );
    let f32_b = f32_magnet.compute_B(point![0.025, 0.01, 0.02]);
    assert_relative_eq!(f32_b.cast::<f64>(), raw, epsilon = 1e-5);
}

// Partitioning a uniformly polarized magnet must preserve its field. These
// comparisons exercise narrow surface kernels against the cylinder's
// independent elliptic-integral implementation.
#[test]
fn reconstructs_cylinders_near_curved_walls_and_caps() {
    use magba::fields::cylinder_B;
    for inner in [0.0, 0.2] {
        for delta in [1e-4, 1e-6, 1e-8, 1e-10, -1e-10, -1e-8] {
            let angle = 0.61_f64;
            let mut observers = vec![
                point![1.0 + delta, 0.0, 0.1],
                point![
                    (1.0 + delta) * angle.cos(),
                    (1.0 + delta) * angle.sin(),
                    0.1
                ],
                point![0.5, 0.1, 0.5 + delta],
            ];
            if inner > 0.0 {
                observers.push(point![inner + delta, 0.0, 0.1]);
            }
            for p in observers {
                for pol in [Vector3::x(), Vector3::y(), Vector3::z()] {
                    let segment = |start, end| {
                        cylinder_segment_B(
                            p,
                            Point3::origin(),
                            UnitQuaternion::identity(),
                            pol,
                            vector![inner, 1.0, 1.0, start, end],
                        )
                    };
                    let mut expected = cylinder_B(
                        p,
                        Point3::origin(),
                        UnitQuaternion::identity(),
                        pol,
                        2.0,
                        1.0,
                    );
                    if inner > 0.0 {
                        expected -= cylinder_B(
                            p,
                            Point3::origin(),
                            UnitQuaternion::identity(),
                            pol,
                            2.0 * inner,
                            1.0,
                        );
                    }
                    let actual = segment(-90.0, 90.0) + segment(90.0, 270.0);
                    assert_relative_eq!(actual, expected, epsilon = 2e-9, max_relative = 2e-9);
                }
            }
        }
    }
}

#[test]
fn partitioning_at_radial_cut_preserves_field() {
    for delta in [1e-6, 1e-8, 1e-10, -1e-10] {
        let p = point![0.5, delta, 0.1];
        let field = |start, end| {
            cylinder_segment_B(
                p,
                Point3::origin(),
                UnitQuaternion::identity(),
                vector![0.2, -0.3, 0.4],
                vector![0.2, 1.0, 1.0, start, end],
            )
        };
        assert_relative_eq!(
            field(-90.0, 0.0) + field(0.0, 90.0),
            field(-90.0, 90.0),
            epsilon = 2e-9,
        );
    }
}

#[test]
fn thin_segments_preserve_interior_fields() {
    for height in [1e-4, 1e-7, 1e-8, 1e-12] {
        let precise = cylinder_segment_B(
            point![0.5_f64, 0.1, 0.0],
            Point3::origin(),
            UnitQuaternion::identity(),
            Vector3::x(),
            vector![0.0, 1.0, height, -90.0, 90.0],
        );
        let single = cylinder_segment_B(
            point![0.5_f32, 0.1, 0.0],
            Point3::origin(),
            UnitQuaternion::identity(),
            Vector3::x(),
            vector![0.0, 1.0, height as f32, -90.0, 90.0],
        );
        assert_relative_eq!(precise, Vector3::x(), epsilon = height);
        assert_relative_eq!(single.cast::<f64>(), precise, epsilon = 1e-7);
    }
}

#[test]
fn full_cylinders_and_rings_use_segment_surface_convention() {
    for inner in [0.0, 0.2] {
        for pol in [Vector3::x(), Vector3::z(), vector![0.2, -0.3, 0.4]] {
            for p in [
                point![1.0, 0.0, 0.0],
                point![1.0, 0.0, 0.5],
                point![0.5, 0.1, 0.5],
                point![0.5, 0.1, -0.5],
                point![inner, 0.0, 0.0],
            ] {
                if inner == 0.0 && p == Point3::origin() {
                    continue; // The axis of a complete solid cylinder is interior.
                }
                assert_eq!(
                    cylinder_segment_B(
                        p,
                        Point3::origin(),
                        UnitQuaternion::identity(),
                        pol,
                        vector![inner, 1.0, 1.0, 17.0, 377.0],
                    ),
                    Vector3::zeros()
                );
            }
        }
    }
    assert!(
        cylinder_segment_B(
            Point3::origin(),
            Point3::origin(),
            UnitQuaternion::identity(),
            Vector3::z(),
            vector![0.0, 1.0, 1.0, 0.0, 360.0],
        )
        .z > 0.0
    );
}

#[test]
fn distant_segments_recover_dipole_limit() {
    let pol = vector![0.2, -0.3, 0.4];
    for inner in [0.0, 0.2, 0.99] {
        for distance in [1e4, 1e6, 1e8] {
            let p = point![distance, 0.3 * distance, 0.7 * distance];
            let radius: f64 = p.coords.norm();
            // Full ring volume is pi*(1-inner^2)*height. Polarization
            // absorbs mu_0, leaving volume/(4*pi) in the dipole formula.
            let expected = (p.coords * (3.0 * pol.dot(&p.coords) / radius.powi(2)) - pol)
                * ((1.0 - inner * inner) / (4.0 * radius.powi(3)));
            let field = |start, end| {
                cylinder_segment_B(
                    p,
                    Point3::origin(),
                    UnitQuaternion::identity(),
                    pol,
                    vector![inner, 1.0, 1.0, start, end],
                )
            };
            for actual in [field(-90.0, 90.0) + field(90.0, 270.0), field(0.0, 360.0)] {
                assert!(
                    (actual - expected).norm() < 2e-8 * expected.norm(),
                    "inner={inner}, distance={distance}: {actual:?} != {expected:?}"
                );
            }
        }
    }
}

#[test]
fn thin_axial_sectors_reconstruct_cylinder() {
    use magba::fields::cylinder_B;
    for height in [1e-4, 1e-8, 1e-12] {
        let p = point![0.5, 0.1, 0.0];
        let field = |start, end| {
            cylinder_segment_B(
                p,
                Point3::origin(),
                UnitQuaternion::identity(),
                Vector3::z(),
                vector![0.0, 1.0, height, start, end],
            )
        };
        let expected = cylinder_B(
            p,
            Point3::origin(),
            UnitQuaternion::identity(),
            Vector3::z(),
            2.0,
            height,
        );
        assert_relative_eq!(
            field(-90.0, 90.0) + field(90.0, 270.0),
            expected,
            epsilon = 2e-10
        );
    }
}
