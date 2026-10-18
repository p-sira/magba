#![cfg(feature = "mesh")]

use magba::{
    base::{Source, mesh::TriMesh},
    currents::SheetCurrent,
    fields::{sheet_current_B, sheet_current_B_batch, sum_multiple_sheet_current_B},
};
use nalgebra::{Point3, UnitQuaternion, Vector3, point, vector};
use std::panic::{AssertUnwindSafe, catch_unwind};

macro_rules! validation_tests {
    ($module:ident, $float:ty) => {
        mod $module {
            use super::*;
            type T = $float;

            fn mesh() -> TriMesh<T> {
                TriMesh::new(
                    [
                        vector![0.0, 0.0, 0.0],
                        vector![0.001, 0.0, 0.0],
                        vector![0.0, 0.001, 0.0],
                        vector![0.0, 0.0, 0.001],
                    ],
                    [[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2]],
                )
                .unwrap()
            }

            #[test]
            fn mismatches_are_rejected_before_mutation() {
                for count in [0, 3, 5] {
                    let densities = vec![Vector3::<T>::x(); count];
                    assert!(
                        catch_unwind(|| SheetCurrent::new(
                            Point3::origin(),
                            UnitQuaternion::identity(),
                            densities.clone(),
                            mesh()
                        ))
                        .is_err()
                    );
                    let mut sheet = SheetCurrent::new(
                        Point3::origin(),
                        UnitQuaternion::identity(),
                        vec![Vector3::x(); 4],
                        mesh(),
                    );
                    let vertices = vec![
                        vector![0.0, 0.0, 0.0],
                        vector![0.001, 0.0, 0.0],
                        vector![0.0, 0.001, 0.0],
                        vector![0.0, 0.0, 0.001],
                    ];
                    let faces = vec![[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2]];
                    assert!(
                        catch_unwind(|| SheetCurrent::from_vertices_and_faces(
                            vertices,
                            faces,
                            densities.clone()
                        ))
                        .is_err()
                    );
                    let before = sheet.clone();
                    assert!(
                        catch_unwind(AssertUnwindSafe(
                            || sheet.set_current_densities(densities.clone())
                        ))
                        .is_err()
                    );
                    assert_eq!(sheet, before);
                    assert!(
                        catch_unwind(|| sheet.clone().with_current_densities(densities.clone()))
                            .is_err()
                    );
                    assert!(
                        catch_unwind(AssertUnwindSafe(
                            || sheet.set_mesh_and_current_densities(mesh(), densities.clone())
                        ))
                        .is_err()
                    );
                    assert_eq!(sheet, before);
                    let other_mesh = TriMesh::from_triangles(vec![mesh().triangles()[0]; count]);
                    assert!(
                        catch_unwind(AssertUnwindSafe(|| sheet.set_mesh(other_mesh.clone())))
                            .is_err()
                    );
                    assert_eq!(sheet, before);
                    assert!(catch_unwind(|| sheet.with_mesh(other_mesh)).is_err());
                }
            }

            #[test]
            fn geometry_and_densities_can_be_replaced_together() {
                let mut sheet = SheetCurrent::<T>::default();
                let observer = point![0.01, 0.02, 0.03];
                assert_eq!(sheet.compute_B(observer), Vector3::zeros());
                sheet.set_mesh_and_current_densities(mesh(), vec![Vector3::x(); 4]);
                let field = sheet.compute_B(observer);
                assert!(field.norm() > 0.0);
                assert_eq!(sheet.compute_B_batch(&[observer]), vec![field]);
                sheet.set_mesh(mesh());
                assert_eq!(sheet.compute_B(observer), field);
                sheet.set_current_densities(vec![Vector3::zeros(); 4]);
                assert_eq!(sheet.compute_B(observer), Vector3::zeros());
                sheet.set_mesh_and_current_densities(
                    TriMesh::from_triangles(Vec::new()),
                    Vec::new(),
                );
                assert_eq!(sheet, SheetCurrent::default());
            }

            #[test]
            fn field_functions_reject_mismatches_without_writing_output() {
                for count in [0, 3, 5] {
                    let densities = vec![Vector3::<T>::x(); count];
                    let mesh = mesh();
                    assert!(
                        catch_unwind(|| sheet_current_B(
                            point![0.01, 0.02, 0.03],
                            Point3::origin(),
                            UnitQuaternion::identity(),
                            &densities,
                            &mesh
                        ))
                        .is_err()
                    );
                    for points in [vec![], vec![point![0.01, 0.02, 0.03]; 120]] {
                        let mut out = vec![Vector3::repeat(42.0); points.len()];
                        let before = out.clone();
                        assert!(
                            catch_unwind(AssertUnwindSafe(|| sheet_current_B_batch(
                                &points,
                                Point3::origin(),
                                UnitQuaternion::identity(),
                                &densities,
                                &mesh,
                                &mut out
                            )))
                            .is_err()
                        );
                        assert_eq!(out, before);
                        assert!(
                            catch_unwind(AssertUnwindSafe(|| sum_multiple_sheet_current_B(
                                &points,
                                &[Point3::origin()],
                                &[UnitQuaternion::identity()],
                                std::slice::from_ref(&densities),
                                &[&mesh],
                                &mut out,
                            )))
                            .is_err()
                        );
                        assert_eq!(out, before);
                    }
                }
            }
        }
    };
}

validation_tests!(f32_tests, f32);
validation_tests!(f64_tests, f64);
