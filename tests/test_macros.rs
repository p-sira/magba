/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

// #[cfg(not(feature = "no_std"))]
// mod need_std {
//     use magba::{CuboidMagnet, CylinderMagnet, Collection, collection};
//     use nalgebra::{Vector3, vector};

//     #[test]
//     fn test_single_source_collection() {
//         let mut cylinder1 = CylinderMagnet::default();
//         cylinder1.set_polarization(Vector3::zeros());
//         let cylinder2 = CylinderMagnet::default().with_polarization(Vector3::<f64>::z());
//         // let cuboid = CuboidMagnet::default().with_dimensions([1.0, 2.0, 3.0]);
//         assert_eq!(
//             collection!(cylinder1.clone(), cylinder2.clone()),
//             Collection::from_sources(vec![cylinder1.clone(), cylinder2.clone()])
//         );
//     }
// }
