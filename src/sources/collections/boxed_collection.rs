/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use crate::{Collection, Source};
use nalgebra::UnitQuaternion;

/// Alias of [Collection] for heap-allocated collection of multiple source types.
pub type BoxedCollection<T> = Collection<Box<dyn Source<T>>, T>;

#[cfg(test)]
mod tests {
    use std::f64::consts::{FRAC_PI_3, PI};

    use super::*;
    use crate::{sources::*, testing_util::*};
    use nalgebra::{Translation3, point, vector};

    fn get_collection() -> BoxedCollection<f64> {
        let mut collection = BoxedCollection::default();
        collection.add(Box::new(CylinderMagnet::new(
            point![0.005, 0.01, 0.015],
            UnitQuaternion::identity(),
            vector![0.1, 0.2, 0.3],
            0.04,
            0.05,
        )));
        collection.add(Box::new(CuboidMagnet::new(
            point![0.015, 0.005, 0.01],
            quat_from_rotvec(0.0, FRAC_PI_3, 0.0),
            vector![0.1, 0.2, 0.3],
            vector![0.02, 0.02, 0.03],
        )));
        collection
    }

    #[test]
    fn test_collection() {
        let collection = get_collection();
        test_B_magnet!(@small, &collection, "multi-collection.csv", 5e-11);
    }

    #[test]
    fn test_collection_translate() {
        let mut collection = get_collection();
        let translation = Translation3::new(0.01, 0.015, 0.02);
        collection.translate(translation.clone());
        test_B_magnet!(@small, &collection, "multi-collection-translate.csv", 5e-11);

        collection.translate(translation.inverse());
        collection.set_position(point![0.01, 0.015, 0.02]);
        test_B_magnet!(@small, &collection, "multi-collection-translate.csv", 5e-11);
    }

    #[test]
    fn test_collection_rotate() {
        let mut collection = get_collection();
        let rotation = quat_from_rotvec(PI / 3.0, PI / 4.0, PI / 5.0);
        collection.rotate(rotation);
        test_B_magnet!(@small, &collection, "multi-collection-rotate.csv", 5e-11);

        collection.rotate(rotation.inverse());
        collection.set_orientation(rotation);
        test_B_magnet!(@small, &collection, "multi-collection-rotate.csv", 5e-11);

        collection.set_position(point![0.01, 0.015, 0.02]);
        test_B_magnet!(@small, &collection, "multi-collection-translate-rotate.csv", 5e-11);
    }

    #[test]
    fn test_collection_display() {
        let collection = get_collection();

        println!("{}", collection);
        assert_eq!("SourceCollection (2 children) at pos=[0, 0, 0], q=[0, 0, 0, 1]
├── 0: CylinderMagnet (pol=[0.1, 0.2, 0.3], d=0.04, h=0.05) at pos=[0.005, 0.01, 0.015], q=[0, 0, 0, 1]
└── 1: CuboidMagnet (pol=[0.1, 0.2, 0.3], dim=[0.02, 0.02, 0.03]) at pos=[0.015, 0.005, 0.01], q=[0, 0.5, 0, <float>]",
         mask_long_floats(&format!("{}", collection)))
    }
}
