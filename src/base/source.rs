/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use enum_dispatch::enum_dispatch;
use nalgebra::{Point3, RealField, Vector3};

use crate::{
    base::{DynClone, Transform},
    crate_util::need_std,
};

// MARK: Source

#[enum_dispatch]
/// Physical representation of magnetic sources.
pub trait Source<T: RealField>: Transform<T> + Send + Sync + DynClone {
    /// Computes the magnetic field (B) at the given point.
    ///
    /// # Arguments
    ///
    /// - `point`: Observer positions (m)
    ///
    /// # Returns
    ///
    /// - B-field vector
    #[allow(non_snake_case)]
    fn compute_B(&self, point: Point3<T>) -> Vector3<T>;

    /// Computes the magnetic field (B) at the given points in batch.
    ///
    /// # Arguments
    ///
    /// - `points`: Slice of observer positions (m)
    ///
    /// # Returns
    ///
    /// - B-field vectors at each observer.
    #[allow(non_snake_case)]
    #[cfg(feature = "alloc")]
    fn compute_B_batch(&self, points: &[Point3<T>]) -> alloc::vec::Vec<Vector3<T>>;

    /// A default formatter that behaves like Display.
    /// Last argument is the indentation, which is for SourceAssembly support.
    /// Override this for custom printouts.
    fn format(&self, f: &mut core::fmt::Formatter<'_>, _: &str) -> core::fmt::Result {
        write!(f, "Source at {}", self.pose())
    }
}

#[cfg(feature = "std")]
impl<T: RealField> core::fmt::Display for dyn Source<T> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        // Delegate to the trait method
        self.format(f, "")
    }
}

// MARK: Box<dyn Source>
need_std!(
    use core::fmt::Display;

    use dyn_clone::clone_trait_object;
    use delegate::delegate;

    use crate::base::{Float, Pose};

    impl<T: Float> Transform<T> for Box<dyn Source<T>> {
        delegate!(
            to (**self) {
                fn pose(&self) -> &Pose<T>;
                fn pose_mut(&mut self) -> &mut Pose<T>;
                fn set_pose(&mut self, pose: Pose<T>);
            }
        );
    }

    clone_trait_object!(<T> Source<T> where T: Float);

    impl<T: Float> core::fmt::Debug for Box<dyn Source<T>> {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            (**self).fmt(f)
        }
    }

    impl<T: Float> Source<T> for Box<dyn Source<T>> {
        delegate!(
            to (**self) {
                fn compute_B(&self, point: Point3<T>) -> Vector3<T>;
                #[cfg(feature = "alloc")]
                fn compute_B_batch(&self, points: &[Point3<T>]) -> Vec<Vector3<T>>;
            }
        );
    }
);
