/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use enum_dispatch::enum_dispatch;
use nalgebra::{Point3, RealField, Vector3};

use crate::{
    base::{DynClone, Transform},
    crate_utils::need_std,
};

// MARK: Source

#[enum_dispatch]
/// Physical representation of magnetic sources.
pub trait Source<T: RealField>: Transform<T> + Send + Sync + DynClone {
    /// Returns an estimate of this source's per-observer field-computation work.
    ///
    /// This is a scheduling hint, not a stable measure of operation count or
    /// physical complexity. Implementations should use saturating arithmetic.
    fn relative_complexity(&self) -> usize {
        1
    }

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
                fn relative_complexity(&self) -> usize;
            }
        );
    }
);

#[cfg(all(test, feature = "std"))]
mod tests {
    use super::*;
    use crate::magnets::Dipole;

    struct DummySource(crate::base::Pose<f64>);
    impl crate::base::Transform<f64> for DummySource {
        fn pose(&self) -> &crate::base::Pose<f64> {
            &self.0
        }
        fn pose_mut(&mut self) -> &mut crate::base::Pose<f64> {
            &mut self.0
        }
    }
    impl Source<f64> for DummySource {
        fn compute_B(&self, _: Point3<f64>) -> Vector3<f64> {
            Vector3::zeros()
        }
        fn compute_B_batch(&self, points: &[Point3<f64>]) -> Vec<Vector3<f64>> {
            vec![Vector3::zeros(); points.len()]
        }
    }
    impl Clone for DummySource {
        fn clone(&self) -> Self {
            DummySource(self.0)
        }
    }

    #[derive(Clone)]
    struct ComplexSource(crate::base::Pose<f64>);

    impl crate::base::Transform<f64> for ComplexSource {
        fn pose(&self) -> &crate::base::Pose<f64> {
            &self.0
        }

        fn pose_mut(&mut self) -> &mut crate::base::Pose<f64> {
            &mut self.0
        }
    }

    impl Source<f64> for ComplexSource {
        fn relative_complexity(&self) -> usize {
            usize::MAX
        }

        fn compute_B(&self, _: Point3<f64>) -> Vector3<f64> {
            Vector3::zeros()
        }

        fn compute_B_batch(&self, points: &[Point3<f64>]) -> Vec<Vector3<f64>> {
            vec![Vector3::zeros(); points.len()]
        }
    }

    #[test]
    fn test_source_trait_defaults_and_box() {
        let dummy = DummySource(crate::base::Pose::default());
        let dyn_src: &dyn Source<f64> = &dummy;
        assert_eq!(dyn_src.relative_complexity(), 1);
        let s = format!("{}", dyn_src);
        assert!(s.contains("Source at"));

        let mut dummy_box: Box<dyn Source<f64>> = Box::new(dummy.clone());
        assert_eq!(dummy_box.relative_complexity(), 1);
        let _ = dummy_box.pose_mut();
        let _ = dummy_box.compute_B(Point3::origin());
        let _ = dummy_box.compute_B_batch(&[Point3::origin()]);
        let _ = dummy_box.clone();

        let boxed: Box<dyn Source<f64>> = Box::new(Dipole::<f64>::default());
        let dbg = format!("{:?}", boxed);
        assert!(dbg.contains("Dipole"));

        let mut cloned = boxed.clone();
        cloned.set_pose(crate::base::Pose::default());
        let _ = cloned.pose_mut();
        let _ = cloned.compute_B(Point3::origin());
        let _ = cloned.compute_B_batch(&[Point3::origin()]);

        let complex_box: Box<dyn Source<f64>> =
            Box::new(ComplexSource(crate::base::Pose::default()));
        assert_eq!(complex_box.relative_complexity(), usize::MAX);
    }
}
