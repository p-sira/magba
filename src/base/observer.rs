/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use delegate::delegate;
#[cfg(feature = "std")]
use dyn_clone::clone_trait_object;
use enum_dispatch::enum_dispatch;
use nalgebra::Vector3;

use crate::{
    base::{DynClone, Float, Source, Transform},
    crate_util::need_std,
};

/// Unified output for varying sensor types.
#[derive(Debug, Clone, PartialEq)]
pub enum SensorOutput<T: Float = f64> {
    /// A single analog value (e.g., 1D Hall voltage, MR resistance).
    Scalar(T),
    /// A 3D analog spatial reading (e.g., 3-Axis Hall sensor).
    Vector(Vector3<T>),
    /// A quantized digital reading from an ADC.
    Digital(i64),
}

#[enum_dispatch]
/// Physical representation of magnetic sensors.
pub trait Observer<T: Float>: Transform<T> + Send + Sync + DynClone {
    /// Acquires a reading from the sensor given a magnetic source environment.
    fn read(&self, source: &dyn Source<T>) -> SensorOutput<T>;

    /// A default formatter that behaves like Display.
    /// Last argument is the indentation, which is for SensorAssembly support.
    /// Override this for custom printouts.
    fn format(&self, f: &mut core::fmt::Formatter<'_>, _: &str) -> core::fmt::Result {
        write!(f, "Observer at {}", self.pose())
    }
}

#[cfg(feature = "std")]
impl<T: Float> core::fmt::Display for dyn Observer<T> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        // Delegate to the trait method
        self.format(f, "")
    }
}

// MARK: Box<dyn Sensor>

need_std!(
    use core::fmt::Display;
    use crate::base::Pose;

    impl<T: Float> Transform<T> for Box<dyn Observer<T>> {
        delegate!(
            to (**self) {
                fn pose(&self) -> &Pose<T>;
                fn pose_mut(&mut self) -> &mut Pose<T>;
                fn set_pose(&mut self, pose: Pose<T>);
            }
        );
    }

    clone_trait_object!(<T> Observer<T> where T: Float);

    impl<T: Float> core::fmt::Debug for Box<dyn Observer<T>> {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            (**self).fmt(f)
        }
    }

    impl<T: Float> Observer<T> for Box<dyn Observer<T>> {
        delegate!(
            to (**self) {
                fn read(&self, source: &dyn Source<T>) -> SensorOutput<T>;
            }
        );
    }
);
