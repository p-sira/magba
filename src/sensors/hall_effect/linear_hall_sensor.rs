/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use std::fmt::Display;

use getset::Getters;
use nalgebra::{Point3, UnitQuaternion, Vector3};

use crate::{
    base::{
        Float, Observer, Pose, SensorOutput, Source, pose::impl_pose_methods,
        transform::impl_transform,
    },
    measurement::hall_effect::linear_hall_voltage,
};

/// A physical representation of a linear Hall effect sensor.
///
/// # Fields
///
/// - `position`: Center of the cylinder (m)
/// - `orientation`: Orientation as unit quaternion
/// - `polarization`: Polarization vector (T)
/// - `diameter`: Cylinder diameter (m)
/// - `height`: Cylinder height (m)
///
#[derive(Clone, Debug, PartialEq, Eq, Getters)]
#[getset(get = "pub")]
pub struct LinearHallSensor<T: Float = f64> {
    pose: Pose<T>,
    sensitivity_vector: Vector3<T>,
    quiescent_voltage: T,
    min_voltage: T,
    max_voltage: T,
}

impl_transform!(LinearHallSensor<T> where T: Float);

impl<T: Float> LinearHallSensor<T> {
    // MARK: New
    pub fn new(
        position: impl Into<Point3<T>>,
        orientation: UnitQuaternion<T>,
        sensitive_axis: Vector3<T>,
        sensitivity: T,
        supply_voltage: T,
    ) -> Self {
        let two = T::from_f64(2.0).unwrap();
        let sensitivity_vector = sensitive_axis.normalize() * sensitivity;

        Self {
            pose: Pose::new(position.into(), orientation),
            sensitivity_vector,
            quiescent_voltage: supply_voltage / two,
            min_voltage: T::zero(),
            max_voltage: supply_voltage,
        }
    }

    impl_pose_methods!();

    // MARK: Read

    /// Computes the analog output voltage (V) in the presence of a magnetic source.
    ///
    /// # Arguments
    ///
    /// - `source`: Magnetic [Source]
    ///
    /// # Returns
    ///
    /// - ADC voltage (V)
    #[inline]
    pub fn read_voltage(&self, source: &dyn Source<T>) -> T {
        let b_field = source.compute_B(self.pose.position());
        let global_sensitivity_vector = self.pose.orientation() * self.sensitivity_vector;

        linear_hall_voltage(
            b_field,
            global_sensitivity_vector,
            self.quiescent_voltage,
            self.min_voltage,
            self.max_voltage,
        )
    }

    // MARK: Getters

    #[inline]
    pub fn sensitivity(&self) -> T {
        self.sensitivity_vector.magnitude()
    }

    #[inline]
    pub fn sensitive_axis(&self) -> Vector3<T> {
        self.sensitivity_vector.normalize()
    }

    #[inline]
    pub fn supply_voltage(&self) -> T {
        self.quiescent_voltage * T::from(2.0).unwrap()
    }

    // MARK: Setters

    #[inline]
    pub fn set_sensitivity(&mut self, sensitivity: T) {
        self.sensitivity_vector = self.sensitive_axis() * sensitivity;
    }

    #[inline]
    pub fn set_supply_voltage(&mut self, supply_voltage: T) {
        let two = T::from_f64(2.0).unwrap();
        self.max_voltage = supply_voltage;
        self.quiescent_voltage = supply_voltage / two;
    }

    // MARK: With setters

    #[inline]
    pub fn with_sensitivity(mut self, sensitivity: T) -> Self {
        self.set_sensitivity(sensitivity);
        self
    }

    #[inline]
    pub fn with_supply_voltage(mut self, supply_voltage: T) -> Self {
        self.set_supply_voltage(supply_voltage);
        self
    }
}

impl<T: Float> Default for LinearHallSensor<T> {
    fn default() -> Self {
        Self {
            pose: Default::default(),
            sensitivity_vector: Vector3::z(),
            quiescent_voltage: T::from(2.5).unwrap(),
            min_voltage: T::zero(),
            max_voltage: T::from(5.0).unwrap(),
        }
    }
}

impl<T: Float> Observer<T> for LinearHallSensor<T> {
    /// Alias of [LinearHallSensor::read_voltage].
    fn read(&self, source: &dyn Source<T>) -> SensorOutput<T> {
        SensorOutput::Scalar(self.read_voltage(source))
    }

    // MARK: Display
    fn format(&self, f: &mut std::fmt::Formatter<'_>, _: &str) -> std::fmt::Result {
        let sensitive_axis = self.sensitive_axis();
        write!(
            f,
            "LinearHallSensor (sensitive_axis=[{:?}, {:?}, {:?}], sensitivity={:?}, supply_voltage={:?}) at {}",
            sensitive_axis.x,
            sensitive_axis.y,
            sensitive_axis.z,
            self.sensitivity(),
            self.supply_voltage(),
            self.pose
        )
    }
}

impl<T: Float> Display for LinearHallSensor<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        <Self as Observer<T>>::format(self, f, "")
    }
}
