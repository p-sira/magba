/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use nalgebra::{Point3, UnitQuaternion, Vector3};

use crate::{
    base::{Float, Source},
    geometry::Pose,
    measurement::hall_effect::linear_hall_voltage,
    transform::impl_transform,
};

/// A physical representation of a linear Hall effect sensor.
#[derive(Clone, Debug, PartialEq)]
pub struct LinearHallSensor<T: Float = f64> {
    pub pose: Pose<T>,

    // Sensor hardware characteristics
    pub local_sensitive_axis: Vector3<T>,
    pub sensitivity: T,
    pub quiescent_voltage: T,
    pub min_voltage: T,
    pub max_voltage: T,

    // Digital conversion properties
    pub bit_step: T, // bit/V
    pub offset: T,
}

impl_transform!(LinearHallSensor<T> where T: Float);

impl<T: Float> LinearHallSensor<T> {
    /// Create a new Hall effect sensor and automatically computes the ADC bit-step.
    pub fn new(
        position: impl Into<Point3<T>>,
        orientation: UnitQuaternion<T>,
        local_sensitive_axis: Vector3<T>,
        sensitivity: T,
        voltage_supply: T,
        bit: u32,
        offset: T,
    ) -> Self {
        let two = T::from_f64(2.0).unwrap();

        let levels = T::from_u64((1_u64 << bit) - 1).unwrap();
        let bit_step = levels / voltage_supply;

        Self {
            pose: Pose::new(position.into(), orientation),
            local_sensitive_axis: local_sensitive_axis.normalize(),
            sensitivity,
            quiescent_voltage: voltage_supply / two,
            min_voltage: T::zero(),
            max_voltage: voltage_supply,
            bit_step,
            offset,
        }
    }

    /// Compute the analog output voltage (V) in the presence of a magnetic source.
    pub fn get_voltage(&self, source: &impl Source<T>) -> T {
        // 1. Get the local field at the sensor's position
        let b_field = source.compute_B(self.pose.position());

        // 2. Compute the global sensitivity vector dynamically.
        let global_axis = self.pose.orientation() * self.local_sensitive_axis;
        let sensitivity_vector = global_axis * self.sensitivity;

        // 3. Delegate pure math to the measurement module
        linear_hall_voltage(
            b_field,
            sensitivity_vector,
            self.quiescent_voltage,
            self.min_voltage,
            self.max_voltage,
        )
    }

    /// Compute the continuous (unrounded) digital ADC reading.
    pub fn get_readings(&self, source: &impl Source<T>) -> T {
        (self.get_voltage(source) * self.bit_step) - self.offset
    }

    /// Compute the quantized (integer) digital ADC reading.
    pub fn get_readings_quantized(&self, source: &impl Source<T>) -> i64 {
        let continuous = self.get_readings(source);
        num_traits::Float::round(continuous).to_i64().unwrap_or(0)
    }
}
