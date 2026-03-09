/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use core::fmt::Display;

use getset::Getters;
use nalgebra::{Point3, UnitQuaternion, Vector3};

use crate::{
    base::{
        pose::impl_pose_methods, transform::impl_transform, Float, Observer, Pose, SensorOutput,
        Source,
    },
    measurement::hall_effect::hall_switch_state,
};

/// A physical representation of a unipolar Hall effect switch sensor.
///
/// Outputs a digital reading solely based on the magnetic operate point (B_OP) threshold.
/// It is completely stateless because it does not model hysteresis natively.
#[derive(Clone, Debug, PartialEq, Eq, Getters)]
#[getset(get = "pub")]
pub struct HallSwitch<T: Float = f64> {
    pose: Pose<T>,
    sensitive_axis: Vector3<T>,
    b_op: T,
}

impl_transform!(HallSwitch<T> where T: Float);

impl<T: Float> HallSwitch<T> {
    // MARK: New
    pub fn new(
        position: impl Into<Point3<T>>,
        orientation: UnitQuaternion<T>,
        sensitive_axis: impl Into<Vector3<T>>,
        b_op: T,
    ) -> Self {
        if b_op < T::zero() {
            panic!("B_OP must be non-negative.");
        }
        Self {
            pose: Pose::new(position.into(), orientation),
            sensitive_axis: sensitive_axis.into().normalize(),
            b_op,
        }
    }

    impl_pose_methods!();

    // MARK: Read

    /// Reads the state of the Hall switch based on the magnetic field in its vicinity.
    ///
    /// # Arguments
    ///
    /// - `source`: Magnetic [Source]
    ///
    /// # Returns
    ///
    /// - `true` if ON, `false` otherwise
    #[inline]
    pub fn read_state(&self, source: &dyn Source<T>) -> bool {
        let b_field = source.compute_B(self.pose.position());
        let global_sensitive_axis = self.pose.orientation() * self.sensitive_axis;

        hall_switch_state(b_field, global_sensitive_axis, self.b_op)
    }

    // MARK: Setters

    #[inline]
    pub fn set_sensitive_axis(&mut self, sensitive_axis: impl Into<Vector3<T>>) {
        self.sensitive_axis = sensitive_axis.into().normalize();
    }

    #[inline]
    pub fn set_b_op(&mut self, b_op: T) {
        if b_op < T::zero() {
            panic!("B_OP must be non-negative.");
        }
        self.b_op = b_op;
    }

    // MARK: With setters

    #[inline]
    pub fn with_sensitive_axis(mut self, sensitive_axis: impl Into<Vector3<T>>) -> Self {
        self.set_sensitive_axis(sensitive_axis);
        self
    }

    #[inline]
    pub fn with_b_op(mut self, b_op: T) -> Self {
        self.set_b_op(b_op);
        self
    }
}

impl<T: Float> Default for HallSwitch<T> {
    fn default() -> Self {
        Self {
            pose: Default::default(),
            sensitive_axis: Vector3::z(),
            b_op: T::from(0.010).unwrap(), // 10 mT
        }
    }
}

impl<T: Float> Observer<T> for HallSwitch<T> {
    /// Alias of [HallSwitch::read_state], returning `SensorOutput::Digital`.
    fn read(&self, source: &dyn Source<T>) -> SensorOutput<T> {
        let state = self.read_state(source);
        SensorOutput::Digital(if state { 1 } else { 0 })
    }

    // MARK: Display
    fn format(&self, f: &mut core::fmt::Formatter<'_>, _: &str) -> core::fmt::Result {
        write!(
            f,
            "HallSwitch (sensitive_axis=[{:?}, {:?}, {:?}], b_op={:?}) at {}",
            self.sensitive_axis.x,
            self.sensitive_axis.y,
            self.sensitive_axis.z,
            self.b_op,
            self.pose
        )
    }
}

impl<T: Float> Display for HallSwitch<T> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        <Self as Observer<T>>::format(self, f, "")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::magnets::StableFieldMagnet;
    use nalgebra::{UnitQuaternion, Vector3};

    #[test]
    fn test_hall_switch() {
        use crate::base::SensorOutput;

        let mut sensor = HallSwitch::default()
            .with_b_op(0.010)
            .with_sensitive_axis([0.0, 0.0, 1.0]);

        // Field below B_OP
        let source_off = StableFieldMagnet::new(Vector3::new(0.0, 0.0, 0.005));
        assert_eq!(sensor.read_state(&source_off), false);
        assert_eq!(sensor.read(&source_off), SensorOutput::Digital(0));

        // Field above B_OP
        let source_on = StableFieldMagnet::new(Vector3::new(0.0, 0.0, 0.015));
        assert_eq!(sensor.read_state(&source_on), true);
        assert_eq!(sensor.read(&source_on), SensorOutput::Digital(1));

        // Field pointing in wrong direction (perpendicular)
        let source_perp = StableFieldMagnet::new(Vector3::new(0.050, 0.0, 0.0));
        assert_eq!(sensor.read_state(&source_perp), false);

        // Field pointing in opposite direction
        let source_opp = StableFieldMagnet::new(Vector3::new(0.0, 0.0, -0.015));
        assert_eq!(sensor.read_state(&source_opp), false);

        // Sensor rotated 180 deg around X, sensitive axis is now -Z.
        // It points its Z-axis downwards.
        let rot_quat = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), core::f64::consts::PI);
        sensor.set_orientation(rot_quat);

        // Same source_on, but now sensor sensitivity is opposite, so perceived field is negative.
        assert_eq!(sensor.read_state(&source_on), false);

        // Source opposite should now trigger it
        assert_eq!(sensor.read_state(&source_opp), true);
    }
}
