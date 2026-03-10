/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use core::fmt::Display;
use core::sync::atomic::{AtomicBool, Ordering};

use getset::Getters;
use nalgebra::{Point3, UnitQuaternion, Vector3};

use crate::{
    base::{
        Float, Observer, Pose, SensorOutput, Source, pose::impl_pose_methods,
        transform::impl_transform,
    },
    measurement::hall_effect::hall_latch_state,
};

/// A physical representation of a Hall effect latch sensor.
///
/// Outputs a digital reading based on the magnetic operate point (B_OP)
/// and release point (B_RP) thresholds. Provides hysteresis by maintaining its internal state.
#[derive(Debug, Getters)]
#[getset(get = "pub")]
pub struct HallLatch<T: Float = f64> {
    pose: Pose<T>,
    sensitive_axis: Vector3<T>,
    b_op: T,
    b_rp: T,
    state: AtomicBool,
}

impl_transform!(HallLatch<T> where T: Float);

impl<T: Float> Clone for HallLatch<T> {
    fn clone(&self) -> Self {
        Self {
            pose: self.pose,
            sensitive_axis: self.sensitive_axis,
            b_op: self.b_op,
            b_rp: self.b_rp,
            state: AtomicBool::new(self.state.load(Ordering::SeqCst)),
        }
    }
}

impl<T: Float> PartialEq for HallLatch<T> {
    fn eq(&self, other: &Self) -> bool {
        self.pose == other.pose
            && self.sensitive_axis == other.sensitive_axis
            && self.b_op == other.b_op
            && self.b_rp == other.b_rp
            && self.state.load(Ordering::SeqCst) == other.state.load(Ordering::SeqCst)
    }
}

impl<T: Float> Eq for HallLatch<T> {}

impl<T: Float> HallLatch<T> {
    // MARK: New
    pub fn new(
        position: impl Into<Point3<T>>,
        orientation: UnitQuaternion<T>,
        sensitive_axis: impl Into<Vector3<T>>,
        b_op: T,
        b_rp: T,
    ) -> Self {
        if b_op <= b_rp {
            panic!("B_OP must be greater than B_RP.");
        }
        Self {
            pose: Pose::new(position.into(), orientation),
            sensitive_axis: sensitive_axis.into().normalize(),
            b_op,
            b_rp,
            state: AtomicBool::new(false),
        }
    }

    impl_pose_methods!();

    // MARK: Read

    /// Reads the state of the Hall latch based on the magnetic field in its vicinity.
    ///
    /// The latch updates and inherently remembers its internal representation over time to simulate hysteresis.
    ///
    /// # Arguments
    ///
    /// - `source`: Magnetic [Source]
    ///
    /// # Returns
    ///
    /// - If the projected field exceeds `b_op`, the state becomes `true` (Active).
    /// - If the projected field falls below `b_rp`, the state becomes `false` (Inactive).
    /// - If the field is between the two, the `current_state` is maintained.
    #[inline]
    pub fn read_state(&self, source: &dyn Source<T>) -> bool {
        let b_field = source.compute_B(self.pose.position());
        let global_sensitive_axis = self.pose.orientation() * self.sensitive_axis;

        let current_state = self.state.load(Ordering::SeqCst);

        let new_state = hall_latch_state(
            b_field,
            global_sensitive_axis,
            self.b_op,
            self.b_rp,
            current_state,
        );

        if new_state != current_state {
            self.state.store(new_state, Ordering::SeqCst);
        }

        new_state
    }

    // MARK: Setters

    #[inline]
    pub fn set_sensitive_axis(&mut self, sensitive_axis: impl Into<Vector3<T>>) {
        self.sensitive_axis = sensitive_axis.into().normalize();
    }

    #[inline]
    pub fn set_b_op(&mut self, b_op: T) {
        if b_op <= self.b_rp {
            panic!("B_OP must be greater than B_RP.");
        }
        self.b_op = b_op;
    }

    #[inline]
    pub fn set_b_rp(&mut self, b_rp: T) {
        if self.b_op <= b_rp {
            panic!("B_OP must be greater than B_RP.");
        }
        self.b_rp = b_rp;
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

    #[inline]
    pub fn with_b_rp(mut self, b_rp: T) -> Self {
        self.set_b_rp(b_rp);
        self
    }
}

impl<T: Float> Default for HallLatch<T> {
    fn default() -> Self {
        Self {
            pose: Default::default(),
            sensitive_axis: Vector3::z(),
            b_op: T::zero(),
            b_rp: T::zero(),
            state: AtomicBool::new(false),
        }
    }
}

impl<T: Float> Observer<T> for HallLatch<T> {
    /// Alias of [HallLatch::read_state], returning `SensorOutput::Digital`.
    fn read(&self, source: &dyn Source<T>) -> SensorOutput<T> {
        let state = self.read_state(source);
        SensorOutput::Digital(if state { 1 } else { 0 })
    }

    // MARK: Display
    fn format(&self, f: &mut core::fmt::Formatter<'_>, _: &str) -> core::fmt::Result {
        write!(
            f,
            "HallLatch (sensitive_axis=[{:?}, {:?}, {:?}], b_op={:?}, b_rp={:?}) at {}",
            self.sensitive_axis.x,
            self.sensitive_axis.y,
            self.sensitive_axis.z,
            self.b_op,
            self.b_rp,
            self.pose
        )
    }
}

impl<T: Float> Display for HallLatch<T> {
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
    fn test_hall_latch() {
        use crate::base::SensorOutput;

        let mut sensor = HallLatch::default()
            .with_b_op(0.010)
            .with_b_rp(-0.010)
            .with_sensitive_axis([0.0, 0.0, 1.0]);

        // Initially state is false (0)
        let source_zero = StableFieldMagnet::new(Vector3::new(0.0, 0.0, 0.0));
        assert_eq!(sensor.read_state(&source_zero), false);

        // Apply field below B_OP but above B_RP
        let source_weak = StableFieldMagnet::new(Vector3::new(0.0, 0.0, 0.005));
        assert_eq!(sensor.read_state(&source_weak), false);

        // Apply field above B_OP -> state becomes true
        let source_op = StableFieldMagnet::new(Vector3::new(0.0, 0.0, 0.015));
        assert_eq!(sensor.read_state(&source_op), true);

        // Reduce field below B_OP but above B_RP -> state remains true (hysteresis)
        assert_eq!(sensor.read_state(&source_weak), true);
        assert_eq!(sensor.read(&source_weak), SensorOutput::Digital(1));

        // Apply field below B_RP -> state becomes false
        let source_rp = StableFieldMagnet::new(Vector3::new(0.0, 0.0, -0.015));
        assert_eq!(sensor.read_state(&source_rp), false);

        // Increase field back to weak -> state remains false
        assert_eq!(sensor.read_state(&source_weak), false);
        assert_eq!(sensor.read(&source_weak), SensorOutput::Digital(0));

        // Sensor rotated 180 deg around X, sensitive axis is now -Z.
        let rot_quat = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), core::f64::consts::PI);
        sensor.set_orientation(rot_quat);

        // source_rp is -15mT in global Z. Sensor Z is -Z global.
        // The projected field is (-0.015) * (-1) = 0.015.
        // This is above B_OP, so it should turn ON.
        assert_eq!(sensor.read_state(&source_rp), true);
    }

    #[test]
    #[should_panic]
    fn test_input_validation() {
        let _ = HallLatch::new(
            [0.0; 3],
            UnitQuaternion::identity(),
            [0.0, 0.0, 1.0],
            0.0,
            0.0,
        );
    }

    #[test]
    #[should_panic]
    fn test_set_b_op_validation() {
        let mut sensor = HallLatch::default().with_b_op(0.010).with_b_rp(-0.010);
        sensor.set_b_op(-0.020);
    }

    #[test]
    #[should_panic]
    fn test_set_b_rp_validation() {
        let mut sensor = HallLatch::default().with_b_op(0.010).with_b_rp(-0.010);
        sensor.set_b_rp(0.020);
    }
}
