use magba::sensors::hall_effect::{HallLatch, HallSwitch, LinearHallSensor};
use nalgebra::{Point3, UnitQuaternion, Vector3};
use std::panic::{AssertUnwindSafe, catch_unwind};

macro_rules! validation_tests {
    ($module:ident, $float:ty) => {
        mod $module {
            use super::*;
            type T = $float;

            fn switch(axis: Vector3<T>, b_op: T) -> HallSwitch<T> {
                HallSwitch::new(Point3::origin(), UnitQuaternion::identity(), axis, b_op)
            }
            fn latch(axis: Vector3<T>, b_op: T, b_rp: T) -> HallLatch<T> {
                HallLatch::new(
                    Point3::origin(),
                    UnitQuaternion::identity(),
                    axis,
                    b_op,
                    b_rp,
                )
            }

            #[test]
            fn invalid_axes_are_rejected_without_mutating_sensors() {
                for axis in [
                    Vector3::zeros(),
                    Vector3::new(T::NAN, 0.0, 1.0),
                    Vector3::new(0.0, T::INFINITY, 1.0),
                    Vector3::new(1.0, 0.0, T::NEG_INFINITY),
                    Vector3::repeat(T::MAX),
                    Vector3::repeat(T::MIN_POSITIVE),
                ] {
                    assert!(catch_unwind(|| switch(axis, 0.01)).is_err());
                    assert!(catch_unwind(|| latch(axis, 0.01, -0.01)).is_err());
                    let mut s = HallSwitch::<T>::default();
                    let before = s.clone();
                    assert!(catch_unwind(AssertUnwindSafe(|| s.set_sensitive_axis(axis))).is_err());
                    assert_eq!(s, before);
                    assert!(catch_unwind(|| s.with_sensitive_axis(axis)).is_err());
                    let mut l = HallLatch::<T>::default();
                    assert!(l.read_state(&magba::magnets::SphereMagnet::<T>::default()));
                    let before = l.clone();
                    assert!(catch_unwind(AssertUnwindSafe(|| l.set_sensitive_axis(axis))).is_err());
                    assert_eq!(l, before);
                    assert!(catch_unwind(|| l.with_sensitive_axis(axis)).is_err());
                }
            }

            #[test]
            fn invalid_thresholds_are_rejected_without_mutating_sensors() {
                for threshold in [T::NAN, T::INFINITY, T::NEG_INFINITY] {
                    assert!(catch_unwind(|| switch(Vector3::z(), threshold)).is_err());
                    assert!(catch_unwind(|| latch(Vector3::z(), threshold, -0.01)).is_err());
                    assert!(catch_unwind(|| latch(Vector3::z(), 0.01, threshold)).is_err());
                    let mut s = HallSwitch::<T>::default();
                    let before = s.clone();
                    assert!(catch_unwind(AssertUnwindSafe(|| s.set_b_op(threshold))).is_err());
                    assert_eq!(s, before);
                    assert!(catch_unwind(|| s.with_b_op(threshold)).is_err());
                    let mut l = HallLatch::<T>::default();
                    assert!(l.read_state(&magba::magnets::SphereMagnet::<T>::default()));
                    let before = l.clone();
                    assert!(catch_unwind(AssertUnwindSafe(|| l.set_b_op(threshold))).is_err());
                    assert_eq!(l, before);
                    assert!(catch_unwind(AssertUnwindSafe(|| l.set_b_rp(threshold))).is_err());
                    assert_eq!(l, before);
                    assert!(catch_unwind(|| l.clone().with_b_op(threshold)).is_err());
                    assert!(catch_unwind(|| l.with_b_rp(threshold)).is_err());
                }
                let mut s = HallSwitch::<T>::default();
                let before = s.clone();
                assert!(catch_unwind(AssertUnwindSafe(|| s.set_b_op(-0.01))).is_err());
                assert_eq!(s, before);
                let mut l = HallLatch::<T>::default();
                let before = l.clone();
                let op = *l.b_op();
                let rp = *l.b_rp();
                assert!(catch_unwind(AssertUnwindSafe(|| l.set_b_op(rp))).is_err());
                assert_eq!(l, before);
                assert!(catch_unwind(AssertUnwindSafe(|| l.set_b_rp(op))).is_err());
                assert_eq!(l, before);
                assert!(catch_unwind(|| switch(Vector3::z(), -0.01)).is_err());
                for (op, rp) in [(0.01, 0.01), (-0.01, 0.01)] {
                    assert!(catch_unwind(|| latch(Vector3::z(), op, rp)).is_err());
                }
            }

            #[test]
            fn linear_supply_voltage_validation_preserves_configuration() {
                for voltage in [T::NAN, T::INFINITY, T::NEG_INFINITY, 0.0, -1.0] {
                    assert!(
                        catch_unwind(|| LinearHallSensor::new(
                            Point3::<T>::origin(),
                            UnitQuaternion::identity(),
                            Vector3::z(),
                            20.0,
                            voltage
                        ))
                        .is_err()
                    );
                    let mut sensor = LinearHallSensor::<T>::default();
                    let before = sensor.clone();
                    assert!(
                        catch_unwind(AssertUnwindSafe(|| sensor.set_supply_voltage(voltage)))
                            .is_err()
                    );
                    assert_eq!(sensor, before);
                    assert!(catch_unwind(|| sensor.with_supply_voltage(voltage)).is_err());
                }
                for voltage in [T::MIN_POSITIVE, 3.3, T::MAX] {
                    let mut sensor = LinearHallSensor::new(
                        Point3::<T>::origin(),
                        UnitQuaternion::identity(),
                        Vector3::z(),
                        20.0,
                        voltage,
                    );
                    assert_eq!(sensor.supply_voltage(), voltage);
                    sensor.set_supply_voltage(5.0);
                    assert_eq!(sensor.supply_voltage(), 5.0);
                    assert_eq!(*sensor.max_voltage(), 5.0);
                    assert_eq!(*sensor.quiescent_voltage(), 2.5);
                }
            }

            #[test]
            fn valid_axes_normalize_and_threshold_boundaries_remain_supported() {
                let axis = Vector3::new(0.0, 0.0, -4.0);
                let mut s = switch(axis, 0.0);
                assert_eq!(*s.sensitive_axis(), -Vector3::z());
                s.set_sensitive_axis(Vector3::new(3.0, 0.0, 0.0));
                assert_eq!(*s.sensitive_axis(), Vector3::x());
                s.set_b_op(T::MAX);
                assert_eq!(*s.b_op(), T::MAX);
                // Latch thresholds may both be negative or both be positive.
                for (op, rp) in [(-0.01, -0.02), (0.02, 0.01), (T::MAX, -T::MAX)] {
                    let mut l = latch(axis, op, rp);
                    assert_eq!(*l.sensitive_axis(), -Vector3::z());
                    l.set_sensitive_axis(Vector3::new(3.0, 0.0, 0.0));
                    assert_eq!(*l.sensitive_axis(), Vector3::x());
                    l.set_b_op(op);
                    l.set_b_rp(rp);
                }
            }
        }
    };
}
validation_tests!(f32_tests, f32);
validation_tests!(f64_tests, f64);
