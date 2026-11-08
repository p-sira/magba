/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use enum_dispatch::enum_dispatch;

use crate::{
    base::{Float, Observer, Pose, SensorOutput, Source, Transform},
    crate_utils::display_enum,
    sensors::hall_effect::{HallLatch, HallSwitch, LinearHallSensor},
};

display_enum! {
    /// Sensor variants
    #[derive(Clone, Debug, PartialEq, Eq)]
    #[enum_dispatch(Observer<T>, Transform<T>)]
    pub enum Sensor<T: Float = f64> {
        LinearHall(LinearHallSensor<T>),
        HallSwitch(HallSwitch<T>),
        HallLatch(HallLatch<T>),
    }
}

#[cfg(all(test, feature = "std"))]
mod tests {
    use super::*;

    #[test]
    fn test_display() {
        let sensor: Sensor = LinearHallSensor::default().into();
        assert_eq!(
            format!("{}", sensor),
            "LinearHallSensor (sensitive_axis=[0.0, 0.0, 1.0], sensitivity=1.0, supply_voltage=5.0) at pos=[0.0, 0.0, 0.0], rot=[0.0, 0.0, 0.0]"
        );
    }
}
