/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

#[cfg(feature = "std")]
use derive_more::Display;
use enum_dispatch::enum_dispatch;

use crate::{base::Float, sensors::hall_effect::LinearHallSensor};

/// Sensor variants
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "std", derive(Display))]
#[enum_dispatch(Observer<T>, Transform<T>, Field<T>)]
pub enum Sensor<T: Float = f64> {
    LinearHall(LinearHallSensor<T>),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[cfg(feature = "std")]
    #[test]
    fn test_display() {
        let sensor: Sensor = LinearHallSensor::default().into();
        assert_eq!(
            format!("{}", sensor),
            "LinearHallSensor (sensitive_axis=[0.0, 0.0, 1.0], sensitivity=1.0, supply_voltage=5.0) at pos=[0.0, 0.0, 0.0], rot=[0.0, 0.0, 0.0]"
        );
    }
}
