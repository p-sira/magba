/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use enum_dispatch::enum_dispatch;

use crate::{
    base::{Float, Observer, Pose, SensorOutput, Source, Transform},
    sensors::{Sensor, hall_effect::LinearHallSensor},
};

#[derive(Debug, Clone)]
#[enum_dispatch(Observer<T>, Transform<T>)]
/// Components that can be grouped into collections.
///
/// ```
/// # use magba::sensors;
/// # use magba::prelude::*;
/// # use magba::sensors::LinearHallSensor;
/// let sensor: SensorComponent = LinearHallSensor::default().into();
/// let sensors = observers!(sensor);
/// ```
pub enum SensorComponent<T: Float = f64> {
    Sensor(Sensor<T>),
    Custom(Box<dyn Observer<T>>),
}

impl<T: Float> PartialEq for SensorComponent<T> {
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (Self::Sensor(l0), Self::Sensor(r0)) => l0 == r0,
            (Self::Custom(_), Self::Custom(_)) => false,
            _ => false,
        }
    }
}

macro_rules! impl_transitive_from {
    ($($primitive:ident),*) => {
        $(
            impl<T: Float> From<$primitive<T>> for SensorComponent<T> {
                fn from(p: $primitive<T>) -> Self {
                    let intermediate: Sensor<T> = p.into();
                    intermediate.into()
                }
            }
        )*
    };
}

impl_transitive_from!(LinearHallSensor);

impl<T: Float> Eq for SensorComponent<T> {}
