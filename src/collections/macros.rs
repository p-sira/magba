/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

/// Constructs a source composite.
///
/// # Examples
///
/// Declaring a source assembly:
/// ```
/// use magba::sources;
/// use magba::prelude::*;
/// # let cylinder = CylinderMagnet::default();
/// # let cuboid = CuboidMagnet::default();
/// # let dipole = Dipole::default();
/// # let dipole2 = Dipole::default().with_moment([1.0, 0.0, 0.0]);
/// // For assemblies, we use comma-separated arguments.
/// let assembly: SourceAssembly = sources!(cylinder, cuboid, dipole);
/// ```
///
/// Declaring a homogenous source array:
/// ```
/// use magba::sources;
/// use magba::prelude::*;
/// # let dipole = Dipole::default();
/// # let dipole2 = Dipole::default().with_moment([1.0, 0.0, 0.0]);
/// // For arrays, we use bracket syntax.
/// let homogenous_array: SourceArray<Dipole, _> = sources!([dipole, dipole2]);
/// ```
///
/// Declaring a heterogenous source array:
/// ```
/// use magba::sources;
/// use magba::prelude::*;
/// # let cylinder = CylinderMagnet::default();
/// # let cuboid = CuboidMagnet::default();
/// # let dipole = Dipole::default();
/// let heterogenous_array: SourceArray<Magnet, _> =
///     sources!([cylinder.into(), cuboid.into(), dipole.into()]);
/// ```
#[macro_export]
macro_rules! sources {
    // sources!([magnet1, magnet2, ...])
    ([$($items:expr),*]) => {
        SourceArray::new([0.0; 3], nalgebra::UnitQuaternion::identity(), [$($items),*])
    };
    // sources!(magnet1, magnet2, ...)
    ($($items:expr),* $(,)?) => {{
        let c: [SourceComponent<_>; _] = [$($items.into()),*];
        SourceAssembly::from(c)
    }};
    () => { SourceAssembly::default() };
}
#[cfg(test)]
pub(crate) use sources;

/// Constructs an observer composite.
///
/// # Examples
///
/// Declaring a sensor assembly:
/// ```
/// use magba::observers;
/// use magba::prelude::*;
/// # use magba::sensors::hall_effect::*;
/// # let lin_hall = LinearHallSensor::default();
/// # let hall_switch = HallSwitch::default();
/// # let hall_latch = HallLatch::default();
/// // For assemblies, we use comma-separated arguments.
/// let assembly: ObserverAssembly = observers!(lin_hall, hall_switch, hall_latch);
/// ```
///
/// Declaring a homogenous observer array:
/// ```
/// use magba::observers;
/// use magba::prelude::*;
/// # use magba::sensors::hall_effect::*;
/// # let lin_hall = LinearHallSensor::default();
/// # let lin_hall2 = LinearHallSensor::default().with_position([0.0, 1.0, 0.0]);
/// // For arrays, we use bracket syntax.
/// let homogenous_array: ObserverArray<LinearHallSensor, _> = observers!([lin_hall, lin_hall2]);
/// ```
///
/// Declaring a heterogenous observer array:
/// ```
/// use magba::observers;
/// use magba::prelude::*;
/// # use magba::sensors::hall_effect::*;
/// # let lin_hall = LinearHallSensor::default();
/// # let hall_switch = HallSwitch::default();
/// # let hall_latch = HallLatch::default();
/// let heterogenous_array: ObserverArray<ObserverComponent, _> =
///     observers!([lin_hall.into(), hall_switch.into(), hall_latch.into()]);
/// ```
#[macro_export]
macro_rules! observers {
    // observers!([sensor1, sensor2, ...])
    ([$($items:expr),*]) => {
        ObserverArray::new([0.0; 3], nalgebra::UnitQuaternion::identity(), [$($items),*])
    };
    // observers!(sensor1, sensor2, ...)
    ($($items:expr),* $(,)?) => {{
        let c: [ObserverComponent<_>; _] = [$($items.into()),*];
        ObserverAssembly::from(c)
    }};
    () => {};
}
#[cfg(test)]
pub(crate) use observers;
