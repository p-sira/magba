/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

/*!
# Magba
**Magba** is a performant analytical magnetic computation library for Rust.
Source available on [GitHub](https://github.com/p-sira/magba).

## Features
- Compute magnetic fields
- Create magnetic sources and group them as collections
- Move and rotate objects

## Quick Start
In this section, you will learn Magba in 5 minutes. To install,
simply: `cargo add magba`.

### Create sources and group as a collection
```
use magba::{CylinderMagnet, SourceCollection};
use nalgebra::{Point3, UnitQuaternion, Vector3};

let magnet1 = CylinderMagnet::new(
    Point3::origin(),
    UnitQuaternion::identity(),
    Vector3::new(0.0, 0.0, 0.9),
    0.005,
    0.020,
);

let magnet2 = CylinderMagnet::new(
    Point3::new(0.1, 0.0, 0.0),
    UnitQuaternion::identity(),
    Vector3::new(0.7, 0.5, 0.0),
    0.01,
    0.03,
);

let mut collection = SourceCollection::default();
collection.add(magnet1);
collection.add(magnet2);
```
Note that [SourceCollection] only holds single type of [Source].
If you need multiple types of Source, use [MultiSourceCollection] instead.

### Compute the field
The methods like [Field::get_B] are available for all [Source] and types that implement [Field].
```
use magba::sources::{CylinderMagnet, Field};
use nalgebra::{Point3, UnitQuaternion, Vector3};

let magnet = CylinderMagnet::new(
    Point3::origin(),
    UnitQuaternion::identity(),
    Vector3::new(0.0, 0.0, 0.9),
    0.005,
    0.020,
);

let points = [
    Point3::new(0.0, 0.0, 0.020),
    Point3::new(0.0, 0.0, 0.025),
    Point3::new(0.0, 0.0, 0.030),
];

let b_field = magnet.get_B(&points).expect("Fail to compute field"); // Don't forget to extract the Result
assert_eq!(
    b_field,
    [
        Vector3::new(0.0, 0.0, 0.041385029774502556),
        Vector3::new(0.0, 0.0, 0.018569788024793737),
        Vector3::new(0.0, 0.0, 0.00996091945575112),
    ]
);

use magba::conversion::*;
let h_field = Bs_to_Hs(&b_field);
assert_eq!(
    h_field,
    [
        Vector3::new(0.0, 0.0, 5.200596220326883e-8),
        Vector3::new(0.0, 0.0, 2.3335483854964688e-8),
        Vector3::new(0.0, 0.0, 1.2517260554074943e-8),
    ]
);
```

### Move and rotate objects
The transformation functionalities are available for all [Source] and types that implement [geometry::Transform].
```
use magba::{CylinderMagnet, Field};
use magba::geometry::Transform;
use nalgebra::{Point3, Translation3, UnitQuaternion, Vector3};

// Don't forget to make the object mutable if you need to move it.
let mut magnet = CylinderMagnet::new(
    Point3::origin(),
    UnitQuaternion::identity(),
    Vector3::new(0.0, 0.0, 0.9),
    0.005,
    0.020,
);

// Here, we try various ways to move the magnet closer to the observer.
let points = [Point3::new(0.0, 0.0, 0.05)];
assert_eq! (magnet.get_B(&points).unwrap(), [Vector3::new(0.0, 0.0, 0.0019205466890453442)]);

magnet.translate(&Translation3::new(0.0, 0.0, 0.01));
assert_eq! (magnet.get_B(&points).unwrap(), [Vector3::new(0.0, 0.0, 0.0038894698700304275)]);

magnet.set_position(Point3::new(0.0, 0.0, 0.02));
assert_eq! (magnet.get_B(&points).unwrap(), [Vector3::new(0.0, 0.0, 0.00996091945575112)]);

// Let's try rotating the magnet.
use std::f64::consts::PI;
magnet.rotate(&UnitQuaternion::from_scaled_axis(Vector3::new(PI / 4.0, 0.0, 0.0)));
assert_eq! (magnet.get_B(&points).unwrap(), [Vector3::new(3.940750052717413e-19, 0.0035238379945531466, 0.005577663229074168)]);

magnet.set_orientation(UnitQuaternion::from_scaled_axis(Vector3::new(PI / 2.0, 0.0, 0.0)));
assert_eq! (magnet.get_B(&points).unwrap(), [Vector3::new(6.086025172136602e-35, 0.003642460886175623, 0.0)]);
```

## Feature Flags
- `default`: enable `sources` and `parallel`
- `sources`: magnet structs and collection
- `parallel`: enable parallelization for performance

## Acknowledgment
Most of the field computation used in Magba is based on [MagpyLib](https://github.com/magpylib/magpylib).
We would like to thank MagpyLib contributors their hard work and contributions to the scientific community.
*/

mod special;
mod util;

pub mod constants;
pub mod fields;
pub mod geometry;

pub use fields::*;

#[cfg(feature = "sources")]
pub mod sources;
#[cfg(feature = "sources")]
pub use sources::*;

#[cfg(test)]
pub mod testing_util;
