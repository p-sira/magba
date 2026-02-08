/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

#![cfg_attr(feature = "no_std", no_std)]

/*!
# Magba
**Magba** is a performant analytical magnetic computation library for Rust.

In Magba, the **[sources]** submodule provides structs such as [CylinderMagnet] and
[Dipole]. These objects can [compute magnetic fields](#field-computation) and [transform](#move-and-rotate-objects).
The sources can be grouped into [**source collections**](#sources-and-collections).
This is the recommended way to interact with the API.

The submodule **[fields]** can be accessed to directly compute the fields. All physical
quantities are assumed to be in SI units. The submodule **[conversion]** provides functions
to conveniently convert between physical quantities.

The source code is available on [GitHub](https://github.com/p-sira/magba).

## Features
- Compute [magnetic fields](#field-computation) analytically for various source geometries.
- Create [magnetic sources](Source) and group them as [source collections](#sources-and-collections).
- [Move and rotate objects](#move-and-rotate-objects) in 3D space.
- Increase performance using [parallelization](#installation) with [Rayon](https://docs.rs/crate/rayon/latest).
- Support calculation with [f32] and [f64].
- Python bindings available via [Pymagba](https://github.com/p-sira/pymagba)

## Installation
To install Magba using `cargo`, simply run in your command line:
```bash
>> cargo add magba
```

By default, Magba installs with all stable features enabled. To install only the specified
feature flags, use:
```bash
>> cargo add magba --no-default-features --features sources,parallel
```
The available feature flags are:
- `default`: Enable `sources`, `parallel`, and `shorthands`.
- `transform`: Spatial transformation functionalities.
- `sources`: Magnetic source structs and collections. Also enable `transform`.
- `parallel`: Parallel computation using Rayon for performance.
- `shorthands`: Enable shorthands for quickly defining sources.
- `unstable`: Enable unstable features. These features may change any time.
*/

//! ## Sources and Collections
//!
#![cfg_attr(
    not(feature = "sources"),
    doc = "<div class=\"warning\">This functionality need `sources` feature flag.</div>"
)]
//!
//! The magnetic sources are located in the **[sources]** submodule. The parameters of the sources
//! can be accessed and modified using *getters* and *setters*, such as `position()` and `set_position()`.
//!
//! The sources can be grouped as *source collections*. For performance, there are two types
//! of source collections, the [SourceCollection] and [MultiSourceCollection]. **SourceCollection**
//! is faster, as it is stack-allocated. However, it can only hold the same type of [Source]. Meanwhile,
//! **MultiSourceCollection** is slower due to heap allocation but can holdheterogenous source types
//! encapsulated in [Box].
//!
//! ### Defining Sources and Grouping as MultiSourceCollection
#![cfg_attr(not(feature = "sources"), doc = "```ignore")]
#![cfg_attr(feature = "sources", doc = "```")]
//! use magba::*;
//! use nalgebra::*;
//!
//! let cylinder = Box::new(CylinderMagnet::default());
//! let cuboid = Box::new(CuboidMagnet::new(
//!     point![1.0, 0.0, 0.0],  // position (m)
//!     UnitQuaternion::identity(),  // orientation
//!     Vector3::z(),                // polarization (T)
//!     vector![0.1, 0.2, 0.3], // dimensions (m)
//! ));
//!
//! let mut collection = MultiSourceCollection::from_sources(vec![cylinder, cuboid]);
//! collection.add(Box::new(Dipole::default()));
//! ```
//!
//! ### Stack-allocated Collection
#![cfg_attr(not(feature = "sources"), doc = "```ignore")]
#![cfg_attr(feature = "sources", doc = "```")]
//! use magba::*;
//! use nalgebra::*;
//! use std::f64::consts::PI;
//!
//! let magnet1 = CylinderMagnet::new(
//!     point![0.0, 0.0, 1.0], // position (m)
//!     UnitQuaternion::identity(), // orientation
//!     Vector3::z(),               // polarization (T)
//!     0.1,                        // radius (m)
//!     0.2,                        // height (m)
//! );
//!
//! let mut magnet2 = magnet1.clone();
//! magnet2.rotate_anchor(
//!     &UnitQuaternion::from_scaled_axis(vector![PI, 0.0, 0.0]),
//!     &Point3::origin(),
//! );
//!
//! let collection = SourceCollection::from_sources(vec![magnet1, magnet2]);
//! ```
//!
//! ## Field Computation
//! The methods like [Field::get_B] are available for all [Source] and types that implement [Field],
//! including [SourceCollection] and [MultiSourceCollection].
//!
#![cfg_attr(
    not(feature = "sources"),
    doc = "<div class=\"warning\">This functionality need `sources` feature flag.</div>"
)]
//!
#![cfg_attr(not(feature = "sources"), doc = "```ignore")]
#![cfg_attr(feature = "sources", doc = "```")]
//! use magba::*;
//! use nalgebra::*;
//!
//! // A unit cylinder magnet (pol=(0,0,1), d=1, h=1) at (0,0,0), q=(0,0,0,1)
//! let mut magnet = CylinderMagnet::default();
//!
//! // Observer positions
//! let points = [
//!     point![0.0, 0.0, 0.020],
//!     point![0.0, 0.0, 0.025],
//!     point![0.0, 0.0, 0.030],
//! ];
//!
//! // Compute the magnetic field
//! let b_fields = magnet.get_B(&points);
//! let expected = [
//!         vector![0.0, 0.0, 0.7066824465457847],
//!         vector![0.0, 0.0, 0.706443696474588],
//!         vector![0.0, 0.0, 0.7061518306386746],
//! ];
//!
//! b_fields.iter().zip(expected).for_each(|(b, b_ref)| {assert_close_vec!(b, &b_ref, 1e-12);});
//!
//! use magba::conversion::*;
//! let h_field = Bs_to_Hs(&b_fields);
//! ```
//!
//! ## Move and Rotate Objects
//! The transformation functionalities are implemented for all [Source] and types with [Transform] trait.
//! The available methods are:
//! - [Transform::position] and [Transform::set_position]
//! - [Transform::orientation] and [Transform::set_orientation]
//! - [Transform::translate]
//! - [Transform::rotate] and [Transform::rotate_anchor]
//!
#![cfg_attr(
    not(feature = "sources"),
    doc = "<div class=\"warning\">This functionality need `sources` feature flag.</div>"
)]
//!
#![cfg_attr(not(feature = "sources"), doc = "```ignore")]
#![cfg_attr(feature = "sources", doc = "```")]
//! use magba::*;
//! use nalgebra::*;
//! use std::f64::consts::PI;
//!
//! // Define mutable object to allow transformations.
//! let mut magnet = CylinderMagnet::new(
//!     Point3::origin(),
//!     UnitQuaternion::identity(),
//!     vector![0.0, 0.0, 0.9],
//!     0.01,
//!     0.02,
//! );
//!
//! // Computing the magnetic field
//! let points = [point![0.0, 0.0, 0.05]];
//! let b = magnet.get_B(&points)[0];
//! let expected = vector![0.0, 0.0, 0.0019205466890453442];
//! assert_close_vec!(&b, &expected, 1e-12);
//!
//! // Moving the magnet
//! magnet.translate(&Translation3::new(0.0, 0.0, 0.01));
//! let b = magnet.get_B(&points)[0];
//! let expected = vector![0.0, 0.0, 0.0038894698700304275];
//! assert_close_vec!(&b, &expected, 1e-12);
//!
//! magnet.set_position(point![0.0, 0.0, 0.02]);
//! let b = magnet.get_B(&points)[0];
//! let expected = vector![0.0, 0.0, 0.00996091945575112];
//! assert_close_vec!(&b, &expected, 1e-12);
//!
//! // Rotating the magnet
//! magnet.rotate(&UnitQuaternion::from_scaled_axis(vector![PI / 4.0, 0.0, 0.0]));
//! let b = magnet.get_B(&points)[0];
//! let expected = vector![3.9407500527173422e-19, 0.0035238379945531874, 0.005577663229073966];
//! assert_close_vec!(&b, &expected, 1e-12);
//!
//! magnet.set_orientation(UnitQuaternion::from_scaled_axis(vector![PI / 2.0, 0.0, 0.0]));
//! let b = magnet.get_B(&points)[0];
//! let expected = vector![6.086025172136602e-35, 0.003642460886175623, 0.0];
//! assert_close_vec!(&b, &expected, 1e-12);
//! ```
//!
//! ## Direct Field Calculation
//! If you only need to access the field functions, you can use the [fields] submodule and disable
//! the `sources` feature by running `cargo add magba --no-default-features --features parallel`.
//! The core field computation functions, such as computation in local frame, are available under
//! the `unstable` feature flag.
//! ```
//! use magba::*;
//! use magba::fields;
//! use nalgebra::*;
//!
//! // Compute the magnetic field of a cylinder magnet.
//! let b = fields::cylinder_B(
//!     &[point![1.0, -1.0, 0.0]], // observer positions (m)
//!     &Point::origin(),               // magnet position (m)
//!     &UnitQuaternion::identity(),    // magnet orientation
//!     &vector![1.0, 2.0, 3.0],   // polarization (T)
//!     2.0,                            // diameter (m)
//!     2.0,                            // height (m)
//! )[0]; // Extract the element since the field function returns a vec of Vector3.
//! let expected = vector![-0.3684605662842379, -0.10171405289381347, -0.330064920993222];
//! assert_close_vec!(&b, &expected, 1e-12);
//! ```
//!
//! ## Acknowledgment
//! Most of the field computation used in Magba is based on [MagpyLib](https://github.com/magpylib/magpylib).
//! We would like to thank MagpyLib contributors their hard work and contributions to the scientific community.

#![cfg_attr(
    not(feature = "default"),
    allow(
        rustdoc::broken_intra_doc_links,
        rustdoc::private_intra_doc_links,
        unused,
    )
)]

#[cfg(all(feature = "no_std", feature = "default"))]
compile_error!(
    "The feature flag `no_std` is incompatible with `default`. Please add --no-default-features"
);

#[cfg(all(feature = "no_std", feature = "parallel"))]
compile_error!("The feature flag `parallel` is incompatible with `no_std`.");

macro_rules! compile_if_valid_feature_flags {
    ($($body:item)*) => {
        $(
            #[cfg(not(any(
            all(feature = "no_std", feature = "default"),
            all(feature = "no_std", feature = "parallel")
        )))]
        $body
        )*
    };
}

compile_if_valid_feature_flags! {
    mod crate_util;
    use crate::crate_util::need_feature;

    pub mod constants;
    pub mod conversion;
    pub mod fields;
    pub mod geometry;

    need_feature!{"shorthands", pub mod shorthands;}
    // need_feature!{"macros", mod macros;}

    use constants::MagneticConstants;

    /// Generic trait for floating point numbers compatible with all [Magba](crate) implementations.
    ///
    /// Supports [f32] and [f64].
    pub trait Float: nalgebra::RealField + num_traits::Float + MagneticConstants + Copy {}
    impl Float for f32 {}
    impl Float for f64 {}

    pub type StrErr = &'static str;

    need_feature!{"sources",
        pub mod sources;
        #[cfg(feature = "sources")]
        #[doc(inline)]
        pub use sources::*;
    }

    need_feature!{"transform",
        #[doc(inline)]
        pub use geometry::Transform;
    }

    #[cfg(test)]
    pub mod testing_util;

    #[cfg(feature = "no_std")]
    /// Size of fixed-size arrays used in no-allocation environments.
    const SIZE: usize = 1000;
}

/* #region: Public Macros */

/// Check if two vectors are close using relative Euclidean distance
#[macro_export]
macro_rules! assert_close_vec {
    ($a:expr, $b:expr, $rtol:expr) => {{
        use nalgebra::{Point3, distance};

        let dist = distance(&Point3::from($a.clone()), &Point3::from($b.clone()));

        let rel_a: f64 = dist / $a.magnitude();
        let rel_b: f64 = dist / $b.magnitude();
        let rel = rel_a.max(rel_b);
        if rel > $rtol {
            panic!(
                "Assertion failed: a={}, b={}, dist = {:e}, rel = {:e}, rtol = {:e}",
                $a, $b, dist, rel, $rtol
            );
        }
    }};
}
