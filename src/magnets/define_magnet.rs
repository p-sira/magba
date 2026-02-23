/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Function and macros shared by all magnetic sources

/// Generates a struct representing a magnetic source.
///
/// This macro handles the creation of the struct, constructor logic (including `Into` conversions),
/// input validation, getters/setters, and the implementation of the `Source` trait.
///
/// # Example
///
/// ```text
/// define_magnet!{
///     Magnet
///     field_fn: magnet_B
///     args: {
///         polarization:Vector3<T>,
///         dimensions:Vector3<T>; where dimensions.iter().all(|&i| i > 0); else "Bad dim.",
///         lucky_number: @val T
///     }
///     arg_display: "pol={}, dim={}, lucky={}";
///     arg_fmt: [format_vector3, format_vector3, format_float]
///     on_new: [
///         if lucky_number != T::from_f64(42.0).unwrap() { panic!() }
///     ]
/// }
/// ```
///
/// # Parameters
///
/// - **$name**: The name of the struct to generate.
/// - **$field_fn**: The function identifier for magnetic field (B) calculation.
///   - Signature must be: `(points, position, orientation, fields...)`.
/// - **$arg**: Name of a specific field/argument for the magnet (e.g., `polarization`, `length`).
///   - `@val`: If the optional keyword is placed before the type (e.g., `radius: @val f64`),
///     the argument is passed by value and strictly typed. If omitted, the argument uses `Into<T>`
///     generic conversion and is passed by reference to the calculation function.
/// - **$arg_type**: The concrete type of the argument.
/// - **$arg_default**: The default value (used in `Default::default()`).
/// - **where $validate:expr; else $error:literal**: Optional pattern for argument validation. If $validate is false, the macros will panic with $error.    
/// - **$arg_display**: A format string used in `std::fmt::Display` (e.g., `"pol={}, len={}"`).
/// - **$arg_fmt**: A list of macros used to format each argument (e.g., `format_vector3`).
/// - **$on_new**: Custom validation logic to run inside the `new()` constructor.
///
/// # Internal Pattern Matching
///
/// To handle the distinction between "pass-by-value" and "pass-by-reference/Into", the macro uses
/// internal helper rules. These allow recursive calls to process arguments differently based on
/// the presence of the `val` token.
///
/// - `@arg_into`: Handles conversion. If `val` is present, returns `$arg`. Otherwise, returns `$arg.into()`.
/// - `@arg_type_decl`: Handles signatures. If `val` is present, uses `$type`. Otherwise, uses `impl Into<$type>`.
/// - `@pass_arg`: Handles field function calls. If `val` is present, passes `$arg`. Otherwise, passes `&$arg`.
///
/// # Generated API
///
/// The macro generates:
/// 1. A struct with `position`, `orientation`, and the specified `$args`.
/// 2. Getters, setters, builders for all arguments. Includes input validation and automatic `Into` conversions for position/orientation and compatible args.
/// 3. `Debug`, `Clone`, `PartialEq`.
/// 4. `impl Default` using the provided `$arg_default` values.
/// 5. `impl crate::Source`, `impl crate::geometry::Transform`.
/// 6. `impl crate::Field` which maps `get_B` to the provided `$field_fn`.
/// 7. `impl std::fmt::Display` (if `no_std` is not active).
///
/// # Rationale
///
/// This macro serves as a consistent mean to implement magnet structs.
///
/// # Notes
///
/// The validation implementation can lead to Clippy's `neg_cmp_op_on_partial_ord` warning.
/// The behaviour is still valid, since NaN comparison will also return false.
///
macro_rules! define_magnet {

    // MARK: Helpers
    (@arg_into $arg:expr) => { $arg.into() };
    (@arg_into $arg:expr, val) => { $arg };
    (@arg_type_decl $arg_type:ty) => { impl Into<$arg_type> };
    (@arg_type_decl $arg_type:ty, val) => { $arg_type };
    (@pass_arg $arg:expr) => { &$arg };
    (@pass_arg $arg:expr, val) => { $arg };

    // MARK: Get, Set, With
    (@getters $struct_name:ident, $(($arg:ident, $arg_type:ty))*) => {
        impl<T: crate::base::Float> $struct_name<T> {
            $(
                pub fn $arg(&self) -> $arg_type {
                    self.$arg
                }
            )*
        }
    };

    (@setters $struct_name:ident, $(
        (
            $arg:ident,
            $arg_type:ty,
            [$(@$is_value:ident)?],
            [$($validate:expr)?],
            [$($error:literal)?]
        )
    )*) => {
        impl<T: crate::base::Float> $struct_name<T> {
            $(
                // Setters
                concat_idents::concat_idents!(fn_name = set_, $arg {
                    pub fn fn_name(&mut self, $arg: define_magnet!(@arg_type_decl $arg_type $(, $is_value)?)) {
                        let $arg: $arg_type = define_magnet!(@arg_into $arg $(, $is_value)?);
                        $(
                            if !($validate) {
                                panic!($error);
                            }
                        )?
                        self.$arg = $arg;
                    }
                });

                // Buliders (with setters)
                concat_idents::concat_idents!(fn_name = with_, $arg {
                    pub fn fn_name(mut self, $arg: define_magnet!(@arg_type_decl $arg_type $(, $is_value)?)) -> Self {
                        concat_idents::concat_idents!(ident = set_, $arg {
                            self.ident($arg);
                        });
                        self
                    }
                });
            )*
        }
    };

    // MARK: Main Entry
    {
        $(#[$meta:meta])*
        $name:ident
        field_fn: $field_fn:ident
        args: {
            $(
                $arg:ident : $(@$is_value:ident)? $arg_type:ty = $arg_default:expr
                $(; where $validate:expr; else $error:literal)?
            ),* $(,)?
        }
        arg_display: $arg_display:expr;
        arg_fmt: [ $($arg_fmt:ident),* $(,)? ]
        $(on_new: [ $($on_new:tt)* ])?
    } => {
        $(#[$meta])*
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        pub struct $name<T: crate::base::Float = f64> {
            pose: crate::geometry::Pose<T>,
            $(
                $arg: $arg_type,
            )*
        }

        define_magnet!(@getters $name, $(($arg, $arg_type))*);

        define_magnet!(@setters $name, $(
            (
                $arg,
                $arg_type,
                [$(@$is_value)?],
                [$($validate)?],
                [$($error)?]
            )
        )*);

        impl<T: crate::base::Float> $name<T> {
            pub fn new(
                position: impl Into<nalgebra::Point3<T>>,
                orientation: nalgebra::UnitQuaternion<T>,
                $(
                    $arg: define_magnet!(@arg_type_decl $arg_type $(, $is_value)?)
                ),*
            ) -> Self {
                let pose = crate::geometry::Pose::new(position.into(), orientation.into());

                $(
                    let $arg: $arg_type = define_magnet!(@arg_into $arg $(, $is_value)?);
                )*

                // Run validation logic after conversion
                $(
                    $(
                        if !($validate) {
                            panic!($error);
                        }
                    )?
                )*

                $($($on_new)*)?
                $name {
                    pose,
                    $($arg),*
                }
            }

            crate::geometry::pose::delegate_to_pose!();

            pub fn set_pose(&mut self, pose: crate::Pose<T>) {
                self.pose = pose;
            }

            pub fn with_position(mut self, position: impl Into<nalgebra::Point3<T>>) -> Self {
                self.set_position(position.into());
                self
            }

            pub fn with_orientation(mut self, orientation: nalgebra::UnitQuaternion<T>) -> Self {
                self.set_orientation(orientation);
                self
            }

            pub fn with_pose(mut self, pose: crate::Pose<T>) -> Self {
                self.set_pose(pose);
                self
            }
        }

        impl<T: crate::base::Float> Default for $name<T> {
            fn default() -> Self {
                Self {
                    pose: Default::default(),
                    $($arg: $arg_default),*
                }
            }
        }

        impl<T: crate::base::Float> crate::base::Source<T> for $name<T> {
            #[cfg(feature = "std")]
            fn format(&self, f: &mut std::fmt::Formatter<'_>, _: &str) -> std::fmt::Result {
                write!(
                    f,
                    concat!(
                        stringify!($name),
                        " (",
                        $arg_display,
                        ") at {}"
                    ),
                    $(crate::crate_util::$arg_fmt!(&self.$arg),)*
                    self.pose
                )
            }
        }

        crate::base::transform::impl_transform!($name<T> where T: crate::base::Float);

        impl<T: crate::base::Float> crate::base::Field<T> for $name<T> {
            fn get_B(&self, points: &[nalgebra::Point3<T>]) -> Vec<nalgebra::Vector3<T>> {
                crate::fields::$field_fn(
                    points,
                    &self.position(),
                    &self.orientation(),
                    $( define_magnet!(@pass_arg self.$arg $(, $is_value)?), )*
                )
            }
        }

        #[cfg(feature = "std")]
        impl<T: crate::base::Float> std::fmt::Display for $name<T> {
            fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                <Self as crate::base::Source<T>>::format(self, f, "")
            }
        }
    }
}
pub(crate) use define_magnet;
