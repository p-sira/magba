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
///     /// Struct docs goes here
///     Magnet
///     field_fn: magnet_B
///     args: {
///         polarization:Vector3<T>,
///         dimensions:Vector3<T>;
///             validate dimensions.iter().all(|&i| i > 0);
///             error "Bad dim.",
///         lucky_number: T
///     }
///     arg_display: "pol={}, dim={}, lucky={}";
///     arg_fmt: [format_vector3, format_vector3, format_float]
///     docs: {
///         new: {
///             /// new() docs goes here
///         }
///     }
/// }
/// ```
///
/// # Parameters
///
/// - **$name**: The name of the struct to generate.
/// - **$field_fn**: The function identifier for magnetic field (B) calculation.
///   - Signature must be: `(points, position, orientation, fields...)`.
/// - **$arg**: Name of a specific field/argument for the magnet (e.g., `polarization`, `length`).
/// - **$arg_type**: The concrete type of the argument.
/// - **$arg_default**: The default value (used in `Default::default()`).
/// - **where $validate:expr; else $error:literal**: Optional pattern for argument validation. If $validate is false, the macros will panic with $error.    
/// - **$arg_display**: A format string used in `std::fmt::Display` (e.g., `"pol={}, len={}"`).
/// - **$arg_fmt**: A list of macros used to format each argument (e.g., `format_vector3`).
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
/// 6. `impl crate::Field` which maps `compute_B` to the provided `$field_fn`.
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
    (@pass_arg $arg:expr) => { $arg };

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
                            #[allow(clippy::neg_cmp_op_on_partial_ord)]
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
                $(; validate $validate:expr; error $error:literal)?
            ),* $(,)?
        }
        arg_display: $arg_display:expr;
        arg_fmt: [ $($arg_fmt:ident),* $(,)? ]

        docs: {
            new: { $(#[$new_docs:meta])* }
        }
    } => {
        $(#[$meta])*
        #[derive(Debug, Clone, PartialEq, Eq)]
        pub struct $name<T: crate::base::Float = f64> {
            pose: crate::base::Pose<T>,
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
            // MARK: New
            $(#[$new_docs])*
            pub fn new(
                position: impl Into<nalgebra::Point3<T>>,
                orientation: nalgebra::UnitQuaternion<T>,
                $(
                    $arg: define_magnet!(@arg_type_decl $arg_type $(, $is_value)?)
                ),*
            ) -> Self {
                let pose = crate::base::Pose::new(position.into(), orientation.into());

                $(
                    let $arg: $arg_type = define_magnet!(@arg_into $arg $(, $is_value)?);
                )*

                // Run validation logic after conversion
                $(
                    $(
                        #[allow(clippy::neg_cmp_op_on_partial_ord)]
                        if !($validate) {
                            panic!($error);
                        }
                    )?
                )*

                $name {
                    pose,
                    $($arg),*
                }
            }

            crate::base::pose::impl_pose_methods!();
        }

        impl<T: crate::base::Float> Default for $name<T> {
            fn default() -> Self {
                Self {
                    pose: Default::default(),
                    $($arg: $arg_default),*
                }
            }
        }

        // MARK: Field
        impl<T: crate::base::Float> crate::base::Source<T> for $name<T> {
            fn compute_B(&self, point: nalgebra::Point3<T>) -> nalgebra::Vector3<T> {
                crate::fields::$field_fn(
                    point,
                    self.position(),
                    self.orientation(),
                    $( define_magnet!(@pass_arg self.$arg $(, $is_value)?), )*
                )
            }

            #[cfg(feature = "alloc")]
            fn compute_B_batch(&self, points: &[nalgebra::Point3<T>]) -> alloc::vec::Vec<nalgebra::Vector3<T>> {
                let mut out = alloc::vec![nalgebra::Vector3::zeros(); points.len()];

                concat_idents::concat_idents!(fn_name = $field_fn, _batch {
                    crate::fields::fn_name(
                        points,
                        self.position(),
                        self.orientation(),
                        $( define_magnet!(@pass_arg self.$arg $(, $is_value)?), )*
                        &mut out,
                    );
                });

                out
            }

            // MARK: Display
            #[cfg(feature = "alloc")]
            fn format(&self, f: &mut core::fmt::Formatter<'_>, _: &str) -> core::fmt::Result {
                $(
                    let $arg = crate::crate_util::$arg_fmt(&mut *f, self.$arg);
                )*

                write!(
                    f,
                    concat!(
                        stringify!($name),
                        " (",
                        $arg_display,
                        ") at {}"
                    ),
                    $( $arg, )*
                    self.pose
                )
            }
        }

        crate::base::transform::impl_transform!($name<T> where T: crate::base::Float);

        impl<T: crate::base::Float> core::fmt::Display for $name<T> {
            fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                <Self as crate::base::Source<T>>::format(self, f, "")
            }
        }
    }
}
pub(crate) use define_magnet;
