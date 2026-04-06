/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Internal utilities for Magba

macro_rules! need_std {
    ($($body:item)*) => {
        $(
            #[cfg(feature = "std")]
            $body
        )*
    };
}
pub(crate) use need_std;

macro_rules! need_alloc {
    ($($body:item)*) => {
        $(
            #[cfg(feature = "alloc")]
            $body
        )*
    };
}
pub(crate) use need_alloc;

macro_rules! need_unstable {
    ($($body:item)*) => {
            #[cfg(feature = "unstable")]
            mod unstable {
                use super::*;
                $(
                    #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
                    $body
                )*
            }
            #[cfg(feature = "unstable")]
            pub use unstable::*;
    };
}
pub(crate) use need_unstable;

need_alloc! {
    use crate::base::Float;
    use core::fmt::Formatter;
    use nalgebra::Vector3;

    // MARK: Formatters
    pub(crate) fn format_float<T: Float>(f: &mut Formatter, v: T) -> alloc::string::String {
        if let Some(p) = f.precision() {
            alloc::format!("{:.p$}", v, p = p)
        } else {
            alloc::format!("{:?}", v)
        }
    }

    pub(crate) fn format_vector3<T: Float>(f: &mut Formatter, v: Vector3<T>) -> alloc::string::String {
        if let Some(p) = f.precision() {
            alloc::format!("[{:.p$}, {:.p$}, {:.p$}]", v.x, v.y, v.z, p = p)
        } else {
            alloc::format!("[{:?}, {:?}, {:?}]", v.x, v.y, v.z)
        }
    }

    pub(crate) fn format_vertices<T: Float>(f: &mut Formatter, v: [Vector3<T>; 3]) -> alloc::string::String {
        if let Some(p) = f.precision() {
            alloc::format!("[({:.p$}, {:.p$}, {:.p$}), ({:.p$}, {:.p$}, {:.p$}), ({:.p$}, {:.p$}, {:.p$})]", v[0].x, v[0].y, v[0].z, v[1].x, v[1].y, v[1].z, v[2].x, v[2].y, v[2].z, p = p)
        } else {
            alloc::format!("[({:?}, {:?}, {:?}), ({:?}, {:?}, {:?}), ({:?}, {:?}, {:?})]", v[0].x, v[0].y, v[0].z, v[1].x, v[1].y, v[1].z, v[2].x, v[2].y, v[2].z)
        }
    }

    pub(crate) fn format_vertices_4<T: Float>(f: &mut Formatter, v: [Vector3<T>; 4]) -> alloc::string::String {
        if let Some(p) = f.precision() {
            alloc::format!("[({:.p$}, {:.p$}, {:.p$}), ({:.p$}, {:.p$}, {:.p$}), ({:.p$}, {:.p$}, {:.p$}), ({:.p$}, {:.p$}, {:.p$})]", v[0].x, v[0].y, v[0].z, v[1].x, v[1].y, v[1].z, v[2].x, v[2].y, v[2].z, v[3].x, v[3].y, v[3].z, p = p)
        } else {
            alloc::format!("[({:?}, {:?}, {:?}), ({:?}, {:?}, {:?}), ({:?}, {:?}, {:?}), ({:?}, {:?}, {:?})]", v[0].x, v[0].y, v[0].z, v[1].x, v[1].y, v[1].z, v[2].x, v[2].y, v[2].z, v[3].x, v[3].y, v[3].z)
        }
    }
}

#[cfg(feature = "mesh")]
pub(crate) fn format_trimesh_count<T: Float>(
    _f: &mut Formatter,
    mesh: &crate::base::mesh::TriMesh<T>,
) -> alloc::string::String {
    alloc::format!("{}", mesh.triangles.len())
}

macro_rules! assert_eq_lens {
    ($str_err:expr, [$ref_vec:expr $(, $vec:expr)+]) => {
        {
            let len = $ref_vec.len();
            $(
                if $vec.len() != len {
                    panic!($str_err)
                }
            )+
        }
    };
}
pub(crate) use assert_eq_lens;

macro_rules! impl_parallel {
    ($func:ident, rayon_threshold: $threshold:expr, input: $inputs:expr, output: $out:expr, args: [$($func_args:expr),* $(,)?]) => {
        {
            assert_eq!($out.len(), $inputs.len(), "Output slice length must match input vectors length.");

            #[cfg(feature = "rayon")]
            {
                if $inputs.len() > $threshold {
                    use rayon::prelude::*;
                    $out.par_iter_mut()
                        .zip($inputs.par_iter())
                        .for_each(|(o, p)| *o = $func(*p, $($func_args),*));
                } else {
                    $out.iter_mut()
                        .zip($inputs.iter())
                        .for_each(|(o, p)| *o = $func(*p, $($func_args),*));
                }
            }

            #[cfg(not(feature = "rayon"))]
            {
                $out.iter_mut()
                    .zip($inputs.iter())
                    .for_each(|(o, p)| *o = $func(*p, $($func_args),*));
            }
        }
    };
}
pub(crate) use impl_parallel;

// TODO: compare par_bridge with zip -> par_iter
macro_rules! impl_parallel_sum {
    ($out:expr, $points:expr, $threshold:expr, [$($vecs:expr),+], |$p:ident, $($args:ident),*| $calc:expr) => {{
        assert_eq!($out.len(), $points.len(), "Output slice length must match points length.");
        crate::crate_utils::assert_eq_lens!(
            "Lengths of input vectors must be equal.",
            [$($vecs),+]
        );

        #[cfg(feature = "rayon")]
        {
            use rayon::prelude::*;

            $out.par_iter_mut()
                .zip($points.par_iter())
                .for_each(|(o, p_ref)| {
                    let $p = p_ref;
                    *o = itertools::izip!($($vecs),+)
                        .fold(nalgebra::Vector3::zeros(), |acc, ($($args),*)| {
                            acc + $calc
                        });
                });
        }

        #[cfg(not(feature = "rayon"))]
        $out.iter_mut()
            .zip($points.iter())
            .for_each(|(o, p_ref)| {
                let $p = p_ref;
                *o = itertools::izip!($($vecs),+)
                    .fold(nalgebra::Vector3::zeros(), |acc, ($($args),*)| {
                        acc + $calc
                    });
            });
    }};
}
pub(crate) use impl_parallel_sum;

// MARK: define_source!

/// Generates a struct representing a magnetic source.
///
/// This macro handles the creation of the struct, constructor logic (including `Into` conversions),
/// input validation, getters/setters, and the implementation of the `Source` trait.
///
/// # Example
///
/// ```text
/// define_source!{
///     /// Struct docs goes here
///     MySource
///     field_fn: source_B
///     args: {
///         polarization:Vector3<T>,
///         dimensions:Vector3<T>;
///             validate dimensions.iter().all(|&i| i > 0);
///             error "Bad dim.",
///         lucky_number: T
///     }
///     arg_display: "pol={}, dim={}, lucky={}";
///     arg_fmt: [format_vector3, format_float]
///     docs: {
///         new: {
///             /// new() docs goes here
///         }
///     }
/// }
/// ```
macro_rules! define_source {

    // MARK: Helpers
    (@arg_into $arg:expr) => { $arg.into() };
    (@arg_into $arg:expr, val) => { $arg };
    (@arg_into $arg:expr, ref) => { $arg };
    (@arg_type_decl $arg_type:ty) => { impl Into<$arg_type> };
    (@arg_type_decl $arg_type:ty, val) => { $arg_type };
    (@arg_type_decl $arg_type:ty, ref) => { $arg_type };
    (@pass_arg $arg:expr) => { $arg };
    (@pass_arg $arg:expr, val) => { $arg };
    (@pass_arg $arg:expr, ref) => { &$arg };

    // MARK: Get, Set, With
    (@getter_ret_type $arg_type:ty) => { $arg_type };
    (@getter_ret_type $arg_type:ty, val) => { $arg_type };
    (@getter_ret_type $arg_type:ty, ref) => { &$arg_type };
    (@getter_body $self:ident, $arg:ident) => { $self.$arg };
    (@getter_body $self:ident, $arg:ident, val) => { $self.$arg };
    (@getter_body $self:ident, $arg:ident, ref) => { &$self.$arg };

    (@getters $struct_name:ident, $(($arg:ident, $arg_type:ty, [$(@$is_value:ident)?]))*) => {
        impl<T: crate::base::Float> $struct_name<T> {
            $(
                pub fn $arg(&self) -> $crate::crate_utils::define_source!(@getter_ret_type $arg_type $(, $is_value)?) {
                    $crate::crate_utils::define_source!(@getter_body self, $arg $(, $is_value)?)
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
                    pub fn fn_name(&mut self, $arg: $crate::crate_utils::define_source!(@arg_type_decl $arg_type $(, $is_value)?)) {
                        let $arg: $arg_type = $crate::crate_utils::define_source!(@arg_into $arg $(, $is_value)?);
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
                    pub fn fn_name(mut self, $arg: $crate::crate_utils::define_source!(@arg_type_decl $arg_type $(, $is_value)?)) -> Self {
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

        $crate::crate_utils::define_source!(@getters $name, $(($arg, $arg_type, [$(@$is_value)?]))*);

        $crate::crate_utils::define_source!(@setters $name, $(
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
                    $arg: $crate::crate_utils::define_source!(@arg_type_decl $arg_type $(, $is_value)?)
                ),*
            ) -> Self {
                let pose = crate::base::Pose::new(position.into(), orientation.into());

                $(
                    let $arg: $arg_type = $crate::crate_utils::define_source!(@arg_into $arg $(, $is_value)?);
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
                    $( $crate::crate_utils::define_source!(@pass_arg self.$arg $(, $is_value)?), )*
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
                        $( $crate::crate_utils::define_source!(@pass_arg self.$arg $(, $is_value)?), )*
                        &mut out,
                    );
                });

                out
            }

            // MARK: Display
            #[cfg(feature = "alloc")]
            fn format(&self, f: &mut core::fmt::Formatter<'_>, _: &str) -> core::fmt::Result {
                $(
                    let $arg = crate::crate_utils::$arg_fmt(&mut *f, $crate::crate_utils::define_source!(@pass_arg self.$arg $(, $is_value)?));
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
pub(crate) use define_source;
