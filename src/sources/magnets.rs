///! Function and macros shared by all magnetic sources

/// Define magnet struct
/// ```text
/// define_magnet!{
///     Magnet
///     field_fn: magnet_B
///     args: {polarization:Vector3<T>, dimensions:Vector3<T>, lucky_number v:T}
///     arg_display: "pol={}, dim={}, lucky={}";
///     arg_fmt: [format_vector3, format_vector3, format_float]
///     on_new: [
///         if lucky_number != T::from_f64(42.0).unwrap() { panic!() }
///     ]
/// }
/// ```
///
/// ```text
/// $(#[$meta:meta])*
/// $name:ident
/// field_fn: $field_fn:ident
/// args: { $($arg:ident $($pass_arg_by:ident)?: $arg_type:ty),* $(,)? }
/// arg_display: $arg_display:expr;
/// arg_fmt: [ $($arg_fmt:ident),* $(,)? ]
/// on_new: [ $($on_new:tt)* ]
/// ```
/// - $name: The name of the struct
/// - $field_fn: Function identifier for magnetic field (B) calculation with signature: (points, position, orientation, fields, ...)
/// - $arg: Arguments of the field function, excluding position and orientation, which are also the field of the struct
/// - $pass_arg_by: Use v to pass by value, leave out to pass by reference
/// - $arg_type: Type of the argument
/// - $arg_display: String for displaying $args
/// - $arg_fmt: Macros to use to format for each $arg
/// - $on_new: Checks to do in new() method before returning the instance
macro_rules! define_magnet {
    (@pass_arg $arg:expr) => { &$arg };
    (@pass_arg $arg:expr, v) => { $arg };
    {
        $(#[$meta:meta])*
        $name:ident
        field_fn: $field_fn:ident
        args: { $($arg:ident $($pass_arg_by:ident)?: $arg_type:ty),* $(,)? }
        arg_display: $arg_display:expr;
        arg_fmt: [ $($arg_fmt:ident),* $(,)? ]
        on_new: [ $($on_new:tt)* ]
    } => {
        $(#[$meta])*
        #[derive(Debug, Clone, PartialEq, Default, getset::Getters, getset::Setters)]
        pub struct $name<T: crate::Float> {
            position: nalgebra::Point3<T>,
            orientation: nalgebra::UnitQuaternion<T>,
            $(
                #[getset(get = "pub", set = "pub")]
                $arg: $arg_type,
            )*
        }

        impl<T: crate::Float> $name<T> {
            pub fn new(
                position: nalgebra::Point3<T>,
                orientation: nalgebra::UnitQuaternion<T>,
                $($arg: $arg_type),*
            ) -> Self {
                $($on_new)*
                $name {
                    position,
                    orientation,
                    $($arg),*
                }
            }
        }

        impl<T: crate::Float> crate::Source<T> for $name<T> {}
        impl<T: crate::Float> crate::geometry::Transform<T> for $name<T> {
            crate::geometry::impl_transform!();
        }
        impl<T: crate::Float> crate::Field<T> for $name<T> {
            fn get_B(&self, points: &[nalgebra::Point3<T>]) -> Vec<nalgebra::Vector3<T>> {
                crate::fields::$field_fn(
                    points,
                    &self.position,
                    &self.orientation,
                    $( define_magnet!(@pass_arg self.$arg $(, $pass_arg_by)?), )*
                )
            }
        }
        impl<T: crate::Float> std::fmt::Display for $name<T> {
            fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                write!(
                    f,
                    concat!(
                        stringify!($name),
                        " (",
                        $arg_display,
                        ") at pos={}, q={}"
                    ),
                    $(crate::crate_util::$arg_fmt!(&self.$arg),)*
                    crate::crate_util::format_point3!(&self.position),
                    crate::crate_util::format_quat!(&self.orientation)
                )
            }
        }
    }
}
pub(crate) use define_magnet;
