/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use nalgebra::{Isometry3, Point3, RealField, Translation3, UnitQuaternion};

/// Struct for holding object's position and orientation with methods for transformations.
///
/// ### Examples
///
/// ```
/// # use magba::base::Pose;
/// # use nalgebra::Point3;
/// let mut pose: Pose = Pose::default();
/// pose.translate([1.0, 2.0, 3.0]);
/// assert_eq!(pose.position(), Point3::new(1.0, 2.0, 3.0));
/// ```
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Pose<T: RealField = f64> {
    isometry: Isometry3<T>,
}

impl<T: RealField> Default for Pose<T> {
    fn default() -> Self {
        Self {
            isometry: Isometry3::identity(),
        }
    }
}

impl<T: RealField> Pose<T> {
    #[inline]
    pub fn new(position: impl Into<Translation3<T>>, orientation: UnitQuaternion<T>) -> Self {
        Self {
            isometry: Isometry3::from_parts(position.into(), orientation),
        }
    }

    #[inline]
    pub fn position(&self) -> Point3<T> {
        self.isometry.translation.vector.clone().into()
    }

    #[inline]
    pub fn orientation(&self) -> UnitQuaternion<T> {
        self.isometry.rotation.clone()
    }

    #[inline]
    pub fn set_position(&mut self, position: impl Into<Translation3<T>>) {
        self.isometry.translation = position.into();
    }

    #[inline]
    pub fn set_orientation(&mut self, orientation: UnitQuaternion<T>) {
        self.isometry.rotation = orientation;
    }

    /// Translate the object.
    #[inline]
    pub fn translate(&mut self, translation: impl Into<Translation3<T>>) {
        self.isometry.append_translation_mut(&translation.into());
    }

    /// Rotate the object.
    #[inline]
    pub fn rotate(&mut self, rotation: UnitQuaternion<T>) {
        self.isometry.append_rotation_wrt_center_mut(&rotation);
    }

    /// Rotate the object using the anchor point as the center of rotation.
    #[inline]
    pub fn rotate_anchor(&mut self, rotation: UnitQuaternion<T>, anchor: impl Into<Point3<T>>) {
        self.isometry
            .append_rotation_wrt_point_mut(&rotation, &anchor.into());
    }

    #[inline]
    pub fn as_isometry(&self) -> &Isometry3<T> {
        &self.isometry
    }
}

impl<T: RealField> From<Isometry3<T>> for Pose<T> {
    fn from(isometry: Isometry3<T>) -> Self {
        Self { isometry }
    }
}

#[cfg(feature = "std")]
impl<T: RealField> std::fmt::Display for Pose<T> {
    /// ```
    /// # use magba::base::Pose;
    /// let mut pose: Pose = Pose::default();
    /// assert_eq!(format!("{}", pose), "pos=[0.0, 0.0, 0.0], rot=[0.0, 0.0, 0.0]");
    ///
    /// pose.translate([1.23456, 0.0, 0.0]);
    /// assert_eq!(format!("{:.3}", pose), "pos=[1.235, 0.000, 0.000], rot=[0.000, 0.000, 0.000]");
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let t = &self.isometry.translation.vector;
        let r = self.isometry.rotation.scaled_axis();

        if let Some(p) = f.precision() {
            write!(
                f,
                "pos=[{:.p$}, {:.p$}, {:.p$}], rot=[{:.p$}, {:.p$}, {:.p$}]",
                t.x,
                t.y,
                t.z,
                r.x,
                r.y,
                r.z,
                p = p
            )
        } else {
            write!(
                f,
                "pos=[{:?}, {:?}, {:?}], rot=[{:?}, {:?}, {:?}]",
                t.x, t.y, t.z, r.x, r.y, r.z
            )
        }
    }
}

#[cfg(feature = "std")]
impl<T: RealField + std::fmt::LowerExp> std::fmt::LowerExp for Pose<T> {
    /// ```
    /// # use magba::base::Pose;
    /// # use nalgebra::UnitQuaternion;
    /// let mut pose: Pose = Pose::default();
    /// pose.translate([0.0, 0.0, 1e5]);
    /// pose.rotate(UnitQuaternion::from_scaled_axis([0.0, std::f64::consts::PI, 0.0].into()));
    /// assert_eq!(format!("{:.3e}", pose), "pos=[0.000e0, 0.000e0, 1.000e5], rot=[0.000e0, 3.142e0, 0.000e0]");
    /// ```
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let t = &self.isometry.translation.vector;
        let r = self.isometry.rotation.scaled_axis();

        if let Some(p) = f.precision() {
            write!(
                f,
                "pos=[{:.p$e}, {:.p$e}, {:.p$e}], rot=[{:.p$e}, {:.p$e}, {:.p$e}]",
                t.x,
                t.y,
                t.z,
                r.x,
                r.y,
                r.z,
                p = p
            )
        } else {
            write!(
                f,
                "pos=[{:e}, {:e}, {:e}], rot=[{:e}, {:e}, {:e}]",
                t.x, t.y, t.z, r.x, r.y, r.z
            )
        }
    }
}

macro_rules! delegate_to_pose {
    () => {
        delegate::delegate! {
            to self.pose {
                pub fn position(&self) -> nalgebra::Point3<T>;
                pub fn orientation(&self) -> nalgebra::UnitQuaternion<T>;
                pub fn set_position(&mut self, position: impl Into<nalgebra::Translation3<T>>);
                pub fn set_orientation(&mut self, orientation: nalgebra::UnitQuaternion<T>);
                pub fn translate(&mut self, translation: impl Into<nalgebra::Translation3<T>>);
                pub fn rotate(&mut self, rotation: nalgebra::UnitQuaternion<T>);
                pub fn rotate_anchor(&mut self, rotation: nalgebra::UnitQuaternion<T>, anchor: impl Into<nalgebra::Point3<T>>);
            }
        }
    };
}
pub(crate) use delegate_to_pose;

macro_rules! impl_pose_methods {
    () => {
        crate::base::pose::delegate_to_pose!();

        pub fn set_pose(&mut self, pose: crate::base::Pose<T>) {
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

        pub fn with_pose(mut self, pose: crate::base::Pose<T>) -> Self {
            self.set_pose(pose);
            self
        }
    };
}
pub(crate) use impl_pose_methods;
