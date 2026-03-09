/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Conversion utilities for magnetic field-related quantities.

use nalgebra::Vector3;

use crate::base::Float;

/// Convert B-field vector to H-field vector.
///
/// # Arguments
///
/// - `B_vector`: Magnetic flux density vector (T)
///
/// # Returns
///
/// - Magnetic field strength vector (A/m)
///
/// # Examples
///
/// ```
/// # use magba::conversion::B_to_H;
/// # use nalgebra::vector;
/// # use approx::assert_relative_eq;
/// let B = vector![0.0, 0.0, 1.2566370614359173e-6];
/// let H = B_to_H(B);
/// assert_relative_eq!(H, vector![0.0, 0.0, 1.0]);
/// ```
#[allow(non_snake_case)]
pub fn B_to_H<T: Float>(B_vector: Vector3<T>) -> Vector3<T> {
    B_vector.scale(T::recip_mu0())
}

/// Convert magnetization (**M**) to polarization (**J**).
///
/// # Arguments
///
/// - `mag_vector`: Magnetization vector (A/m)
///
/// # Returns
///
/// - Polarization vector (T)
///
/// # Examples
///
/// ```
/// # use magba::conversion::mag_to_pol;
/// # use nalgebra::vector;
/// # use approx::assert_relative_eq;
/// let M = vector![0.0, 0.0, 1.0];
/// let J = mag_to_pol(M);
/// assert_relative_eq!(J, vector![0.0, 0.0, 1.2566370614359173e-6]);
/// ```
pub fn mag_to_pol<T: Float>(mag_vector: Vector3<T>) -> Vector3<T> {
    mag_vector.scale(T::mu0())
}

crate::crate_utils::need_alloc! {
    use alloc::vec::Vec;

    /// Convert B-field vectors to H-field vectors.
    ///
    /// # Arguments
    ///
    /// - `B_vectors`: Magnetic flux density vectors (T)
    ///
    /// # Returns
    ///
    /// - Magnetic field strength vectors (A/m)
    ///
    /// # Examples
    ///
    /// ```
    /// # use magba::conversion::B_to_H_batch;
    /// # use nalgebra::vector;
    /// # use approx::assert_relative_eq;
    /// let B = vec![vector![0.0, 0.0, 1.2566370614359173e-6]];
    /// let H = B_to_H_batch(&B);
    /// assert_relative_eq!(H[0], vector![0.0, 0.0, 1.0]);
    /// ```
    #[allow(non_snake_case)]
    pub fn B_to_H_batch<T: Float>(B_vectors: &[Vector3<T>]) -> Vec<Vector3<T>> {
        B_vectors
            .iter()
            .map(|vector| vector.scale(T::recip_mu0()))
            .collect()
    }

    /// Convert magnetization vectors (**M**) to polarization vectors (**J**).
    ///
    /// # Arguments
    ///
    /// - `mag_vectors`: Magnetization vectors (A/m)
    ///
    /// # Returns
    ///
    /// - Polarization vectors (T)
    ///
    /// # Examples
    ///
    /// ```
    /// # use magba::conversion::mag_to_pol_batch;
    /// # use nalgebra::vector;
    /// # use approx::assert_relative_eq;
    /// let M = vec![vector![0.0, 0.0, 1.0]];
    /// let J = mag_to_pol_batch(&M);
    /// assert_relative_eq!(J[0], vector![0.0, 0.0, 1.2566370614359173e-6]);
    /// ```
    pub fn mag_to_pol_batch<T: Float>(
        mag_vectors: &[Vector3<T>],
    ) -> Vec<Vector3<T>> {
        mag_vectors
            .iter()
            .map(|vector| vector.scale(T::mu0()))
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use nalgebra::vector;

    #[test]
    fn test_b_to_h() {
        let b = vector![0.0, 0.0, 1.2566370614359173e-6];
        let h = B_to_H(b);
        assert_relative_eq!(h, vector![0.0, 0.0, 1.0]);
    }

    #[test]
    fn test_mag_to_pol() {
        let m = vector![0.0, 0.0, 1.0];
        let j = mag_to_pol(m);
        assert_relative_eq!(j, vector![0.0, 0.0, 1.2566370614359173e-6]);
    }

    #[test]
    fn test_b_to_h_batch() {
        let b = vec![
            vector![0.0, 0.0, 1.2566370614359173e-6],
            vector![1.2566370614359173e-6, 0.0, 0.0],
        ];
        let h = B_to_H_batch(&b);
        assert_relative_eq!(h[0], vector![0.0, 0.0, 1.0]);
        assert_relative_eq!(h[1], vector![1.0, 0.0, 0.0]);
    }

    #[test]
    fn test_mag_to_pol_batch() {
        let m = vec![vector![0.0, 0.0, 1.0], vector![1.0, 0.0, 0.0]];
        let j = mag_to_pol_batch(&m);
        assert_relative_eq!(j[0], vector![0.0, 0.0, 1.2566370614359173e-6]);
        assert_relative_eq!(j[1], vector![1.2566370614359173e-6, 0.0, 0.0]);
    }
}
