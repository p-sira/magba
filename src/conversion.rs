/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Conversion utilities for magnetic field-related quantities.

use nalgebra::Vector3;

use crate::{base::Float, crate_utils::need_std};

/// Convert B-field vector to H-field vector.
///
/// # Arguments
///
/// - `B_vector`: Magnetic flux density vector (T)
///
/// # Returns
///
/// - Magnetic field strength vector (A/m)
#[allow(non_snake_case)]
pub fn B_to_H<T: Float>(B_vector: Vector3<T>) -> Vector3<T> {
    B_vector.scale(T::mu0())
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
pub fn mag_to_pol<T: Float>(mag_vector: Vector3<T>) -> Vector3<T> {
    mag_vector.scale(T::one() / T::mu0())
}

need_std! {
    /// Convert B-field vectors to H-field vectors.
    ///
    /// # Arguments
    ///
    /// - `B_vectors`: Magnetic flux density vectors (T)
    ///
    /// # Returns
    ///
    /// - Magnetic field strength vectors (A/m)
    #[allow(non_snake_case)]
    pub fn B_to_H_batch<T: Float>(B_vectors: &[Vector3<T>]) -> Vec<Vector3<T>> {
        B_vectors
            .iter()
            .map(|vector| vector.scale(T::mu0()))
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
    pub fn mag_to_pol_batch<T: Float>(
        mag_vectors: &[Vector3<T>],
    ) -> Vec<Vector3<T>> {
        mag_vectors
            .iter()
            .map(|vector| vector.scale(T::one() / T::mu0()))
            .collect()
    }
}
