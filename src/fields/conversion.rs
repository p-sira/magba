/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Conversion tools for magnetic field-related quantities.

use nalgebra::{RealField, Vector3};

use crate::constants::MagneticConstants;

/// Convert B-field to H-field.
///
/// # Arguments
/// * `B_vector` - Magnetic flux density vector (B) in T
///
/// # Returns
/// * `Vector3<f64>` - Magnetic field strength vector (H) in A/m
#[allow(non_snake_case)]
pub fn B_to_H<T: RealField + MagneticConstants>(B_vector: &Vector3<T>) -> Vector3<T> {
    B_vector.scale(T::mu0())
}

/// Convert vector of B-fields to vector of H-fields.
///
/// # Arguments
/// * `B_vectors` - Slice of B-field vectors
///
/// # Returns
/// * `Vec<Vector3<f64>>` - H-field vectors
#[allow(non_snake_case)]
pub fn Bs_to_Hs<T: RealField + MagneticConstants>(B_vectors: &[Vector3<T>]) -> Vec<Vector3<T>> {
    B_vectors
        .iter()
        .map(|vector| vector.scale(T::mu0()))
        .collect()
}

/// Convert magnetization (**M**) to polarization (**J**).
///
/// # Arguments
/// * `mag_vector` - Magnetization vector (M) in A/m
///
/// # Returns
/// * `Vector3<f64>` - Polarization vector (J) in T
pub fn mag_to_pol<T: RealField + MagneticConstants>(mag_vector: &Vector3<T>) -> Vector3<T> {
    mag_vector.scale(T::one() / T::mu0())
}

/// Convert magnetizations (**M**) to polarizations (**J**).
///
/// # Arguments
/// * `mag_vectors` - Slice of magnetization vectors (M) in A/m
///
/// # Returns
/// * `Vec<Vector3<f64>>` - Polarization vectors (J) in T
pub fn mags_to_pols<T: RealField + MagneticConstants>(
    mag_vectors: &[Vector3<T>],
) -> Vec<Vector3<T>> {
    mag_vectors
        .iter()
        .map(|vector| vector.scale(T::one() / T::mu0()))
        .collect()
}
