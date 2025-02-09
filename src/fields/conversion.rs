/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Conversion tools for magnetic field-related quantities.

use nalgebra::Vector3;

use crate::constants::MU0;

/// Convert B-field to H-field.
#[allow(non_snake_case)]
pub fn B_to_H(B_vector: &Vector3<f64>) -> Vector3<f64> {
    B_vector.scale(MU0)
}

/// Convert vector of B-fields to vector of H-fields.
#[allow(non_snake_case)]
pub fn Bs_to_Hs(B_vectors: &[Vector3<f64>]) -> Vec<Vector3<f64>> {
    B_vectors.iter().map(|vector| vector.scale(MU0)).collect()
}

/// Convert magnetization (**M**) to polarization (**J**).
pub fn mag_to_pol(mag_vector: &Vector3<f64>) -> Vector3<f64> {
    mag_vector.scale(1.0 / MU0)
}

/// Convert magnetizations (**M**) to polarizations (**J**).
pub fn mags_to_pols(mag_vectors: &[Vector3<f64>]) -> Vec<Vector3<f64>> {
    mag_vectors
        .iter()
        .map(|vector| vector.scale(1.0 / MU0))
        .collect()
}
