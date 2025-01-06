/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

#[macro_use]
mod helper;

use magba::fields::axial_cyl_b_cyl;
use numpy::*;
use pyo3::PyObject;

fn magpy_cylinder_axial_bfield(r: f64, z: f64, radius: f64, height: f64) {
    let z0 = (height / 2.0) / radius;

    pyo3::prepare_freethreaded_python();
    pyo3::Python::with_gil(|py| {
        let arr_z0 = pyarray![py, z0];
        let arr_r = pyarray![py, r];
        let arr_z = pyarray![py, z];
        let arr_result = pyfnvec!(
            py,
            "magpylib._src.fields.field_BH_cylinder",
            "magnet_cylinder_axial_Bfield",
            (arr_z0, arr_r, arr_z),
            PyObject
        );

        arr_result
    });
}

#[test]
fn test_axial_cyl_b_cyl() {
    
    let r = 1.0;
    let z = 3.0;
    let radius: f64 = 0.5;
    let length = 2.0;
    test!(axial_cyl_b_cyl, magpy_cylinder_axial_bfield, (r, z, radius, length));
}
