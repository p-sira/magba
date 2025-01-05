/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use magba::special::cel;
use pyo3::prelude::*;
use pyo3::types::PyModule;
use rand::Rng;

/// Rust wrapper for MagpyLib
fn magpy_cel0(kc: f64, p: f64, c: f64, s: f64) -> f64 {
    pyo3::prepare_freethreaded_python();
    Python::with_gil(|py| {
        let module = PyModule::import(py, "magpylib._src.fields.special_cel")
            .expect("Failed to import magpylib._src.fields.special_cel");
        let result: f64 = module
            .getattr("cel0")
            .expect("Function cel0 not found")
            .call1((kc, p, c, s))
            .expect("Function call failed")
            .extract()
            .expect("Failed to extract result");
        result
    })
}

#[test]
fn test_cel() {
    let n = 1000;
    let mut rng = rand::thread_rng();

    for _ in 0..n {
        let kc = rng.gen_range(1.0..10.0);
        let p = rng.gen_range(-25.0..25.0);
        let c = rng.gen_range(-1.0..1.0);
        let s = rng.gen_range(-5.0..5.0);

        let cel_result = cel(kc, p, c, s);
        let magpy_cel0_result = magpy_cel0(kc, p, c, s);

        assert_eq!(
            cel_result, magpy_cel0_result,
            "Outputs do not match for input: ({}, {}, {}, {})",
            kc, p, c, s
        );
    }
}
