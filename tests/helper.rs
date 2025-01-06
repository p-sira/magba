/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

#[macro_export]
macro_rules! pyfn {
    ($module:expr, $func:expr, ($($arg:expr),*), $return_ty:ty) => {{
        use pyo3::types::PyAnyMethods;

        pyo3::prepare_freethreaded_python();
        pyo3::Python::with_gil(|py| {
            let the_module = pyo3::types::PyModule::import(py, $module)
                .expect(&format!("Failed to import {}", $module));
            
            let the_func = the_module.getattr($func)
                .expect(&format!("Function {} not found", $func));

            let result: $return_ty = the_func
                .call1(($($arg),*))
                .expect("Function call failed")
                .extract()
                .expect("Failed to extract result");
            result
        })
    }};
}

#[macro_export]
macro_rules! pyfnvec {
    ($py: expr, $module:expr, $func:expr, ($($arg:expr),*), $return_ty:ty) => {{
        use pyo3::types::PyAnyMethods;

        let the_module = pyo3::types::PyModule::import($py, $module)
            .expect(&format!("Failed to import {}", $module));
        
        let the_func = the_module.getattr($func)
            .expect(&format!("Function {} not found", $func));

        let result = the_func
            .call1(($($arg),*))
            .expect("Function call failed");

        result.extract::<$return_ty>()
            .expect("Failed to extract result")
    }};
}

#[macro_export]
macro_rules! test {
    ($fn1: expr, $fn2: expr, ($($arg:expr),*)) => {
        let res1 = $fn1($($arg),*);
        let res2 = $fn2($($arg),*);
        assert_eq!(res1, res2);
    };
}