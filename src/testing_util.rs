/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Testing utilities.

use std::{fs::File, io::BufReader, path::Path};

use csv::ReaderBuilder;
use nalgebra::{DMatrix, Point3, UnitQuaternion, Vector3};

use crate::crate_util::relative_vec_distance;

pub fn load_matrix_from_csv(path: &Path) -> DMatrix<f64> {
    let file =
        File::open(path).unwrap_or_else(|e| panic!("Cannot open file {}: {}", path.display(), e));
    let mut reader = ReaderBuilder::new()
        .has_headers(false)
        .from_reader(BufReader::new(file));

    let mut data = Vec::new();
    let mut ncols = 0;
    let mut nrows = 0;

    for result in reader.records() {
        let record = result.unwrap_or_else(|e| panic!("Cannot read file: {}", e));
        if ncols == 0 {
            ncols = record.len();
        } else if ncols != record.len() {
            panic!(
                "CSV row {} has inconsistent number of columns: expected {}, got {}",
                nrows + 1,
                ncols,
                record.len()
            );
        }

        for field in record.iter() {
            let value: f64 = field.parse().unwrap_or_else(|e| {
                panic!(
                    "Failed to parse '{}' as f64 on row {}: {}",
                    field,
                    nrows + 1,
                    e
                )
            });
            data.push(value);
        }

        nrows += 1;
    }

    DMatrix::from_row_slice(nrows, ncols, &data)
}

pub fn matrix_to_point_vec(matrix: &DMatrix<f64>) -> Vec<Point3<f64>> {
    matrix
        .row_iter()
        .map(|row| Point3::new(row[0], row[1], row[2]))
        .collect()
}

pub fn matrix_to_vector_vec(matrix: &DMatrix<f64>) -> Vec<Vector3<f64>> {
    matrix
        .row_iter()
        .map(|row| Vector3::new(row[0], row[1], row[2]))
        .collect()
}

pub fn quat_from_rotvec(x: f64, y: f64, z: f64) -> UnitQuaternion<f64> {
    UnitQuaternion::from_scaled_axis(Vector3::new(x, y, z))
}

pub fn assert_close_vec_vector(vecs1: &Vec<Vector3<f64>>, vecs2: &Vec<Vector3<f64>>, rtol: f64) {
    let len = vecs1.len();
    if len != vecs2.len() {
        panic!("assert_close_vector fails. Two vecs of Vector3 must be the same length.")
    }
    // Use parallel comparison for large vectors
    #[cfg(feature = "parallel")]
    if len > 1000 {
        use rayon::prelude::*;
        let failures: Vec<_> = vecs1.par_iter().zip(vecs2).enumerate().filter_map(|(n, (vec1, vec2))| {
            let rdist = relative_vec_distance(*vec1, *vec2);
            if rdist > rtol {
                Some((n, *vec1, *vec2, rdist))
            } else {
                None
            }
        }).collect();
        if !failures.is_empty() {
            let (worst_n, worst_v1, worst_v2, worst_rdist) = failures.iter().max_by(|a, b| a.3.partial_cmp(&b.3).unwrap()).unwrap();
            let percent_fail = failures.len() as f64 / len as f64 * 100.0;
            eprintln!("Failed {}/{} vectors ({}%).", failures.len(), len, percent_fail);
            eprintln!("Worst on vector {}: actual={:?}, expected={:?}, rdist={:e}, rtol={:e}.", worst_n, worst_v1, worst_v2, worst_rdist, rtol);
            panic!("assert_close_vec_vector");
        }
        return;
    }
    
    // Serial fallback for small vectors or no parallel feature
    let mut n_fail: usize = 0;
    let mut worst_rdist = 0.0;
    let mut worst_params: (usize, Vector3<f64>, Vector3<f64>, f64) =
        (0, Vector3::default(), Vector3::default(), 0.0);
    vecs1
        .iter()
        .zip(vecs2)
        .enumerate()
        .for_each(|(n, (&vec1, &vec2))| {
            let rdist = relative_vec_distance(vec1, vec2);
            if rdist > rtol {
                if rdist > worst_rdist {
                    worst_rdist = rdist;
                    worst_params = (n, vec1, vec2, rdist);
                }
                eprintln!(
                    "Vector {} mismatched. actual={:?}, expected={:?}, rdist={:e}, rtol={:e}.",
                    n, vec1, vec2, rdist, rtol
                );
                n_fail += 1;
            }
        });
    if n_fail > 0 {
        let percent_fail = n_fail as f64 / len as f64 * 100.0;
        eprintln!("Failed {n_fail}/{len} vectors ({percent_fail}%).");
        eprintln!(
            "Worst on vector {}: actual={:?}, expected={:?}, rdist={:e}, rtol={:e}.",
            worst_params.0, worst_params.1, worst_params.2, worst_params.3, rtol
        );
        panic!("assert_close_vec_vector")
    }
}

#[cfg(feature = "sources")]
pub use source_testing_util::*;

#[cfg(feature = "sources")]
pub mod source_testing_util {
    use super::*;
    use crate::sources::*;

    #[allow(non_snake_case)]
    pub fn compare_B_with_file<T: Source>(
        source: &T,
        points_path_str: &str,
        ref_path_str: &str,
        rtol: f64,
    ) {
        let points_path = Path::new(points_path_str);
        let ref_path = Path::new(ref_path_str);
        if !points_path.is_file() {
            println!("Test data {points_path:?} not found. Download from https://github.com/p-sira/magba/tree/main/tests/test-data.");
            return;
        }
        if !ref_path.is_file() {
            println!("Test data {ref_path:?} not found. Download from https://github.com/p-sira/magba/tree/main/tests/test-data.");
            return;
        }

        let expected = matrix_to_vector_vec(&load_matrix_from_csv(ref_path));
        let points = matrix_to_point_vec(&load_matrix_from_csv(points_path));

        let b_fields = source.get_B(&points).expect("Cannot calculate b field");

        assert_close_vec_vector(&b_fields, &expected, rtol);
    }
}
