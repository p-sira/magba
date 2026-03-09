/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use std::fs;
use std::path::Path;

#[derive(Debug, Clone)]
struct BenchmarkResult {
    function_name: String,
    threshold: usize,
}

fn parse_markdown_table(content: &str) -> Vec<BenchmarkResult> {
    let mut results = Vec::new();
    let lines: Vec<&str> = content.lines().collect();

    // Skip header lines (first 2 lines are | Function | ... | and |-----|...)
    for line in lines.iter().skip(2) {
        if line.trim().is_empty() || !line.starts_with('|') {
            continue;
        }

        let parts: Vec<&str> = line.split('|').collect();
        if parts.len() >= 3 {
            // Handle Criterion group names like "threshold/cylinder_B"
            let raw_name = parts[1].trim();
            let function_name = if let Some(idx) = raw_name.rfind('/') {
                raw_name[idx + 1..].to_string()
            } else {
                raw_name.to_string()
            };

            let threshold_str = parts[2].trim();

            if let Ok(threshold) = threshold_str.parse::<usize>() {
                results.push(BenchmarkResult {
                    function_name,
                    threshold,
                });
            }
        }
    }

    results
}

fn generate_code(func_name: &str, args: &str, bench_result: &BenchmarkResult) -> String {
    format!(
        "
    impl_parallel!(
        {},
        rayon_threshold: {},
        input: points,
        output: out,
        args: {}
    )
",
        func_name, bench_result.threshold, args
    )
}

fn generate_lib_rs_code(results: &[BenchmarkResult]) -> String {
    let mut code = String::new();

    code.push_str("// Generated threshold values from benchmark results\n");

    // Map each function to its specific macro arguments
    let functions = [
        ("circular_B", "[position, orientation, diameter, current]"),
        (
            "cuboid_B",
            "[position, orientation, polarization, dimensions]",
        ),
        (
            "cylinder_B",
            "[position, orientation, polarization, diameter, height]",
        ),
        ("dipole_B", "[position, orientation, moment]"),
        (
            "sphere_B",
            "[position, orientation, polarization, diameter]",
        ),
    ];

    for (func_name, args) in &functions {
        if let Some(result) = results.iter().find(|r| r.function_name == *func_name) {
            code.push_str(&generate_code(func_name, args, result));
        } else {
            panic!("Benchmark not found for function {}", func_name)
        }
    }

    code
}

fn main() {
    let md_file_path = Path::new("benches/par_threshold.md");

    if !md_file_path.exists() {
        eprintln!("Error: {} not found", md_file_path.display());
        panic!("Please run the benchmark first to generate the results.");
    }

    let content = match fs::read_to_string(md_file_path) {
        Ok(content) => content,
        Err(e) => {
            panic!("Error reading {}: {}", md_file_path.display(), e);
        }
    };

    let results = parse_markdown_table(&content);

    if results.is_empty() {
        panic!("No benchmark results found in {}", md_file_path.display());
    }

    let generated_code = generate_lib_rs_code(&results);
    println!("{generated_code}");

    let save_path = Path::new("benches/impl_parallel.txt");
    fs::write(save_path, generated_code).expect("Cannot write to target file");
    println!("// Saved to {}", save_path.display());
}
