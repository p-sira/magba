/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2026 Sira Pornsiriprasert <code@psira.me>
 */

//! Shared helpers for primitive and collection parallel-threshold calibration.

use std::io::Write;
use std::path::{Path, PathBuf};
use tabled::{Table, Tabled, settings::Style};

pub const ACCEPT_THRESHOLD: f64 = 0.05;
pub const MIN_STEP: usize = 5;
pub const MAX_ITER: usize = 20;

#[derive(Clone, Copy)]
pub struct Timing {
    pub parallel_ns: f64,
    pub serial_ns: f64,
}

impl Timing {
    pub fn ratio(self) -> f64 {
        self.parallel_ns / self.serial_ns
    }
}

#[derive(Clone, Copy)]
pub enum ExitCond {
    None,
    Converged,
    MinStep,
    MaxIter,
    MaxThres,
}

impl std::fmt::Display for ExitCond {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::None => "-",
            Self::Converged => "Converged",
            Self::MinStep => "Min step size",
            Self::MaxIter => "Max iteration",
            Self::MaxThres => "Max threshold",
        })
    }
}

pub struct SearchResult {
    pub threshold: usize,
    pub timing: Timing,
    pub exit: ExitCond,
}

/// Find the serial/parallel crossover, recording only candidate sizes.
/// The caller owns workload construction and the actual Criterion benchmarks.
pub fn find_threshold(
    initial: usize,
    max_threshold: usize,
    min_step: usize,
    mut measure: impl FnMut(usize) -> Option<Timing>,
) -> Option<SearchResult> {
    let step = min_step.max(1).min(max_threshold);
    if step == 0 {
        return None;
    }
    let max_units = max_threshold / step;
    let mut high = (initial.saturating_add(step / 2) / step).clamp(1, max_units);
    let mut high_timing = measure(high * step)?;

    let result = |units, timing, exit| SearchResult {
        threshold: units * step,
        timing,
        exit,
    };
    let near_crossover = |timing: Timing| (timing.ratio() - 1.0).abs() <= ACCEPT_THRESHOLD;
    if near_crossover(high_timing) {
        return Some(result(high, high_timing, ExitCond::Converged));
    }

    let mut low = 1;
    if high_timing.ratio() > 1.0 {
        low = high;
        while high < max_units {
            high = high.saturating_mul(2).min(max_units);
            high_timing = measure(high * step)?;
            if near_crossover(high_timing) {
                return Some(result(high, high_timing, ExitCond::Converged));
            }
            if high_timing.ratio() < 1.0 {
                break;
            }
            low = high;
        }
        if low == high {
            return Some(result(high, high_timing, ExitCond::MaxThres));
        }
    } else if high > 1 {
        let first_timing = measure(step)?;
        if near_crossover(first_timing) {
            return Some(result(1, first_timing, ExitCond::Converged));
        }
        if first_timing.ratio() < 1.0 {
            return Some(result(1, first_timing, ExitCond::MinStep));
        }
    } else {
        return Some(result(high, high_timing, ExitCond::MinStep));
    }

    for _ in 0..MAX_ITER {
        if high - low <= 1 {
            return Some(result(high, high_timing, ExitCond::MinStep));
        }
        let mid = low + (high - low) / 2;
        let timing = measure(mid * step)?;
        if near_crossover(timing) {
            return Some(result(mid, timing, ExitCond::Converged));
        }
        if timing.ratio() < 1.0 {
            high = mid;
            high_timing = timing;
        } else {
            low = mid;
        }
    }
    Some(result(high, high_timing, ExitCond::MaxIter))
}

pub fn criterion_root() -> PathBuf {
    if Path::new("target/criterion").exists() || !Path::new("../target/criterion").exists() {
        PathBuf::from("target/criterion")
    } else {
        PathBuf::from("../target/criterion")
    }
}

pub fn criterion_mean_ns(group: &str, mode: &str, id: &str) -> Option<f64> {
    let path = criterion_root()
        .join(group)
        .join(mode)
        .join(id)
        .join("new/estimates.json");
    reproducible::benchmark::extract_criterion_mean_ns(&path).ok()
}

fn format_float(val: &f64) -> String {
    if val.is_nan() {
        "-".to_string()
    } else {
        format!("{:.3}", val)
    }
}

fn format_performance(val: &f64) -> String {
    let val = *val / 1e9;
    if val.is_nan() {
        "-".to_string()
    } else if val < 1e-6 {
        format!("{:.3} ns", val * 1e9)
    } else if val < 1e-3 {
        format!("{:.3} µs", val * 1e6)
    } else if val < 1.0 {
        format!("{:.3} ms", val * 1e3)
    } else {
        format!("{:.3} s", val)
    }
}

fn parse_performance(s: &str) -> f64 {
    let s = s.trim();
    if s == "-" {
        return f64::NAN;
    }
    if let Some(num) = s.strip_suffix("ns") {
        num.trim().parse::<f64>().unwrap_or(f64::NAN)
    } else if let Some(num) = s.strip_suffix("µs").or_else(|| s.strip_suffix("us")) {
        num.trim()
            .parse::<f64>()
            .map(|v| v * 1e3)
            .unwrap_or(f64::NAN)
    } else if let Some(num) = s.strip_suffix("ms") {
        num.trim()
            .parse::<f64>()
            .map(|v| v * 1e6)
            .unwrap_or(f64::NAN)
    } else if let Some(num) = s.strip_suffix("s") {
        num.trim()
            .parse::<f64>()
            .map(|v| v * 1e9)
            .unwrap_or(f64::NAN)
    } else {
        f64::NAN
    }
}

#[derive(Tabled)]
pub struct Record {
    #[tabled(rename = "Function")]
    pub function_name: String,
    #[tabled(rename = "Threshold")]
    pub threshold: usize,
    #[tabled(rename = "Parallel Time", display = "format_performance")]
    pub par_time: f64,
    #[tabled(rename = "Serial Time", display = "format_performance")]
    pub ser_time: f64,
    #[tabled(rename = "Ratio", display = "format_float")]
    pub ratio: f64,
    #[tabled(rename = "Exit Condition")]
    pub exit_cond: ExitCond,
}

pub fn load_results(path: &Path) -> Vec<Record> {
    let Ok(content) = std::fs::read_to_string(path) else {
        return Vec::new();
    };
    let mut records = Vec::new();
    for line in content.lines().skip(2) {
        let line = line.trim();
        if line.is_empty() || !line.starts_with('|') {
            continue;
        }
        let parts: Vec<&str> = line.split('|').map(|s| s.trim()).collect();
        if parts.len() >= 7 {
            let function_name = parts[1].to_string();
            let Ok(threshold) = parts[2].parse::<usize>() else {
                continue;
            };
            let par_time = parse_performance(parts[3]);
            let ser_time = parse_performance(parts[4]);
            let ratio = parts[5].parse::<f64>().unwrap_or(f64::NAN);
            let exit_cond = match parts[6] {
                "Converged" => ExitCond::Converged,
                "Min step size" => ExitCond::MinStep,
                "Max iteration" => ExitCond::MaxIter,
                "Max threshold" => ExitCond::MaxThres,
                _ => ExitCond::None,
            };
            records.push(Record {
                function_name,
                threshold,
                par_time,
                ser_time,
                ratio,
                exit_cond,
            });
        }
    }
    records
}

pub fn update_or_push_record(results: &mut Vec<Record>, record: Record) {
    if let Some(pos) = results
        .iter()
        .position(|r| r.function_name == record.function_name)
    {
        results[pos] = record;
    } else {
        results.push(record);
    }
}

pub fn save_results(path: &Path, results: &[Record]) {
    let result_str = Table::new(results).with(Style::markdown()).to_string();
    let mut file = std::fs::File::create(path).unwrap();
    file.write_all(result_str.as_bytes()).unwrap();
}
