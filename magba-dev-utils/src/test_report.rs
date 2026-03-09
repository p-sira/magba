pub fn format_float(val: &f64) -> String {
    if val.is_nan() {
        "-".to_string()
    } else {
        format!("{:.3}", val)
    }
}

pub fn format_performance(val: &f64) -> String {
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
