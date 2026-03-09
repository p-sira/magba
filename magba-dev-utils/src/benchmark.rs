use std::path::Path;

/// Criterion benchmark estimates structure
#[derive(Debug, serde::Deserialize)]
pub struct CriterionEstimates {
    pub mean: CriterionMean,
}

/// Criterion mean structure
#[derive(Debug, serde::Deserialize)]
pub struct CriterionMean {
    pub point_estimate: f64,
}

pub fn extract_criterion_mean(path: &Path) -> Result<f64, String> {
    use std::fs;
    let content =
        fs::read_to_string(path).map_err(|e| format!("Cannot read estimates.json file: {}", e))?;

    let estimates: CriterionEstimates = serde_json::from_str(&content)
        .map_err(|e| format!("Cannot parse estimates.json file: {}", e))?;

    Ok(estimates.mean.point_estimate)
}
