/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Calibration-only execution controls.
//!
//! This module is intended for isolated benchmark workers, not for changing
//! policy in production programs.

use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU64, Ordering};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum ExecutionMode {
    Auto = 0,
    Serial = 1,
    Parallel = 2,
}

static MODE: AtomicU8 = AtomicU8::new(ExecutionMode::Auto as u8);
static COLLECTION_MODE: AtomicU8 = AtomicU8::new(ExecutionMode::Auto as u8);
static INSTRUMENT: AtomicBool = AtomicBool::new(false);
static SERIAL_BRANCHES: AtomicU64 = AtomicU64::new(0);
static PARALLEL_BRANCHES: AtomicU64 = AtomicU64::new(0);
static COLLECTION_SERIAL_BRANCHES: AtomicU64 = AtomicU64::new(0);
static COLLECTION_PARALLEL_BRANCHES: AtomicU64 = AtomicU64::new(0);

pub fn set_execution_mode(mode: ExecutionMode) {
    MODE.store(mode as u8, Ordering::Relaxed);
}

pub fn execution_mode() -> ExecutionMode {
    decode_mode(MODE.load(Ordering::Relaxed))
}

pub fn set_collection_execution_mode(mode: ExecutionMode) {
    COLLECTION_MODE.store(mode as u8, Ordering::Relaxed);
}

pub fn collection_execution_mode() -> ExecutionMode {
    decode_mode(COLLECTION_MODE.load(Ordering::Relaxed))
}

fn decode_mode(mode: u8) -> ExecutionMode {
    match mode {
        1 => ExecutionMode::Serial,
        2 => ExecutionMode::Parallel,
        _ => ExecutionMode::Auto,
    }
}

pub fn set_instrumentation(enabled: bool) {
    INSTRUMENT.store(enabled, Ordering::Relaxed);
}

pub fn reset_branch_counts() {
    SERIAL_BRANCHES.store(0, Ordering::Relaxed);
    PARALLEL_BRANCHES.store(0, Ordering::Relaxed);
    COLLECTION_SERIAL_BRANCHES.store(0, Ordering::Relaxed);
    COLLECTION_PARALLEL_BRANCHES.store(0, Ordering::Relaxed);
}

/// Returns `(serial, parallel)` primitive branch counts.
pub fn branch_counts() -> (u64, u64) {
    (
        SERIAL_BRANCHES.load(Ordering::Relaxed),
        PARALLEL_BRANCHES.load(Ordering::Relaxed),
    )
}

/// Returns `(serial, parallel)` outer collection branch counts.
pub fn collection_branch_counts() -> (u64, u64) {
    (
        COLLECTION_SERIAL_BRANCHES.load(Ordering::Relaxed),
        COLLECTION_PARALLEL_BRANCHES.load(Ordering::Relaxed),
    )
}

#[inline]
pub(crate) fn should_parallel(auto: bool) -> bool {
    let parallel = match execution_mode() {
        ExecutionMode::Auto => auto,
        ExecutionMode::Serial => false,
        ExecutionMode::Parallel => true,
    };

    record_branch(parallel, &SERIAL_BRANCHES, &PARALLEL_BRANCHES);
    parallel
}

#[inline]
pub(crate) fn should_parallel_collection(auto: bool, can_parallelize: bool) -> bool {
    let parallel = can_parallelize
        && match execution_mode() {
            ExecutionMode::Auto => match collection_execution_mode() {
                ExecutionMode::Auto => auto,
                ExecutionMode::Serial => false,
                ExecutionMode::Parallel => true,
            },
            ExecutionMode::Serial => false,
            ExecutionMode::Parallel => true,
        };

    record_branch(
        parallel,
        &COLLECTION_SERIAL_BRANCHES,
        &COLLECTION_PARALLEL_BRANCHES,
    );
    parallel
}

#[inline]
fn record_branch(parallel: bool, serial_count: &AtomicU64, parallel_count: &AtomicU64) {
    if INSTRUMENT.load(Ordering::Relaxed) {
        if parallel {
            parallel_count.fetch_add(1, Ordering::Relaxed);
        } else {
            serial_count.fetch_add(1, Ordering::Relaxed);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn modes_override_automatic_choice() {
        set_instrumentation(true);
        reset_branch_counts();

        set_execution_mode(ExecutionMode::Serial);
        assert!(!should_parallel(true));
        set_execution_mode(ExecutionMode::Parallel);
        assert!(should_parallel(false));
        set_execution_mode(ExecutionMode::Auto);
        assert!(should_parallel(true));
        assert!(!should_parallel(false));

        assert_eq!(branch_counts(), (2, 2));
        set_instrumentation(false);
    }

    #[test]
    fn collections_require_outer_parallelism() {
        set_instrumentation(true);
        reset_branch_counts();
        set_execution_mode(ExecutionMode::Parallel);

        assert!(!should_parallel_collection(false, false));
        assert!(should_parallel_collection(false, true));
        assert_eq!(collection_branch_counts(), (1, 1));

        set_execution_mode(ExecutionMode::Auto);
        set_collection_execution_mode(ExecutionMode::Auto);
        set_instrumentation(false);
    }
}
