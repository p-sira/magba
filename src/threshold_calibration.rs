/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Calibration-only execution controls.
//!
//! This module is deliberately excluded from normal builds. It is intended for
//! isolated benchmark workers, not for changing policy in production programs.

use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU64, Ordering};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum ExecutionMode {
    Auto = 0,
    Serial = 1,
    Parallel = 2,
}

static MODE: AtomicU8 = AtomicU8::new(ExecutionMode::Auto as u8);
static INSTRUMENT: AtomicBool = AtomicBool::new(false);
static SERIAL_BRANCHES: AtomicU64 = AtomicU64::new(0);
static PARALLEL_BRANCHES: AtomicU64 = AtomicU64::new(0);

pub fn set_execution_mode(mode: ExecutionMode) {
    MODE.store(mode as u8, Ordering::Relaxed);
}

pub fn execution_mode() -> ExecutionMode {
    match MODE.load(Ordering::Relaxed) {
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
}

pub fn branch_counts() -> (u64, u64) {
    (
        SERIAL_BRANCHES.load(Ordering::Relaxed),
        PARALLEL_BRANCHES.load(Ordering::Relaxed),
    )
}

#[inline]
pub(crate) fn should_parallel(auto: bool) -> bool {
    let parallel = match execution_mode() {
        ExecutionMode::Auto => auto,
        ExecutionMode::Serial => false,
        ExecutionMode::Parallel => true,
    };

    if INSTRUMENT.load(Ordering::Relaxed) {
        if parallel {
            PARALLEL_BRANCHES.fetch_add(1, Ordering::Relaxed);
        } else {
            SERIAL_BRANCHES.fetch_add(1, Ordering::Relaxed);
        }
    }
    parallel
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
}
