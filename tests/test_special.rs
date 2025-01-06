/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

#[macro_use]
mod helper;
 
use magba::special::cel;
use rand::Rng;

// Rust wrapper for MagpyLib
fn magpy_cel0(kc: f64, p: f64, c: f64, s: f64) -> f64 {
    pyfn!("magpylib._src.fields.special_cel", "cel0", (kc, p, c, s), f64)
}

// Tests
#[test]
fn test_cel() {
    let n = 1000;
    let mut rng = rand::thread_rng();

    for _ in 0..n {
        let kc = rng.gen_range(1.0..10.0);
        let p = rng.gen_range(-25.0..25.0);
        let c = rng.gen_range(-1.0..1.0);
        let s = rng.gen_range(-5.0..5.0);

        test!(cel, magpy_cel0, (kc, p, c, s));
    }
}
