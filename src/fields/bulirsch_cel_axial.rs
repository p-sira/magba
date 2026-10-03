/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use ellip::cel;
use num_traits::Float;
use numeric_literals::replace_float_literals;

num_lazy::declare_nums! {@constant T}

/// Fused Bulirsch CEL evaluation for axial coordinates:
/// `(cel(kc, 1.0, 1.0, -1.0), cel(kc, gamma^2, 1.0, gamma))`
///
/// This routine takes two complementary moduli with the same `gamma`
/// and shares the computation loop.
#[inline]
pub(crate) fn cel_axial_pair<T: Float>(kc_p: T, kc_m: T, gamma: T) -> ((T, T), (T, T)) {
    let mut state_p = cel_axial_state(kc_p, gamma);
    let mut state_m = cel_axial_state(kc_m, gamma);
    let mut ans_p = (T::zero(), T::zero());
    let mut ans_m = (T::zero(), T::zero());
    let mut done_p = false;
    let mut done_m = false;

    for _ in 0..CEL_AXIAL_MAX_ITER {
        if !done_p && let Some(ans) = cel_axial_step(&mut state_p) {
            ans_p = ans;
            done_p = true;
        }
        if !done_m && let Some(ans) = cel_axial_step(&mut state_m) {
            ans_m = ans;
            done_m = true;
        }
        if done_p && done_m {
            return (ans_p, ans_m);
        }
    }

    if !done_p {
        ans_p = cel_axial_fallback(state_p.kc, gamma);
    }
    if !done_m {
        ans_m = cel_axial_fallback(state_m.kc, gamma);
    }
    (ans_p, ans_m)
}

const CEL_AXIAL_MAX_ITER: usize = 10;

struct CelAxialState<T> {
    kc: T,
    e: T,
    m: T,
    aa_r: T,
    bb_r: T,
    c_r: T,
    aa_z: T,
    bb_z: T,
    pp_z: T,
}

#[inline]
#[replace_float_literals(T::from(literal).unwrap())]
fn cel_axial_state<T: Float>(kc: T, gamma: T) -> CelAxialState<T> {
    let kc = Float::abs(kc);
    let pp_z = Float::abs(gamma);
    CelAxialState {
        kc,
        e: kc,
        m: 1.0,
        aa_r: 0.0,
        bb_r: -1.0,
        c_r: 1.0,
        aa_z: 1.0,
        bb_z: gamma / pp_z,
        pp_z,
    }
}

/// One Landen step; returns the fused pair when this stream has converged.
#[inline]
#[replace_float_literals(T::from(literal).unwrap())]
fn cel_axial_step<T: Float>(state: &mut CelAxialState<T>) -> Option<(T, T)> {
    let ca = if core::mem::size_of::<T>() <= 4 {
        1e-3
    } else {
        1e-8
    };
    let mut kc = state.kc;

    state.bb_r = (state.c_r * kc + state.bb_r) * 2.0;
    state.c_r = state.aa_r;

    let f_z = state.aa_z;
    let inv_pp_z = T::one() / state.pp_z;
    state.aa_z = state.bb_z * inv_pp_z + state.aa_z;
    let g_z = state.e * inv_pp_z;
    state.bb_z = 2.0 * (f_z * g_z + state.bb_z);
    state.pp_z = g_z + state.pp_z;

    let m0 = state.m;
    state.m = kc + state.m;
    state.aa_r = state.bb_r / state.m + state.aa_r;

    if Float::abs(m0 - kc) > m0 * ca {
        kc = 2.0 * Float::sqrt(state.e);
        state.e = kc * state.m;
        state.kc = kc;
        return None;
    }

    let ans_r = pi!() / 4.0 * state.aa_r / state.m;
    let ans_z =
        (pi!() / 2.0) * (state.aa_z * state.m + state.bb_z) / (state.m * (state.m + state.pp_z));
    Some((ans_r, ans_z))
}

#[cold]
#[replace_float_literals(T::from(literal).unwrap())]
fn cel_axial_fallback<T: Float>(kc: T, gamma: T) -> (T, T) {
    (
        cel(kc, 1.0, 1.0, -1.0).unwrap(),
        cel(kc, gamma * gamma, 1.0, gamma).unwrap(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Fused Bulirsch evaluation for axial coordinates:
    /// `(cel(kc, 1.0, 1.0, -1.0), cel(kc, gamma^2, 1.0, gamma))`
    /// simultaneously by sharing the common AGM/Landen sequence for complementary modulus `kc`.
    #[inline]
    fn cel_axial<T: Float>(kc: T, gamma: T) -> (T, T) {
        let mut state = cel_axial_state(kc, gamma);
        for _ in 0..CEL_AXIAL_MAX_ITER {
            if let Some(ans) = cel_axial_step(&mut state) {
                return ans;
            }
        }
        cel_axial_fallback(state.kc, gamma)
    }

    #[test]
    fn test_cel_axial_pair() {
        let gamma = 0.7f64;
        let pairs = [(0.3, 0.5), (0.9, 0.2), (0.15, 0.85), (0.99, 0.4)];
        for (kp, km) in pairs {
            let ((f_pr, f_pz), (f_mr, f_mz)) = cel_axial_pair(kp, km, gamma);
            let (s_pr, s_pz) = cel_axial(kp, gamma);
            let (s_mr, s_mz) = cel_axial(km, gamma);
            approx::assert_relative_eq!(f_pr, s_pr, epsilon = 1e-14);
            approx::assert_relative_eq!(f_pz, s_pz, epsilon = 1e-14);
            approx::assert_relative_eq!(f_mr, s_mr, epsilon = 1e-14);
            approx::assert_relative_eq!(f_mz, s_mz, epsilon = 1e-14);

            let ref_p = (
                cel(kp, 1.0, 1.0, -1.0).unwrap(),
                cel(kp, gamma * gamma, 1.0, gamma).unwrap(),
            );
            let ref_m = (
                cel(km, 1.0, 1.0, -1.0).unwrap(),
                cel(km, gamma * gamma, 1.0, gamma).unwrap(),
            );
            approx::assert_relative_eq!(f_pr, ref_p.0, epsilon = 1e-12);
            approx::assert_relative_eq!(f_pz, ref_p.1, epsilon = 1e-12);
            approx::assert_relative_eq!(f_mr, ref_m.0, epsilon = 1e-12);
            approx::assert_relative_eq!(f_mz, ref_m.1, epsilon = 1e-12);
        }
    }
}
