/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use ellip::cel;
use num_traits::Float;
use numeric_literals::replace_float_literals;

num_lazy::declare_nums! {@constant T}

/// Fused Bulirsch evaluation for axial coordinates:
/// `(cel(kc, 1.0, 1.0, -1.0), cel(kc, gamma^2, 1.0, gamma))`
/// simultaneously by sharing the common AGM/Landen sequence for complementary modulus `kc`.
#[inline]
#[replace_float_literals(T::from(literal).unwrap())]
pub(crate) fn cel_axial<T: Float>(kc: T, gamma: T) -> (T, T) {
    let mut kc = Float::abs(kc);
    let mut aa_r: T = 0.0;
    let mut bb_r: T = -1.0;
    let mut c_r: T = 1.0;
    let pp_z: T = Float::abs(gamma);
    let mut aa_z: T = 1.0;
    let mut bb_z: T = gamma / pp_z;
    let mut pp_z: T = pp_z;
    let mut e: T = kc;
    let mut m: T = 1.0;

    let ca = if core::mem::size_of::<T>() <= 4 {
        1e-3
    } else {
        1e-8
    };

    for _ in 0..MAX_ITER {
        bb_r = (c_r * kc + bb_r) * 2.0;
        c_r = aa_r;
        let f_z = aa_z;
        let inv_pp_z = 1.0 / pp_z;
        aa_z = bb_z * inv_pp_z + aa_z;
        let g_z = e * inv_pp_z;
        bb_z = 2.0 * (f_z * g_z + bb_z);
        pp_z = g_z + pp_z;
        let m0 = m;
        m = kc + m;
        aa_r = bb_r / m + aa_r;
        if Float::abs(m0 - kc) > m0 * ca {
            kc = 2.0 * Float::sqrt(e);
            e = kc * m;
            continue;
        }
        let ans_r = pi!() / 4.0 * aa_r / m;
        let ans_z = (pi!() / 2.0) * (aa_z * m + bb_z) / (m * (m + pp_z));
        return (ans_r, ans_z);
    }
    (
        cel(kc, 1.0, 1.0, -1.0).unwrap(),
        cel(kc, gamma * gamma, 1.0, gamma).unwrap(),
    )
}

#[cfg(not(feature = "test_force_fail"))]
const MAX_ITER: usize = 16;

#[cfg(feature = "test_force_fail")]
const MAX_ITER: usize = 1;

#[cfg(not(feature = "test_force_fail"))]
#[cfg(test)]
mod tests {
    #[test]
    fn test_cel_axial() {
        let (r, z) = super::cel_axial(0.3, 0.7);
        let ref_r = ellip::cel(0.3, 1.0, 1.0, -1.0).unwrap();
        let ref_z = ellip::cel(0.3, 0.49, 1.0, 0.7).unwrap();
        approx::assert_relative_eq!(r, ref_r, epsilon = 1e-14);
        approx::assert_relative_eq!(z, ref_z, epsilon = 1e-14);
    }
}

#[cfg(feature = "test_force_fail")]
crate::test_force_unreachable! {
    let _ = cel_axial(0.3, 0.7);
}
