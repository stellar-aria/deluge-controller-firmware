use crate::complex::Complex;
use crate::trig::{TWO_PI, cos_f32, sin_f32};

// ---------------------------------------------------------------------------
// AoS twiddle table (used by test_utils::dft only)
// ---------------------------------------------------------------------------

/// Build the full natural-order twiddle table.  Entry `k` = `W_N^k`.
const fn make_twiddles_aos<const N: usize>() -> [Complex; N] {
    let mut table = [Complex::ZERO; N];
    let mut k = 0usize;
    while k < N {
        let angle = -TWO_PI * k as f64 / N as f64;
        table[k] = Complex { re: cos_f32(angle), im: sin_f32(angle) };
        k += 1;
    }
    table
}

/// Natural-order `W_N^k` table.  Only used by `test_utils::dft`.
pub(crate) struct TwiddleTable<const N: usize>;
impl<const N: usize> TwiddleTable<N> {
    pub(crate) const TABLE: [Complex; N] = make_twiddles_aos::<N>();
}

// ---------------------------------------------------------------------------
// Stage-sequential SoA twiddle table (used by all FFT hot paths)
// ---------------------------------------------------------------------------
//
// Layout: the table for size N has N-1 entries, split into two flat f32 arrays
// (real parts and imaginary parts).  Entries are laid out in stage order:
//
//   stage s (half_m = 2^s):  W_{2*half_m}^j  for j = 0..half_m
//   offset into array:        half_m - 1
//
//   [W_2^0 | W_4^0, W_4^1 | W_8^0..W_8^3 | W_16^0..W_16^7 | ...]
//    off=0      1       2       3   4  5  6     7  ...
//
// For stage s (half_m = 2^s), m = 2*half_m:
//   W_m^j = exp(-2πi·j/m)
//
// The SIMD hot loop fetches its LANES twiddles with two sequential slice reads:
//   Simd::from_slice(&TW_RE[half_m - 1 + j ..])
//   Simd::from_slice(&TW_IM[half_m - 1 + j ..])

const fn make_twiddles_soa_re<const N: usize>() -> [f32; N] {
    // N entries, but we only fill N-1 meaningful ones (indices 0..N-1).
    // Index half_m-1+j  (half_m in 1,2,4,...,N/2; j in 0..half_m)
    //   = W_{2*half_m}^j .re
    let mut table = [0f32; N];
    let mut half_m = 1usize;
    while half_m < N {
        let m = half_m * 2;
        let mut j = 0usize;
        while j < half_m {
            let angle = -TWO_PI * j as f64 / m as f64;
            table[half_m - 1 + j] = cos_f32(angle);
            j += 1;
        }
        half_m *= 2;
    }
    table
}

const fn make_twiddles_soa_im<const N: usize>() -> [f32; N] {
    let mut table = [0f32; N];
    let mut half_m = 1usize;
    while half_m < N {
        let m = half_m * 2;
        let mut j = 0usize;
        while j < half_m {
            let angle = -TWO_PI * j as f64 / m as f64;
            table[half_m - 1 + j] = sin_f32(angle);
            j += 1;
        }
        half_m *= 2;
    }
    table
}

/// Stage-sequential SoA twiddle tables.
///
/// For stage with `half_m`, the twiddles start at index `half_m - 1`:
/// ```ignore
/// let wr = Simd::from_slice(&TwiddleTableSoa::<N>::RE[half_m - 1 + j ..]);
/// let wi = Simd::from_slice(&TwiddleTableSoa::<N>::IM[half_m - 1 + j ..]);
/// ```
pub(crate) struct TwiddleTableSoa<const N: usize>;

impl<const N: usize> TwiddleTableSoa<N> {
    pub(crate) const RE: [f32; N] = make_twiddles_soa_re::<N>();
    pub(crate) const IM: [f32; N] = make_twiddles_soa_im::<N>();
}
