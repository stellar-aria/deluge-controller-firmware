//! `deluge-fft` — portable-SIMD, no-std, const-generic radix-2 FFT.
//!
//! # Design
//!
//! * **No heap** — twiddle factors are computed inline; no global allocator needed.
//! * **Portable SIMD** — the butterfly inner loop operates on
//!   `core::simd::Simd<f32, LANES>`.  On Cortex-A9 with
//!   `-C target-feature=+neon`, LLVM emits `float32x4_t` instructions for
//!   `LANES = 4`.
//! * **Cooley-Tukey DIT radix-2** — standard in-place algorithm.
//!   Bit-reversal permutation applied first, then log₂(N) butterfly stages.
//! * **Const trig** — `cos`/`sin` implemented as Taylor-series `const fn`s
//!   over `f64`; no `libm` needed.

#![no_std]
#![feature(portable_simd)]

use core::ops::{Add, Mul, Sub};
use core::simd::Simd;

// ---------------------------------------------------------------------------
// sqrt shim — works in both no_std (bare-metal) and std (host tests)
// ---------------------------------------------------------------------------

/// `vsqrt.f32` / `fsqrts` on VFP targets; `sqrtss` on x86 SSE.
/// In no_std builds we call the `sqrtf` C symbol provided by compiler_builtins
/// (always present when targeting ARM with VFP/NEON).
/// In test builds std is linked so `f32::sqrt()` is available directly.
#[cfg(not(test))]
#[inline(always)]
fn sqrt_f32(x: f32) -> f32 {
    unsafe extern "C" {
        fn sqrtf(x: f32) -> f32;
    }
    // SAFETY: sqrtf is a pure IEEE 754 sqrt with no side-effects.
    unsafe { sqrtf(x) }
}

#[cfg(test)]
#[inline(always)]
fn sqrt_f32(x: f32) -> f32 {
    x.sqrt()
}

// ---------------------------------------------------------------------------
// Complex number
// ---------------------------------------------------------------------------

/// Complex number (re, im) in `f32`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct Complex {
    pub re: f32,
    pub im: f32,
}

impl Complex {
    pub const ZERO: Self = Self { re: 0.0, im: 0.0 };

    #[inline(always)]
    pub const fn new(re: f32, im: f32) -> Self {
        Self { re, im }
    }

    /// Squared magnitude (avoids `sqrt`).
    #[inline(always)]
    pub fn norm_sq(self) -> f32 {
        self.re * self.re + self.im * self.im
    }

    /// Magnitude.
    #[inline(always)]
    pub fn abs(self) -> f32 {
        sqrt_f32(self.norm_sq())
    }
}

impl Add for Complex {
    type Output = Self;
    #[inline(always)]
    fn add(self, rhs: Self) -> Self {
        Self { re: self.re + rhs.re, im: self.im + rhs.im }
    }
}

impl Sub for Complex {
    type Output = Self;
    #[inline(always)]
    fn sub(self, rhs: Self) -> Self {
        Self { re: self.re - rhs.re, im: self.im - rhs.im }
    }
}

impl Mul for Complex {
    type Output = Self;
    #[inline(always)]
    fn mul(self, rhs: Self) -> Self {
        Self {
            re: self.re * rhs.re - self.im * rhs.im,
            im: self.re * rhs.im + self.im * rhs.re,
        }
    }
}

impl From<f32> for Complex {
    #[inline(always)]
    fn from(re: f32) -> Self {
        Self { re, im: 0.0 }
    }
}

// ---------------------------------------------------------------------------
// Const trigonometry (Taylor series, f64 precision)
// ---------------------------------------------------------------------------

/// Taylor sin for `x` pre-reduced to `[-π/2, π/2]`.  Accurate to < 5 ulp there.
#[inline]
const fn sin_kernel(x: f64) -> f64 {
    let x2 = x * x;
    x * (1.0
        - x2 / 6.0
        + x2 * x2 / 120.0
        - x2 * x2 * x2 / 5040.0
        + x2 * x2 * x2 * x2 / 362880.0
        - x2 * x2 * x2 * x2 * x2 / 39916800.0
        + x2 * x2 * x2 * x2 * x2 * x2 / 6227020800.0)
}

/// `sin(x)` for any finite `x`.  Accurate to < 5 ulp.
/// Float→int cast in `const fn` is stable since Rust 1.83.
const fn sin_f32(x: f64) -> f32 {
    const PI: f64 = core::f64::consts::PI;
    const HALF_PI: f64 = PI / 2.0;
    // 1. Reduce to (-π, π] via integer truncation.
    let n = (x / TWO_PI) as i64;
    let mut r = x - n as f64 * TWO_PI;
    if r > PI {
        r -= TWO_PI;
    }
    if r < -PI {
        r += TWO_PI;
    }
    // 2. Reduce to [-π/2, π/2] using sin(π-r) = sin(r) symmetry.
    let v = if r > HALF_PI {
        sin_kernel(PI - r)
    } else if r < -HALF_PI {
        -sin_kernel(PI + r)
    } else {
        sin_kernel(r)
    };
    v as f32
}

/// `cos(x)` for any finite `x`.  Uses cos(x) = sin(π/2 − x).
#[inline]
const fn cos_f32(x: f64) -> f32 {
    sin_f32(core::f64::consts::FRAC_PI_2 - x)
}

const TWO_PI: f64 = 2.0 * core::f64::consts::PI;

// ---------------------------------------------------------------------------
// Bit-reversal helpers
// ---------------------------------------------------------------------------

const fn bit_rev(mut i: usize, mut bits: u32) -> usize {
    let mut r = 0usize;
    while bits > 0 {
        r = (r << 1) | (i & 1);
        i >>= 1;
        bits -= 1;
    }
    r
}

const fn ilog2(mut n: usize) -> usize {
    let mut r = 0;
    while n > 1 {
        n >>= 1;
        r += 1;
    }
    r
}

const fn assert_valid_size<const N: usize>() {
    assert!(N >= 2 && N.is_power_of_two(), "FFT size N must be a power of two >= 2");
}

// ---------------------------------------------------------------------------
// Twiddle factor helper
// ---------------------------------------------------------------------------

/// W_m^j = exp(-2πi·j/m).
#[inline(always)]
fn twiddle(j: usize, m: usize) -> Complex {
    let angle = -TWO_PI * (j as f64) / (m as f64);
    Complex { re: cos_f32(angle), im: sin_f32(angle) }
}

// ---------------------------------------------------------------------------
// Main FFT type
// ---------------------------------------------------------------------------

/// In-place radix-2 DIT FFT, `N` complex samples, `LANES` SIMD lanes.
///
/// The bound `Simd<f32, LANES>: Add + Sub + Mul + Copy` replaces the removed
/// `LaneCount<LANES>: SupportedLaneCount` as the SIMD constraint.
/// Valid `LANES` values: `1`, `2`, `4`, `8`.
pub struct Fft<const N: usize, const LANES: usize>;

impl<const N: usize, const LANES: usize> Fft<N, LANES>
where
    Simd<f32, LANES>: Add<Output = Simd<f32, LANES>>
        + Sub<Output = Simd<f32, LANES>>
        + Mul<Output = Simd<f32, LANES>>
        + Copy,
{
    /// Scalar in-place forward FFT.
    pub fn process(buf: &mut [Complex; N]) {
        assert_valid_size::<N>();
        Self::bit_reverse(buf);
        let log2n = ilog2(N);
        let mut half_m = 1usize;
        for _ in 0..log2n {
            let m = half_m * 2;
            let mut k = 0usize;
            while k < N {
                for j in 0..half_m {
                    let w = twiddle(j, m);
                    let u = buf[k + j];
                    let t = w * buf[k + j + half_m];
                    buf[k + j] = u + t;
                    buf[k + j + half_m] = u - t;
                }
                k += m;
            }
            half_m = m;
        }
    }

    /// SIMD-accelerated in-place forward FFT.
    ///
    /// Vectorises across `LANES` independent j-values per stage.
    /// Falls back to scalar for the small early stages where `half_m < LANES`.
    pub fn process_simd(buf: &mut [Complex; N]) {
        assert_valid_size::<N>();
        Self::bit_reverse(buf);
        let log2n = ilog2(N);
        let mut half_m = 1usize;
        for _ in 0..log2n {
            Self::simd_stage(buf, half_m);
            half_m *= 2;
        }
    }

    fn bit_reverse(buf: &mut [Complex; N]) {
        let bits = ilog2(N) as u32;
        for i in 0..N {
            let j = bit_rev(i, bits);
            if j > i {
                buf.swap(i, j);
            }
        }
    }

    fn simd_stage(buf: &mut [Complex; N], half_m: usize) {
        let m = half_m * 2;

        if half_m < LANES {
            // Scalar fallback for small stages.
            let mut k = 0usize;
            while k < N {
                for j in 0..half_m {
                    let w = twiddle(j, m);
                    let u = buf[k + j];
                    let t = w * buf[k + j + half_m];
                    buf[k + j] = u + t;
                    buf[k + j + half_m] = u - t;
                }
                k += m;
            }
            return;
        }

        let mut k = 0usize;
        while k < N {
            let mut j = 0usize;

            // SIMD: process LANES j-values simultaneously.
            while j + LANES <= half_m {
                let mut wr_arr = [0f32; LANES];
                let mut wi_arr = [0f32; LANES];
                let mut ur_arr = [0f32; LANES];
                let mut ui_arr = [0f32; LANES];
                let mut vr_arr = [0f32; LANES];
                let mut vi_arr = [0f32; LANES];

                for l in 0..LANES {
                    let w = twiddle(j + l, m);
                    wr_arr[l] = w.re;
                    wi_arr[l] = w.im;
                    let u = buf[k + j + l];
                    let v = buf[k + j + half_m + l];
                    ur_arr[l] = u.re;
                    ui_arr[l] = u.im;
                    vr_arr[l] = v.re;
                    vi_arr[l] = v.im;
                }

                let wr: Simd<f32, LANES> = Simd::from_array(wr_arr);
                let wi: Simd<f32, LANES> = Simd::from_array(wi_arr);
                let ur: Simd<f32, LANES> = Simd::from_array(ur_arr);
                let ui: Simd<f32, LANES> = Simd::from_array(ui_arr);
                let vr: Simd<f32, LANES> = Simd::from_array(vr_arr);
                let vi: Simd<f32, LANES> = Simd::from_array(vi_arr);

                let tr = wr * vr - wi * vi;
                let ti = wr * vi + wi * vr;

                let oar = (ur + tr).to_array();
                let oai = (ui + ti).to_array();
                let obr = (ur - tr).to_array();
                let obi = (ui - ti).to_array();

                for l in 0..LANES {
                    buf[k + j + l] = Complex { re: oar[l], im: oai[l] };
                    buf[k + j + half_m + l] = Complex { re: obr[l], im: obi[l] };
                }

                j += LANES;
            }

            // Scalar tail.
            while j < half_m {
                let w = twiddle(j, m);
                let u = buf[k + j];
                let t = w * buf[k + j + half_m];
                buf[k + j] = u + t;
                buf[k + j + half_m] = u - t;
                j += 1;
            }

            k += m;
        }
    }
}

// ---------------------------------------------------------------------------
// Spectrum helpers
// ---------------------------------------------------------------------------

/// Write `|bin|` for the first `N/2+1` bins into `out` (length ≥ `N/2+1`).
pub fn magnitude_spectrum<const N: usize>(fft_out: &[Complex; N], out: &mut [f32]) {
    let bins = (N / 2 + 1).min(out.len());
    for i in 0..bins {
        out[i] = fft_out[i].abs();
    }
}

/// Apply a Hann window to `buf` in-place before calling `process*`.
pub fn apply_hann_window<const N: usize>(buf: &mut [Complex; N]) {
    for (i, s) in buf.iter_mut().enumerate() {
        let w = (0.5 * (1.0 - cos_f32(TWO_PI * (i as f64) / (N as f64)) as f64)) as f32;
        s.re *= w;
        s.im *= w;
    }
}

// ---------------------------------------------------------------------------
// Test utilities
// ---------------------------------------------------------------------------

#[cfg(any(test, feature = "test-utils"))]
pub mod test_utils {
    use super::*;

    /// Naïve O(N²) DFT — reference for tests only.
    pub fn dft<const N: usize>(buf: &[Complex; N]) -> [Complex; N] {
        let mut out = [Complex::ZERO; N];
        for k in 0..N {
            let mut sum = Complex::ZERO;
            for n in 0..N {
                // Reduce k*n mod N so the angle stays in (-2π, 0]; the exponential
                // is periodic with period N so this is exact.
                let w = twiddle((k * n) % N, N);
                sum = sum + buf[n] * w;
            }
            out[k] = sum;
        }
        out
    }

    /// Maximum absolute error between two complex arrays.
    pub fn max_error<const N: usize>(a: &[Complex; N], b: &[Complex; N]) -> f32 {
        a.iter()
            .zip(b.iter())
            .map(|(x, y)| (Complex { re: x.re - y.re, im: x.im - y.im }).abs())
            .fold(0.0f32, f32::max)
    }
}

// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use test_utils::{dft, max_error};

    fn impulse<const N: usize>() -> [Complex; N] {
        let mut buf = [Complex::ZERO; N];
        buf[0] = Complex::new(1.0, 0.0);
        buf
    }

    fn sine<const N: usize>(freq_bin: usize) -> [Complex; N] {
        let mut buf = [Complex::ZERO; N];
        for i in 0..N {
            // Reduce phase integer-first so the angle is always in [0, 2π)
            // and our Taylor sin is accurate.
            let phase = (freq_bin * i) % N;
            buf[i].re = sin_f32(TWO_PI * phase as f64 / N as f64);
        }
        buf
    }

    #[test]
    fn impulse_scalar_8() {
        let mut buf = impulse::<8>();
        Fft::<8, 1>::process(&mut buf);
        for c in buf.iter() {
            assert!((c.re - 1.0).abs() < 1e-5, "re={}", c.re);
            assert!(c.im.abs() < 1e-5, "im={}", c.im);
        }
    }

    #[test]
    fn impulse_simd4_64() {
        let mut a = impulse::<64>();
        let mut b = a;
        Fft::<64, 4>::process_simd(&mut a);
        Fft::<64, 1>::process(&mut b);
        let err = max_error(&a, &b);
        assert!(err < 1e-4, "SIMD4 vs scalar: {err}");
    }

    #[test]
    fn sine_512_bin5_peak() {
        let mut buf = sine::<512>(5);
        Fft::<512, 4>::process_simd(&mut buf);
        let peak = buf
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.norm_sq().partial_cmp(&b.norm_sq()).unwrap())
            .map(|(i, _)| i)
            .unwrap();
        assert!(peak == 5 || peak == 507, "peak at {peak}");
    }

    #[test]
    fn simd_matches_scalar_512() {
        let base = sine::<512>(7);
        let mut a = base;
        let mut b = base;
        Fft::<512, 1>::process(&mut a);
        Fft::<512, 4>::process_simd(&mut b);
        let err = max_error(&a, &b);
        assert!(err < 5e-3, "SIMD/scalar mismatch: {err}");
    }

    #[test]
    fn simd_matches_dft_64() {
        let base = sine::<64>(3);
        let expected = dft(&base);
        let mut got = base;
        Fft::<64, 4>::process_simd(&mut got);
        let err = max_error(&got, &expected);
        assert!(err < 1e-3, "vs DFT: {err}");
    }

    #[test]
    fn hann_no_nan() {
        let mut buf = sine::<256>(10);
        apply_hann_window(&mut buf);
        Fft::<256, 4>::process_simd(&mut buf);
        for c in buf.iter() {
            assert!(c.re.is_finite() && c.im.is_finite());
        }
    }
}
