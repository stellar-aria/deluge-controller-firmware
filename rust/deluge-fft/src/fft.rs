use core::ops::{Add, Mul, Sub};
use core::simd::Simd;

use crate::buf::FftBuf;
use crate::complex::Complex;
use crate::twiddle::TwiddleTableSoa;

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

const fn assert_valid_size<const N: usize>() {
    assert!(
        N >= 2 && N.is_power_of_two(),
        "FFT size N must be a power of two >= 2"
    );
}

pub(crate) const fn assert_valid_size_r4<const N: usize>() {
    assert!(
        N >= 4 && N.is_power_of_two(),
        "Radix-4 FFT size N must be a power of two >= 4"
    );
}

const fn bit_rev(mut i: usize, mut bits: u32) -> usize {
    let mut r = 0usize;
    while bits > 0 {
        r = (r << 1) | (i & 1);
        i >>= 1;
        bits -= 1;
    }
    r
}

pub(crate) const fn ilog2(mut n: usize) -> usize {
    let mut r = 0;
    while n > 1 {
        n >>= 1;
        r += 1;
    }
    r
}

fn bit_reverse_permutation_aos<const N: usize>(buf: &mut [Complex; N]) {
    let bits = ilog2(N) as u32;
    for i in 0..N {
        let j = bit_rev(i, bits);
        if j > i {
            buf.swap(i, j);
        }
    }
}

fn bit_reverse_permutation_soa<const N: usize>(buf: &mut FftBuf<N>) {
    let bits = ilog2(N) as u32;
    for i in 0..N {
        let j = bit_rev(i, bits);
        if j > i {
            buf.re.swap(i, j);
            buf.im.swap(i, j);
        }
    }
}

// ---------------------------------------------------------------------------
// Fft
// ---------------------------------------------------------------------------

/// In-place radix-2 DIT FFT over `N` complex `f32` samples.
///
/// Twiddle factors are evaluated entirely at compile time and stored in ROM.
/// The hot loop only reads a flat table and performs arithmetic — zero runtime
/// trig cost.
///
/// Valid `LANES` values: `1`, `2`, `4`, `8`.
///
/// # Methods
///
/// | Method | Buffer layout | SIMD |
/// |--------|--------------|------|
/// | `process` | AoS `[Complex; N]` | scalar |
/// | `process_simd` | AoS `[Complex; N]` | LANES-wide |
/// | `process_soa` | `FftBuf<N>` (SoA) | scalar |
/// | `process_simd_soa` | `FftBuf<N>` (SoA) | LANES-wide, sequential loads |
pub struct Fft<const N: usize, const LANES: usize>;

impl<const N: usize, const LANES: usize> Fft<N, LANES>
where
    Simd<f32, LANES>: Add<Output = Simd<f32, LANES>>
        + Sub<Output = Simd<f32, LANES>>
        + Mul<Output = Simd<f32, LANES>>
        + Copy,
{
    // -----------------------------------------------------------------------
    // AoS paths  (interleaved [Complex; N] buffer)
    // -----------------------------------------------------------------------

    /// Scalar in-place forward FFT on an AoS `[Complex; N]` buffer.
    pub fn process(buf: &mut [Complex; N]) {
        assert_valid_size::<N>();
        bit_reverse_permutation_aos(buf);
        let mut half_m = 1usize;
        while half_m < N {
            let tw_off = half_m - 1;
            let mut k = 0usize;
            while k < N {
                for j in 0..half_m {
                    let wr = TwiddleTableSoa::<N>::RE[tw_off + j];
                    let wi = TwiddleTableSoa::<N>::IM[tw_off + j];
                    let u = buf[k + j];
                    let v = buf[k + j + half_m];
                    let tr = wr * v.re - wi * v.im;
                    let ti = wr * v.im + wi * v.re;
                    buf[k + j] = Complex {
                        re: u.re + tr,
                        im: u.im + ti,
                    };
                    buf[k + j + half_m] = Complex {
                        re: u.re - tr,
                        im: u.im - ti,
                    };
                }
                k += half_m * 2;
            }
            half_m *= 2;
        }
    }

    /// SIMD-accelerated in-place forward FFT on an AoS `[Complex; N]` buffer.
    ///
    /// Vectorises across `LANES` independent j-values per stage.
    /// Falls back to scalar for stages where `half_m < LANES`.
    pub fn process_simd(buf: &mut [Complex; N]) {
        assert_valid_size::<N>();
        bit_reverse_permutation_aos(buf);
        let mut half_m = 1usize;
        while half_m < N {
            Self::simd_stage_aos(buf, half_m);
            half_m *= 2;
        }
    }

    fn simd_stage_aos(buf: &mut [Complex; N], half_m: usize) {
        let tw_off = half_m - 1;

        if half_m < LANES {
            // Scalar fallback — stage is narrower than one vector.
            let mut k = 0usize;
            while k < N {
                for j in 0..half_m {
                    let wr = TwiddleTableSoa::<N>::RE[tw_off + j];
                    let wi = TwiddleTableSoa::<N>::IM[tw_off + j];
                    let u = buf[k + j];
                    let v = buf[k + j + half_m];
                    let tr = wr * v.re - wi * v.im;
                    let ti = wr * v.im + wi * v.re;
                    buf[k + j] = Complex {
                        re: u.re + tr,
                        im: u.im + ti,
                    };
                    buf[k + j + half_m] = Complex {
                        re: u.re - tr,
                        im: u.im - ti,
                    };
                }
                k += half_m * 2;
            }
            return;
        }

        let mut k = 0usize;
        while k < N {
            let mut j = 0usize;

            while j + LANES <= half_m {
                // Sequential loads — no stride, no deinterleave needed.
                let wr = Simd::<f32, LANES>::from_slice(&TwiddleTableSoa::<N>::RE[tw_off + j..]);
                let wi = Simd::<f32, LANES>::from_slice(&TwiddleTableSoa::<N>::IM[tw_off + j..]);

                // AoS buf: extract re/im the old way (elements are interleaved).
                let mut ur_arr = [0f32; LANES];
                let mut ui_arr = [0f32; LANES];
                let mut vr_arr = [0f32; LANES];
                let mut vi_arr = [0f32; LANES];
                for l in 0..LANES {
                    let u = buf[k + j + l];
                    let v = buf[k + j + half_m + l];
                    ur_arr[l] = u.re;
                    ui_arr[l] = u.im;
                    vr_arr[l] = v.re;
                    vi_arr[l] = v.im;
                }
                let ur = Simd::<f32, LANES>::from_array(ur_arr);
                let ui = Simd::<f32, LANES>::from_array(ui_arr);
                let vr = Simd::<f32, LANES>::from_array(vr_arr);
                let vi = Simd::<f32, LANES>::from_array(vi_arr);

                let tr = wr * vr - wi * vi;
                let ti = wr * vi + wi * vr;

                let oar = (ur + tr).to_array();
                let oai = (ui + ti).to_array();
                let obr = (ur - tr).to_array();
                let obi = (ui - ti).to_array();
                for l in 0..LANES {
                    buf[k + j + l] = Complex {
                        re: oar[l],
                        im: oai[l],
                    };
                    buf[k + j + half_m + l] = Complex {
                        re: obr[l],
                        im: obi[l],
                    };
                }

                j += LANES;
            }

            // Scalar tail.
            while j < half_m {
                let wr = TwiddleTableSoa::<N>::RE[tw_off + j];
                let wi = TwiddleTableSoa::<N>::IM[tw_off + j];
                let u = buf[k + j];
                let v = buf[k + j + half_m];
                let tr = wr * v.re - wi * v.im;
                let ti = wr * v.im + wi * v.re;
                buf[k + j] = Complex {
                    re: u.re + tr,
                    im: u.im + ti,
                };
                buf[k + j + half_m] = Complex {
                    re: u.re - tr,
                    im: u.im - ti,
                };
                j += 1;
            }

            k += half_m * 2;
        }
    }

    // -----------------------------------------------------------------------
    // SoA paths  (FftBuf<N> — separate re[] and im[] arrays)
    // -----------------------------------------------------------------------

    /// Scalar in-place forward FFT on an `FftBuf<N>` (SoA) buffer.
    pub fn process_soa(buf: &mut FftBuf<N>) {
        assert_valid_size::<N>();
        bit_reverse_permutation_soa(buf);
        let mut half_m = 1usize;
        while half_m < N {
            let tw_off = half_m - 1;
            let mut k = 0usize;
            while k < N {
                for j in 0..half_m {
                    let wr = TwiddleTableSoa::<N>::RE[tw_off + j];
                    let wi = TwiddleTableSoa::<N>::IM[tw_off + j];
                    let ur = buf.re[k + j];
                    let ui = buf.im[k + j];
                    let vr = buf.re[k + j + half_m];
                    let vi = buf.im[k + j + half_m];
                    let tr = wr * vr - wi * vi;
                    let ti = wr * vi + wi * vr;
                    buf.re[k + j] = ur + tr;
                    buf.im[k + j] = ui + ti;
                    buf.re[k + j + half_m] = ur - tr;
                    buf.im[k + j + half_m] = ui - ti;
                }
                k += half_m * 2;
            }
            half_m *= 2;
        }
    }

    /// SIMD-accelerated in-place forward FFT on an `FftBuf<N>` (SoA) buffer.
    ///
    /// All twiddle and sample memory operations are sequential contiguous
    /// vector loads/stores — maximally vectorisation-friendly.
    pub fn process_simd_soa(buf: &mut FftBuf<N>) {
        assert_valid_size::<N>();
        bit_reverse_permutation_soa(buf);
        let mut half_m = 1usize;
        while half_m < N {
            Self::simd_stage_soa(buf, half_m);
            half_m *= 2;
        }
    }

    fn simd_stage_soa(buf: &mut FftBuf<N>, half_m: usize) {
        let tw_off = half_m - 1;

        if half_m < LANES {
            // Scalar fallback.
            let mut k = 0usize;
            while k < N {
                for j in 0..half_m {
                    let wr = TwiddleTableSoa::<N>::RE[tw_off + j];
                    let wi = TwiddleTableSoa::<N>::IM[tw_off + j];
                    let ur = buf.re[k + j];
                    let ui = buf.im[k + j];
                    let vr = buf.re[k + j + half_m];
                    let vi = buf.im[k + j + half_m];
                    let tr = wr * vr - wi * vi;
                    let ti = wr * vi + wi * vr;
                    buf.re[k + j] = ur + tr;
                    buf.im[k + j] = ui + ti;
                    buf.re[k + j + half_m] = ur - tr;
                    buf.im[k + j + half_m] = ui - ti;
                }
                k += half_m * 2;
            }
            return;
        }

        let mut k = 0usize;
        while k < N {
            let mut j = 0usize;

            while j + LANES <= half_m {
                // All 8 loads are sequential, aligned, contiguous.
                let wr = Simd::<f32, LANES>::from_slice(&TwiddleTableSoa::<N>::RE[tw_off + j..]);
                let wi = Simd::<f32, LANES>::from_slice(&TwiddleTableSoa::<N>::IM[tw_off + j..]);
                let ur = Simd::<f32, LANES>::from_slice(&buf.re[k + j..]);
                let ui = Simd::<f32, LANES>::from_slice(&buf.im[k + j..]);
                let vr = Simd::<f32, LANES>::from_slice(&buf.re[k + j + half_m..]);
                let vi = Simd::<f32, LANES>::from_slice(&buf.im[k + j + half_m..]);

                let tr = wr * vr - wi * vi;
                let ti = wr * vi + wi * vr;

                // All 4 stores are sequential, aligned, contiguous.
                (ur + tr).copy_to_slice(&mut buf.re[k + j..]);
                (ui + ti).copy_to_slice(&mut buf.im[k + j..]);
                (ur - tr).copy_to_slice(&mut buf.re[k + j + half_m..]);
                (ui - ti).copy_to_slice(&mut buf.im[k + j + half_m..]);

                j += LANES;
            }

            // Scalar tail.
            while j < half_m {
                let wr = TwiddleTableSoa::<N>::RE[tw_off + j];
                let wi = TwiddleTableSoa::<N>::IM[tw_off + j];
                let ur = buf.re[k + j];
                let ui = buf.im[k + j];
                let vr = buf.re[k + j + half_m];
                let vi = buf.im[k + j + half_m];
                let tr = wr * vr - wi * vi;
                let ti = wr * vi + wi * vr;
                buf.re[k + j] = ur + tr;
                buf.im[k + j] = ui + ti;
                buf.re[k + j + half_m] = ur - tr;
                buf.im[k + j + half_m] = ui - ti;
                j += 1;
            }

            k += half_m * 2;
        }
    }
}
