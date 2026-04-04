//! Complex and real FFT wrappers.
//!
//! Two flavours are provided, selected automatically at build time:
//!
//! * **NEON path** (`cfg(target_arch = "arm")` — set when the C library was compiled
//!   with `-DARM_MATH_NEON`, i.e. on ARM targets with the `neon` feature):
//!   the API is **out-of-place** and requires an extra temporary buffer as
//!   documented in the CMSIS-DSP README.
//!
//! * **Scalar path**: the classic **in-place** API (all other targets).
//!
//! Use the size-specific constructors (e.g. [`CfftF32::new_256`]) to allow the
//! linker to dead-strip unused twiddle tables.

use cmsis_dsp_sys as sys;

use crate::Error;

// ── helper ────────────────────────────────────────────────────────────────────

#[inline(always)]
fn check(s: sys::arm_status) -> Result<(), Error> {
    Error::from_status(s)
}

// ── Complex FFT (f32) ────────────────────────────────────────────────────────

/// Owning wrapper around `arm_cfft_instance_f32`.
///
/// The struct layout differs between NEON and scalar builds; always construct
/// via one of the `new_*` methods.
pub struct CfftF32(sys::arm_cfft_instance_f32);

// arm_cfft_instance_f32 holds only plain data (no thread-local bookkeeping).
unsafe impl Send for CfftF32 {}

impl CfftF32 {
    // ── size-specific constructors (linker-friendly) ─────────────────────────

    pub fn new_16() -> Result<Self, Error> {
        let mut s = unsafe { core::mem::zeroed() };
        check(unsafe { sys::arm_cfft_init_16_f32(&mut s) })?;
        Ok(Self(s))
    }
    pub fn new_32() -> Result<Self, Error> {
        let mut s = unsafe { core::mem::zeroed() };
        check(unsafe { sys::arm_cfft_init_32_f32(&mut s) })?;
        Ok(Self(s))
    }
    pub fn new_64() -> Result<Self, Error> {
        let mut s = unsafe { core::mem::zeroed() };
        check(unsafe { sys::arm_cfft_init_64_f32(&mut s) })?;
        Ok(Self(s))
    }
    pub fn new_128() -> Result<Self, Error> {
        let mut s = unsafe { core::mem::zeroed() };
        check(unsafe { sys::arm_cfft_init_128_f32(&mut s) })?;
        Ok(Self(s))
    }
    pub fn new_256() -> Result<Self, Error> {
        let mut s = unsafe { core::mem::zeroed() };
        check(unsafe { sys::arm_cfft_init_256_f32(&mut s) })?;
        Ok(Self(s))
    }
    pub fn new_512() -> Result<Self, Error> {
        let mut s = unsafe { core::mem::zeroed() };
        check(unsafe { sys::arm_cfft_init_512_f32(&mut s) })?;
        Ok(Self(s))
    }
    pub fn new_1024() -> Result<Self, Error> {
        let mut s = unsafe { core::mem::zeroed() };
        check(unsafe { sys::arm_cfft_init_1024_f32(&mut s) })?;
        Ok(Self(s))
    }
    pub fn new_2048() -> Result<Self, Error> {
        let mut s = unsafe { core::mem::zeroed() };
        check(unsafe { sys::arm_cfft_init_2048_f32(&mut s) })?;
        Ok(Self(s))
    }
    pub fn new_4096() -> Result<Self, Error> {
        let mut s = unsafe { core::mem::zeroed() };
        check(unsafe { sys::arm_cfft_init_4096_f32(&mut s) })?;
        Ok(Self(s))
    }

    /// Generic constructor — links all twiddle tables up to 4096 points.
    /// Prefer size-specific constructors when the length is compile-time constant.
    pub fn new(fft_len: u16) -> Result<Self, Error> {
        let mut s = unsafe { core::mem::zeroed() };
        check(unsafe { sys::arm_cfft_init_f32(&mut s, fft_len) })?;
        Ok(Self(s))
    }

    // ── NEON out-of-place API ─────────────────────────────────────────────────
    //
    // arm_cfft_f32(&S, pIn, pOut, pBuffer, ifftFlag)
    // pIn, pOut, pBuffer must all be distinct slices of equal length (2*N).

    /// Forward complex FFT — NEON out-of-place variant.
    ///
    /// `src`, `dst`, and `buf` must be three distinct slices of length `2 * N`.
    #[cfg(target_arch = "arm")]
    #[inline]
    pub fn process_forward(&self, src: &[f32], dst: &mut [f32], buf: &mut [f32]) {
        unsafe {
            sys::arm_cfft_f32(&self.0, src.as_ptr(), dst.as_mut_ptr(), buf.as_mut_ptr(), 0);
        }
    }

    /// Inverse complex FFT — NEON out-of-place variant.
    #[cfg(target_arch = "arm")]
    #[inline]
    pub fn process_inverse(&self, src: &[f32], dst: &mut [f32], buf: &mut [f32]) {
        unsafe {
            sys::arm_cfft_f32(&self.0, src.as_ptr(), dst.as_mut_ptr(), buf.as_mut_ptr(), 1);
        }
    }

    // ── Scalar in-place API ───────────────────────────────────────────────────
    //
    // arm_cfft_f32(&S, p1, ifftFlag, bitReverseFlag)
    // p1 is both input and output.

    /// Forward complex FFT — scalar in-place variant.
    ///
    /// `data` is `[re0, im0, …]` and is modified in place.
    #[cfg(not(target_arch = "arm"))]
    #[inline]
    pub fn process_forward(&self, data: &mut [f32]) {
        unsafe { sys::arm_cfft_f32(&self.0, data.as_mut_ptr(), 0, 1) }
    }

    /// Inverse complex FFT — scalar in-place variant.
    #[cfg(not(target_arch = "arm"))]
    #[inline]
    pub fn process_inverse(&self, data: &mut [f32]) {
        unsafe { sys::arm_cfft_f32(&self.0, data.as_mut_ptr(), 1, 1) }
    }
}

// ── Real FFT fast (f32) ───────────────────────────────────────────────────────

/// Owning wrapper around `arm_rfft_fast_instance_f32`.
pub struct RfftF32(sys::arm_rfft_fast_instance_f32);

unsafe impl Send for RfftF32 {}

impl RfftF32 {
    pub fn new_32() -> Result<Self, Error> {
        let mut s = unsafe { core::mem::zeroed() };
        check(unsafe { sys::arm_rfft_fast_init_32_f32(&mut s) })?;
        Ok(Self(s))
    }
    pub fn new_64() -> Result<Self, Error> {
        let mut s = unsafe { core::mem::zeroed() };
        check(unsafe { sys::arm_rfft_fast_init_64_f32(&mut s) })?;
        Ok(Self(s))
    }
    pub fn new_128() -> Result<Self, Error> {
        let mut s = unsafe { core::mem::zeroed() };
        check(unsafe { sys::arm_rfft_fast_init_128_f32(&mut s) })?;
        Ok(Self(s))
    }
    pub fn new_256() -> Result<Self, Error> {
        let mut s = unsafe { core::mem::zeroed() };
        check(unsafe { sys::arm_rfft_fast_init_256_f32(&mut s) })?;
        Ok(Self(s))
    }
    pub fn new_512() -> Result<Self, Error> {
        let mut s = unsafe { core::mem::zeroed() };
        check(unsafe { sys::arm_rfft_fast_init_512_f32(&mut s) })?;
        Ok(Self(s))
    }
    pub fn new_1024() -> Result<Self, Error> {
        let mut s = unsafe { core::mem::zeroed() };
        check(unsafe { sys::arm_rfft_fast_init_1024_f32(&mut s) })?;
        Ok(Self(s))
    }
    pub fn new_2048() -> Result<Self, Error> {
        let mut s = unsafe { core::mem::zeroed() };
        check(unsafe { sys::arm_rfft_fast_init_2048_f32(&mut s) })?;
        Ok(Self(s))
    }
    pub fn new_4096() -> Result<Self, Error> {
        let mut s = unsafe { core::mem::zeroed() };
        check(unsafe { sys::arm_rfft_fast_init_4096_f32(&mut s) })?;
        Ok(Self(s))
    }

    pub fn new(fft_len: u16) -> Result<Self, Error> {
        let mut s = unsafe { core::mem::zeroed() };
        check(unsafe { sys::arm_rfft_fast_init_f32(&mut s, fft_len) })?;
        Ok(Self(s))
    }

    // ── NEON API ──────────────────────────────────────────────────────────────

    /// Forward real FFT — NEON out-of-place variant.
    ///
    /// * `src` — N real input samples.
    /// * `dst` — N-element output (packed complex spectrum, CMSIS convention).
    /// * `buf` — scratch of N elements.
    #[cfg(target_arch = "arm")]
    #[inline]
    pub fn process_forward(&self, src: &[f32], dst: &mut [f32], buf: &mut [f32]) {
        unsafe {
            sys::arm_rfft_fast_f32(&self.0, src.as_ptr(), dst.as_mut_ptr(), buf.as_mut_ptr(), 0);
        }
    }

    /// Inverse real FFT — NEON out-of-place variant.
    #[cfg(target_arch = "arm")]
    #[inline]
    pub fn process_inverse(&self, src: &[f32], dst: &mut [f32], buf: &mut [f32]) {
        unsafe {
            sys::arm_rfft_fast_f32(&self.0, src.as_ptr(), dst.as_mut_ptr(), buf.as_mut_ptr(), 1);
        }
    }

    // ── Scalar API ────────────────────────────────────────────────────────────

    /// Forward real FFT — scalar variant.
    ///
    /// `p` real input (consumed), `p_out` packed complex output.
    #[cfg(not(target_arch = "arm"))]
    #[inline]
    pub fn process_forward(&self, p: &mut [f32], p_out: &mut [f32]) {
        unsafe { sys::arm_rfft_fast_f32(&self.0, p.as_mut_ptr(), p_out.as_mut_ptr(), 0) }
    }

    /// Inverse real FFT — scalar variant.
    #[cfg(not(target_arch = "arm"))]
    #[inline]
    pub fn process_inverse(&self, p: &mut [f32], p_out: &mut [f32]) {
        unsafe { sys::arm_rfft_fast_f32(&self.0, p.as_mut_ptr(), p_out.as_mut_ptr(), 1) }
    }
}

// ── Magnitude helpers ─────────────────────────────────────────────────────────

/// Compute magnitudes from a complex interleaved spectrum.
///
/// `cmplx` — `[re0, im0, re1, im1, …]` of length `2 * mag.len()`.
/// `mag`   — output magnitudes, one per bin.
pub fn cmplx_mag_f32(cmplx: &[f32], mag: &mut [f32]) {
    unsafe {
        sys::arm_cmplx_mag_f32(cmplx.as_ptr(), mag.as_mut_ptr(), mag.len() as u32);
    }
}

/// Squared magnitude (avoids a square root).
pub fn cmplx_mag_squared_f32(cmplx: &[f32], mag_sq: &mut [f32]) {
    unsafe {
        sys::arm_cmplx_mag_squared_f32(cmplx.as_ptr(), mag_sq.as_mut_ptr(), mag_sq.len() as u32);
    }
}
