//! Basic and fast-math operations with NEON acceleration.

use cmsis_dsp_sys as sys;

// ── Vector operations ─────────────────────────────────────────────────────────

/// Element-wise f32 addition: `dst[i] = a[i] + b[i]`.
pub fn add_f32(a: &[f32], b: &[f32], dst: &mut [f32]) {
    let n = dst.len() as u32;
    unsafe { sys::arm_add_f32(a.as_ptr(), b.as_ptr(), dst.as_mut_ptr(), n) }
}

/// Element-wise f32 multiplication: `dst[i] = a[i] * b[i]`.
pub fn mult_f32(a: &[f32], b: &[f32], dst: &mut [f32]) {
    let n = dst.len() as u32;
    unsafe { sys::arm_mult_f32(a.as_ptr(), b.as_ptr(), dst.as_mut_ptr(), n) }
}

/// Element-wise f32 subtraction: `dst[i] = a[i] - b[i]`.
pub fn sub_f32(a: &[f32], b: &[f32], dst: &mut [f32]) {
    let n = dst.len() as u32;
    unsafe { sys::arm_sub_f32(a.as_ptr(), b.as_ptr(), dst.as_mut_ptr(), n) }
}

/// Scale a vector: `dst[i] = src[i] * scale`.
pub fn scale_f32(src: &[f32], scale: f32, dst: &mut [f32]) {
    let n = dst.len() as u32;
    unsafe { sys::arm_scale_f32(src.as_ptr(), scale, dst.as_mut_ptr(), n) }
}

/// Negate a vector: `dst[i] = -src[i]`.
pub fn negate_f32(src: &[f32], dst: &mut [f32]) {
    let n = dst.len() as u32;
    unsafe { sys::arm_negate_f32(src.as_ptr(), dst.as_mut_ptr(), n) }
}

/// Dot product of two f32 vectors.
pub fn dot_prod_f32(a: &[f32], b: &[f32]) -> f32 {
    let mut result = 0.0f32;
    let n = a.len() as u32;
    unsafe { sys::arm_dot_prod_f32(a.as_ptr(), b.as_ptr(), n, &mut result) }
    result
}

/// Absolute value: `dst[i] = |src[i]|`.
pub fn abs_f32(src: &[f32], dst: &mut [f32]) {
    let n = dst.len() as u32;
    unsafe { sys::arm_abs_f32(src.as_ptr(), dst.as_mut_ptr(), n) }
}

// ── Fast math ─────────────────────────────────────────────────────────────────

/// Fast sine / cosine via the CMSIS table-based approximation.
/// Returns `(sin(theta_deg), cos(theta_deg))` where theta is in degrees.
#[inline]
pub fn sin_cos_f32(theta: f32) -> (f32, f32) {
    let mut s = 0.0f32;
    let mut c = 0.0f32;
    unsafe { sys::arm_sin_cos_f32(theta, &mut s, &mut c) };
    (s, c)
}

/// Table-based approximate sine (input in radians).
#[inline]
pub fn sin_f32(x: f32) -> f32 {
    unsafe { sys::arm_sin_f32(x) }
}

/// Table-based approximate cosine (input in radians).
#[inline]
pub fn cos_f32(x: f32) -> f32 {
    unsafe { sys::arm_cos_f32(x) }
}
