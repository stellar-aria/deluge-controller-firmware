//! Statistics functions with NEON acceleration.

use cmsis_dsp_sys as sys;

/// Maximum value and index in an f32 slice.
pub fn max_f32(src: &[f32]) -> (f32, u32) {
    let mut val = 0.0f32;
    let mut idx = 0u32;
    unsafe { sys::arm_max_f32(src.as_ptr(), src.len() as u32, &mut val, &mut idx) }
    (val, idx)
}

/// Minimum value and index in an f32 slice.
pub fn min_f32(src: &[f32]) -> (f32, u32) {
    let mut val = 0.0f32;
    let mut idx = 0u32;
    unsafe { sys::arm_min_f32(src.as_ptr(), src.len() as u32, &mut val, &mut idx) }
    (val, idx)
}

/// Mean (average) of an f32 slice.
pub fn mean_f32(src: &[f32]) -> f32 {
    let mut result = 0.0f32;
    unsafe { sys::arm_mean_f32(src.as_ptr(), src.len() as u32, &mut result) }
    result
}

/// RMS (root mean square) of an f32 slice.
pub fn rms_f32(src: &[f32]) -> f32 {
    let mut result = 0.0f32;
    unsafe { sys::arm_rms_f32(src.as_ptr(), src.len() as u32, &mut result) }
    result
}

/// Variance of an f32 slice.
pub fn var_f32(src: &[f32]) -> f32 {
    let mut result = 0.0f32;
    unsafe { sys::arm_var_f32(src.as_ptr(), src.len() as u32, &mut result) }
    result
}

/// Standard deviation of an f32 slice.
pub fn std_f32(src: &[f32]) -> f32 {
    let mut result = 0.0f32;
    unsafe { sys::arm_std_f32(src.as_ptr(), src.len() as u32, &mut result) }
    result
}

/// Power (sum of squares) of an f32 slice.
pub fn power_f32(src: &[f32]) -> f32 {
    let mut result = 0.0f32;
    unsafe { sys::arm_power_f32(src.as_ptr(), src.len() as u32, &mut result) }
    result
}
