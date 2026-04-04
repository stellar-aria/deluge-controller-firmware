//! Error type mapping `arm_status` to Rust.

use cmsis_dsp_sys as sys;

/// Errors returned by CMSIS-DSP functions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// `ARM_MATH_ARGUMENT_ERROR` — invalid argument (e.g. unsupported FFT length).
    ArgumentError,
    /// `ARM_MATH_LENGTH_ERROR` — mismatched length.
    LengthError,
    /// `ARM_MATH_SIZE_MISMATCH` — incompatible sizes.
    SizeMismatch,
    /// `ARM_MATH_NANINF` — NaN or infinity in input.
    NanInf,
    /// `ARM_MATH_SINGULAR` — singular matrix.
    Singular,
    /// `ARM_MATH_TEST_FAILURE` — test failure.
    TestFailure,
    /// Unknown status code.
    Unknown(i32),
}

impl Error {
    pub(crate) fn from_status(s: sys::arm_status) -> Result<(), Self> {
        // arm_status is a C enum; CMSIS-DSP defines ARM_MATH_SUCCESS = 0.
        match s as i32 {
            0 => Ok(()),
            1 => Err(Error::ArgumentError),
            2 => Err(Error::LengthError),
            3 => Err(Error::SizeMismatch),
            4 => Err(Error::NanInf),
            5 => Err(Error::Singular),
            6 => Err(Error::TestFailure),
            n => Err(Error::Unknown(n)),
        }
    }
}
