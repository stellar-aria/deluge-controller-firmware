/* Minimal wrapper header consumed by bindgen.
 *
 * -D__GNUC_PYTHON__    avoids the need for CMSIS-5 Core headers.
 * -DARM_MATH_NEON      selects the Neon struct layouts / function prototypes
 *                      (passed via bindgen clang_arg in build.rs when targeting ARM).
 * -DDISABLEFLOAT16     skips f16 types that require compiler-specific support.
 */
#include "arm_math.h"

/* arm_sqrt_f32 is __STATIC_FORCEINLINE in the CMSIS-DSP header; expose a
 * non-inline C shim so that Rust FFI can call it by address.             */
static inline arm_status cmsis_sqrt_f32(float32_t in, float32_t *pOut) {
    return arm_sqrt_f32(in, pOut);
}
