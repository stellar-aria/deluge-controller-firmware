#pragma once

#include <stdint.h>

// ---------------------------------------------------------------------------
// Conditional debug logging
//
// Enable verbose boot / loop tracing by defining CONTROLLER_DEBUG at compile
// time (e.g. -DCONTROLLER_DEBUG).  In release builds every CDBG_* call
// compiles away to nothing.
// ---------------------------------------------------------------------------

#ifdef ENABLE_RTT
// Forward-declare the two RTT functions used by the macros so this header
// can be included from both C and C++ translation units without pulling in
// the full SEGGER_RTT.h (which has its own extern-C guards).
#ifdef __cplusplus
extern "C" {
#endif
unsigned SEGGER_RTT_WriteString(unsigned BufferIndex, const char* pBuffer);
int      SEGGER_RTT_printf(unsigned BufferIndex, const char* sFormat, ...);
#ifdef __cplusplus
}
#endif
#define CDBG_STR(s)          SEGGER_RTT_WriteString(0, s)
#define CDBG_FMT(fmt, ...)   SEGGER_RTT_printf(0, fmt, ##__VA_ARGS__)
#else
#define CDBG_STR(s)          ((void)0)
#define CDBG_FMT(fmt, ...)   ((void)0)
#endif
