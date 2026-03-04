// DelugeASIO — Standalone Test Application
// Instantiates the driver directly (no COM registration needed) and runs
// a short audio loopback test, printing diagnostics along the way.
//
// Usage:  asio_test.exe [duration_seconds]
//   Default duration: 5 seconds
//
// What it does:
//   1. Creates the DelugeASIO driver instance
//   2. Queries device info (channels, buffer sizes, sample rate)
//   3. Creates ASIO buffers (all inputs + all outputs)
//   4. Starts streaming — audio from mic IN is looped back to speaker OUT
//   5. Prints statistics every second (buffer switches, sample position)
//   6. Stops and cleans up

#include <windows.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <atomic>

// We include the driver header directly — no COM needed for testing
#include "asio_driver.h"
#include "usb_transport.h"
#include "deluge_usb.h"

// ============================================================================
// Globals for ASIO callbacks
// ============================================================================

static std::atomic<long>   g_bufferSwitchCount{0};
static std::atomic<double> g_samplePos{0.0};
static long                g_bufferSize = 0;
static ASIOBufferInfo*     g_bufferInfos = nullptr;
static long                g_numIn = 0;
static long                g_numOut = 0;

// Sine tone state
static double              g_phase = 0.0;
static const double        g_toneFreq = 440.0;  // A4
static const double        g_amplitude = 0.25;   // -12 dB
static double              g_sampleRate = 44100.0;

// ============================================================================
// Tone Generator — writes a 440 Hz sine to all output buffers
// ============================================================================

static void generateTone(long doubleBufferIndex) {
    // Find output buffer infos (they follow the input channels)
    for (long ch = 0; ch < g_numOut; ch++) {
        long idx = g_numIn + ch; // output channels come after inputs
        int32_t* buf = static_cast<int32_t*>(g_bufferInfos[idx].buffers[doubleBufferIndex]);
        if (!buf) continue;

        double phase = g_phase;
        for (long s = 0; s < g_bufferSize; s++) {
            double sample = g_amplitude * sin(phase);
            // ASIOSTInt32LSB: 24-bit left-justified in 32-bit
            // Full scale = 0x7FFFFF00
            int32_t val = static_cast<int32_t>(sample * 2147483392.0); // 0x7FFFFF00
            buf[s] = val;
            phase += 2.0 * 3.14159265358979323846 * g_toneFreq / g_sampleRate;
        }
    }
    // Advance phase once (same for all channels — mono tone)
    double phaseInc = 2.0 * 3.14159265358979323846 * g_toneFreq / g_sampleRate;
    g_phase += phaseInc * g_bufferSize;
    // Keep phase in [0, 2*PI) to avoid precision loss
    while (g_phase >= 2.0 * 3.14159265358979323846)
        g_phase -= 2.0 * 3.14159265358979323846;
}

// ============================================================================
// ASIO Callbacks
// ============================================================================

static void bufferSwitch(long doubleBufferIndex, ASIOBool directProcess) {
    g_bufferSwitchCount.fetch_add(1, std::memory_order_relaxed);
    generateTone(doubleBufferIndex);
    (void)directProcess;
}

static void sampleRateDidChange(ASIOSampleRate sRate) {
    printf("  [callback] Sample rate changed to %.0f\n", sRate);
}

static long asioMessage(long selector, long value, void* message, double* opt) {
    switch (selector) {
        case kAsioSelectorSupported:
            // Tell host which selectors we support
            if (value == kAsioEngineVersion) return 1;
            if (value == kAsioSupportsTimeInfo) return 1;
            return 0;

        case kAsioEngineVersion:
            return 2; // ASIO 2.0

        case kAsioSupportsTimeInfo:
            return 1;

        default:
            return 0;
    }
}

static ASIOTime* bufferSwitchTimeInfo(ASIOTime* params, long doubleBufferIndex, ASIOBool directProcess) {
    g_bufferSwitchCount.fetch_add(1, std::memory_order_relaxed);

    // Extract sample position
    double pos = params->timeInfo.samplePosition.hi * 4294967296.0
               + params->timeInfo.samplePosition.lo;
    g_samplePos.store(pos, std::memory_order_relaxed);

    generateTone(doubleBufferIndex);

    (void)directProcess;
    return params;
}

// ============================================================================
// Helper: print ASIOError as string
// ============================================================================

static const char* asioErrorStr(ASIOError err) {
    switch (err) {
        case ASE_OK:               return "OK";
        case ASE_SUCCESS:          return "SUCCESS";
        case ASE_NotPresent:       return "NotPresent";
        case ASE_HWMalfunction:    return "HWMalfunction";
        case ASE_InvalidParameter: return "InvalidParameter";
        case ASE_InvalidMode:      return "InvalidMode";
        case ASE_SPNotAdvancing:    return "SPNotAdvancing";
        case ASE_NoClock:          return "NoClock";
        case ASE_NoMemory:         return "NoMemory";
        default:                   return "Unknown";
    }
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char* argv[]) {
    int durationSec = 5;
    bool diagDirect = false; // --direct: bypass ASIO, generate sine in USB transport

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--direct") == 0) {
            diagDirect = true;
        } else {
            durationSec = atoi(argv[i]);
            if (durationSec < 1) durationSec = 1;
            if (durationSec > 300) durationSec = 300;
        }
    }

    printf("=== DelugeASIO Test ===\n\n");

    // ---- Direct USB transport mode (bypass ASIO layer) ----
    if (diagDirect) {
        printf("MODE: --direct  (440 Hz sine generated directly in USB transport)\n");
        printf("This bypasses the ASIO ring buffer to isolate USB transport issues.\n\n");

        UsbTransport usb;
        if (!usb.open()) {
            printf("FAIL: Could not open Deluge USB device.\n");
            printf("      Ensure libusbK driver is installed via Zadig.\n");
            return 1;
        }
        printf("   Device opened.\n");

        if (!usb.startDiagTone(deluge::kAlt24Bit, 440.0, 0.25)) {
            printf("FAIL: Could not start USB streaming.\n");
            return 1;
        }
        printf("   Streaming 440 Hz sine for %d seconds...\n\n", durationSec);

        for (int sec = 0; sec < durationSec; sec++) {
            Sleep(1000);
            printf("   t=%2d  rate=%.1f Hz  streaming=%s\n",
                   sec + 1, usb.measuredSampleRate(),
                   usb.isStreaming() ? "yes" : "NO");
        }

        printf("\n   Stopping...\n");
        usb.stop();
        usb.close();
        printf("\n=== Direct Test Complete ===\n");
        return 0;
    }

    // ---- Create driver instance directly (no COM) ----
    HRESULT hr = S_OK;
    DelugeASIO driver(nullptr, &hr);
    if (FAILED(hr)) {
        printf("FAIL: Could not create driver instance (hr=0x%08lX)\n", hr);
        return 1;
    }

    // ---- init ----
    printf("1. Initializing driver...\n");
    ASIOBool ok = driver.init(nullptr);
    if (ok != ASIOTrue) {
        char errMsg[128];
        driver.getErrorMessage(errMsg);
        printf("FAIL: init() returned false: %s\n", errMsg);
        return 1;
    }

    // ---- Driver info ----
    char driverName[64];
    driver.getDriverName(driverName);
    printf("   Driver:  %s (v%ld)\n", driverName, driver.getDriverVersion());

    // ---- Channels ----
    long numIn = 0, numOut = 0;
    ASIOError err = driver.getChannels(&numIn, &numOut);
    printf("   Channels: %ld in, %ld out  [%s]\n", numIn, numOut, asioErrorStr(err));

    // ---- Buffer size ----
    long minBuf, maxBuf, prefBuf, gran;
    err = driver.getBufferSize(&minBuf, &maxBuf, &prefBuf, &gran);
    printf("   Buffer:  min=%ld  max=%ld  preferred=%ld  gran=%ld  [%s]\n",
           minBuf, maxBuf, prefBuf, gran, asioErrorStr(err));

    // ---- Sample rate ----
    ASIOSampleRate sr;
    driver.getSampleRate(&sr);
    printf("   Rate:    %.0f Hz\n", sr);

    err = driver.canSampleRate(44100.0);
    printf("   44100:   %s\n", err == ASE_OK ? "supported" : "NOT supported");
    err = driver.canSampleRate(48000.0);
    printf("   48000:   %s\n", err == ASE_OK ? "supported" : "NOT supported");

    // ---- Channel info ----
    for (long i = 0; i < numIn; i++) {
        ASIOChannelInfo info = {};
        info.channel = i;
        info.isInput = ASIOTrue;
        driver.getChannelInfo(&info);
        printf("   In[%ld]:   \"%s\"  type=%ld\n", i, info.name, info.type);
    }
    for (long i = 0; i < numOut; i++) {
        ASIOChannelInfo info = {};
        info.channel = i;
        info.isInput = ASIOFalse;
        driver.getChannelInfo(&info);
        printf("   Out[%ld]:  \"%s\"  type=%ld\n", i, info.name, info.type);
    }

    // ---- Clock sources ----
    ASIOClockSource clocks[4];
    long numClocks = 0;
    driver.getClockSources(clocks, &numClocks);
    for (long i = 0; i < numClocks; i++) {
        printf("   Clock[%ld]: \"%s\" %s\n", i, clocks[i].name,
               clocks[i].isCurrentSource ? "(current)" : "");
    }

    // ---- Create buffers ----
    printf("\n2. Creating buffers (size=%ld)...\n", prefBuf);

    const long totalChannels = numIn + numOut;
    ASIOBufferInfo* bufferInfos = new ASIOBufferInfo[totalChannels];
    long idx = 0;

    // Input channels first
    for (long i = 0; i < numIn; i++) {
        bufferInfos[idx].isInput = ASIOTrue;
        bufferInfos[idx].channelNum = i;
        bufferInfos[idx].buffers[0] = nullptr;
        bufferInfos[idx].buffers[1] = nullptr;
        idx++;
    }
    // Then output channels
    for (long i = 0; i < numOut; i++) {
        bufferInfos[idx].isInput = ASIOFalse;
        bufferInfos[idx].channelNum = i;
        bufferInfos[idx].buffers[0] = nullptr;
        bufferInfos[idx].buffers[1] = nullptr;
        idx++;
    }

    ASIOCallbacks callbacks = {};
    callbacks.bufferSwitch = bufferSwitch;
    callbacks.sampleRateDidChange = sampleRateDidChange;
    callbacks.asioMessage = asioMessage;
    callbacks.bufferSwitchTimeInfo = bufferSwitchTimeInfo;

    g_bufferSize = prefBuf;
    g_bufferInfos = bufferInfos;
    g_numIn = numIn;
    g_numOut = numOut;
    g_sampleRate = sr;
    err = driver.createBuffers(bufferInfos, totalChannels, prefBuf, &callbacks);
    if (err != ASE_OK) {
        printf("FAIL: createBuffers() => %s\n", asioErrorStr(err));
        delete[] bufferInfos;
        return 1;
    }

    printf("   Buffers created.\n");
    for (long i = 0; i < totalChannels; i++) {
        printf("   [%ld] %s ch%ld  buf[0]=%p  buf[1]=%p\n",
               i, bufferInfos[i].isInput ? " IN" : "OUT",
               bufferInfos[i].channelNum,
               bufferInfos[i].buffers[0], bufferInfos[i].buffers[1]);
    }

    // ---- Latencies ----
    long latIn, latOut;
    driver.getLatencies(&latIn, &latOut);
    printf("   Latency: in=%ld  out=%ld samples (%.1f / %.1f ms)\n",
           latIn, latOut, latIn * 1000.0 / sr, latOut * 1000.0 / sr);

    // ---- Start streaming ----
    printf("\n3. Starting audio stream for %d seconds...\n", durationSec);

    if (diagDirect) {
        printf("   MODE: --direct (sine generated in USB transport, ASIO layer bypassed)\n");
        // Access the USB transport directly via the driver's start path
        // but use the diag tone variant
        // We need to start the transport manually since start() uses the normal callback
        // For simplicity, just start normally then we'll compare with/without
    }

    err = driver.start();
    if (err != ASE_OK) {
        char errMsg[128];
        driver.getErrorMessage(errMsg);
        printf("FAIL: start() => %s: %s\n", asioErrorStr(err), errMsg);
        driver.disposeBuffers();
        delete[] bufferInfos;
        return 1;
    }
    printf("   Streaming!\n\n");

    // ---- Monitor for N seconds ----
    long prevCount = 0;
    uint32_t prevCb = 0, prevSw = 0, prevUr = 0;
    for (int sec = 0; sec < durationSec; sec++) {
        Sleep(1000);

        long count = g_bufferSwitchCount.load(std::memory_order_relaxed);
        double pos = g_samplePos.load(std::memory_order_relaxed);
        long switchesThisSec = count - prevCount;
        prevCount = count;

        // Diagnostic counters
        uint32_t cb = driver.diagCallbackCount_.load(std::memory_order_relaxed);
        uint32_t sw = driver.diagSwitchCount_.load(std::memory_order_relaxed);
        uint32_t ur = driver.diagOutUnderruns_.load(std::memory_order_relaxed);

        // Expected: sampleRate / bufferSize switches per second
        double expectedHz = sr / prefBuf;

        printf("   t=%2d  bufSw: %4ld (%3ld/s, exp~%.0f)  pos: %.0f (%.1fs)"
               "  usbCb: %u (+%u)  drvrSw: %u (+%u)  underruns: %u (+%u)\n",
               sec + 1, count, switchesThisSec, expectedHz, pos, pos / sr,
               cb, cb - prevCb, sw, sw - prevSw, ur, ur - prevUr);

        prevCb = cb;
        prevSw = sw;
        prevUr = ur;
    }

    // ---- Stop ----
    printf("\n4. Stopping...\n");
    driver.stop();

    long finalCount = g_bufferSwitchCount.load();
    double finalPos = g_samplePos.load();
    printf("   Total buffer switches: %ld\n", finalCount);
    printf("   Final sample position: %.0f (%.2f seconds)\n",
           finalPos, finalPos / sr);

    double expectedSwitches = sr * durationSec / prefBuf;
    double pct = (finalCount / expectedSwitches) * 100.0;
    printf("   Efficiency: %.1f%% of expected (%.0f)\n", pct, expectedSwitches);

    // Diagnostics summary
    printf("\n   --- Diagnostics ---\n");
    printf("   USB callbacks:     %u\n", driver.diagCallbackCount_.load());
    printf("   Driver buf switch: %u\n", driver.diagSwitchCount_.load());
    printf("   Output underruns:  %u\n", driver.diagOutUnderruns_.load());
    printf("   Output overflows:  %u\n", driver.diagOutOverflows_.load());

    // ---- Cleanup ----
    printf("\n5. Disposing buffers...\n");
    driver.disposeBuffers();
    delete[] bufferInfos;

    printf("\n=== Test Complete ===\n");
    return 0;
}
