// DelugeASIO — USB Transport Layer (header)
// Handles libusbK isochronous I/O for speaker OUT and mic IN endpoints.

#pragma once

#include "deluge_usb.h"

#include <Windows.h>
#include <libusbk.h>

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <vector>

// ============================================================================
// UsbTransport — manages isochronous USB audio streaming
// ============================================================================

class UsbTransport {
public:
    // Callback: delivers interleaved audio from mic IN and accepts speaker OUT data.
    // Called from the ISO processing thread.
    //   inSamples:  interleaved int32_t[numSamples * 2] from mic (left-justified 24-bit)
    //   outSamples: interleaved int32_t[numSamples * 2] to write for speaker
    //   numSamples: number of sample frames this callback
    using AudioCallback = std::function<void(const int32_t* inSamples, int32_t* outSamples, uint32_t numSamples)>;

    UsbTransport();
    ~UsbTransport();

    // Open the Deluge USB device. Returns false if not found.
    bool open();

    // Close the device and release all resources.
    void close();

    // Is the device open?
    bool isOpen() const { return device_ != nullptr; }

    // Start streaming. The callback will be invoked from the ISO thread.
    bool start(uint8_t altSetting, AudioCallback callback);

    // Start streaming with a built-in sine tone (bypasses ASIO layer for diagnostics).
    bool startDiagTone(uint8_t altSetting, double freq = 440.0, double amplitude = 0.25);

    // Stop streaming.
    void stop();

    // Is streaming active?
    bool isStreaming() const { return streaming_.load(std::memory_order_acquire); }

    // Current measured sample rate (from implicit feedback).
    double measuredSampleRate() const { return measuredRate_.load(std::memory_order_relaxed); }

private:
    // ISO transfer context
    struct IsoContext {
        KOVL_HANDLE          overlapped = nullptr;
        KISOCH_HANDLE        isochHandle = nullptr;
        std::vector<uint8_t> buffer;
        bool                 submitted  = false;
        uint32_t             packedBytes = 0;  // actual bytes packed by processOutComplete
    };

    // The ISO processing thread
    static DWORD WINAPI isoThreadProc(LPVOID param);
    void isoThread();

    // Submit an ISO OUT transfer
    bool submitOut(IsoContext& ctx);
    // Submit an ISO IN transfer
    bool submitIn(IsoContext& ctx);

    // Process completed IN transfer — extract samples and compute feedback
    void processInComplete(IsoContext& ctx);
    // Process completed OUT transfer — fill next buffer from callback
    void processOutComplete(IsoContext& ctx);

    // Convert 24-bit LE packed bytes → int32 left-justified
    static void unpackIn24(const uint8_t* src, int32_t* dst, uint32_t numSamples);
    // Convert int32 left-justified → 24-bit LE packed bytes
    static void packOut24(const int32_t* src, uint8_t* dst, uint32_t numSamples);

    // Device
    KUSB_HANDLE             device_        = nullptr;
    KOVL_POOL_HANDLE        ovlPool_       = nullptr;

    // Streaming state
    std::atomic<bool>       streaming_{false};
    HANDLE                  thread_        = nullptr;
    HANDLE                  stopEvent_     = nullptr;

    // ISO contexts (double-buffered for each direction)
    IsoContext              outCtx_[deluge::kPendingTransfers];
    IsoContext              inCtx_[deluge::kPendingTransfers];

    // Audio callback
    AudioCallback           callback_;

    // Implicit feedback: measured sample rate from mic IN timing
    std::atomic<double>     measuredRate_{deluge::kSampleRate};
    double                  rateAccum_     = 0.0;
    uint64_t                rateCount_     = 0;

    // Scratch buffers for sample conversion
    std::vector<int32_t>    inScratch_;
    std::vector<int32_t>    outScratch_;

    // Current alt setting (1=16-bit, 2=24-bit)
    uint8_t                 altSetting_    = deluge::kAlt24Bit;
    uint32_t                bytesPerSample_ = 3; // 2 or 3
    uint32_t                maxPacketSize_ = deluge::kMaxPacketSize24;

    // Adaptive rate tracking: how many samples to put in next OUT packet
    double                  outFractionalSamples_ = 0.0;

    // Diagnostic tone state (used by startDiagTone)
    double                  diagTonePhase_ = 0.0;
    double                  diagToneFreq_  = 440.0;
    double                  diagToneAmp_   = 0.25;
    uint32_t                diagDumpCount_ = 0; // transfer counter for debug dumps
};
