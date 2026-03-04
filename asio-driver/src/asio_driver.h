// DelugeASIO — ASIO Driver Implementation (header)
// Extends the Steinberg ASIO SDK AsioDriver pattern for USB audio via libusbK.

#pragma once

// Windows headers MUST come before the ASIO SDK headers.
// The SDK's combase.h defines DWORD/LONG/etc. as externs and expects the
// Windows SDK typedefs to already be established.
#include <windows.h>
#include <objbase.h>

#include "asiosys.h"
#include "asio.h"
#include "iasiodrv.h"
#include "combase.h"

#include "usb_transport.h"

#include <mutex>
#include <vector>
#include <atomic>

// {7D2E8A30-4F1C-4B5E-9A3D-1C6F8E2B0D4A}
// CLSID for the Deluge ASIO driver — unique identifier for COM registration
static const CLSID CLSID_DelugeASIO = {
    0x7D2E8A30, 0x4F1C, 0x4B5E,
    {0x9A, 0x3D, 0x1C, 0x6F, 0x8E, 0x2B, 0x0D, 0x4A}
};

// ============================================================================
// DelugeASIO — COM ASIO driver
// ============================================================================

class DelugeASIO : public IASIO, public CUnknown {
public:
    DelugeASIO(LPUNKNOWN pUnk, HRESULT* phr);
    ~DelugeASIO() override;

    DECLARE_IUNKNOWN

    // Factory method — called by SDK class factory
    static CUnknown* CreateInstance(LPUNKNOWN pUnk, HRESULT* phr);

    // IUnknown
    HRESULT STDMETHODCALLTYPE NonDelegatingQueryInterface(REFIID riid, void** ppvObject) override;

    // IASIO
    ASIOBool init(void* sysHandle) override;
    void     getDriverName(char* name) override;
    long     getDriverVersion() override;
    void     getErrorMessage(char* string) override;

    ASIOError start() override;
    ASIOError stop() override;

    ASIOError getChannels(long* numInputChannels, long* numOutputChannels) override;
    ASIOError getLatencies(long* inputLatency, long* outputLatency) override;
    ASIOError getBufferSize(long* minSize, long* maxSize,
                            long* preferredSize, long* granularity) override;

    ASIOError canSampleRate(ASIOSampleRate sampleRate) override;
    ASIOError getSampleRate(ASIOSampleRate* sampleRate) override;
    ASIOError setSampleRate(ASIOSampleRate sampleRate) override;
    ASIOError getClockSources(ASIOClockSource* clocks, long* numSources) override;
    ASIOError setClockSource(long reference) override;

    ASIOError getSamplePosition(ASIOSamples* sPos, ASIOTimeStamp* tStamp) override;
    ASIOError getChannelInfo(ASIOChannelInfo* info) override;

    ASIOError createBuffers(ASIOBufferInfo* bufferInfos, long numChannels,
                            long bufferSize, ASIOCallbacks* callbacks) override;
    ASIOError disposeBuffers() override;

    ASIOError controlPanel() override;
    ASIOError future(long selector, void* opt) override;
    ASIOError outputReady() override;

private:
    // Audio callback from USB transport thread
    void audioCallback(const int32_t* inSamples, int32_t* outSamples, uint32_t numSamples);

    // Notify host of buffer switch
    void performBufferSwitch();

    static constexpr long kNumInputChannels  = 2; // Stereo mic
    static constexpr long kNumOutputChannels = 2; // Stereo speaker

    // State
    bool                initialized_ = false;
    bool                started_     = false;
    bool                buffersCreated_ = false;

    // USB transport
    UsbTransport        usb_;

    // Host callbacks
    ASIOCallbacks*      callbacks_   = nullptr;

    // Buffer management — double-buffered
    long                bufferSize_  = 0;
    long                toggle_      = 0;  // 0 or 1 — current buffer half

    // Per-channel buffer storage [channel][doubleBufferIndex]
    // Input channels: interleaved int32 from mic
    // Output channels: interleaved int32 to speaker
    struct ChannelBuffer {
        bool    isInput;
        long    channel;
        int32_t* buffers[2]; // double-buffer pair
    };
    std::vector<ChannelBuffer> channelBuffers_;

    // Ring buffer for bridging USB ISO timing to ASIO buffer timing
    // USB delivers ~44 samples per ms; ASIO expects fixed bufferSize blocks
    std::vector<int32_t> inRing_;     // interleaved stereo
    std::vector<int32_t> outRing_;    // interleaved stereo
    uint32_t             inRingWr_   = 0;
    uint32_t             inRingRd_   = 0;
    uint32_t             outRingWr_  = 0;
    uint32_t             outRingRd_  = 0;
    uint32_t             ringSize_   = 0;
    std::mutex           ringMutex_;

    // Sample position tracking
    double               samplePosition_ = 0.0;
    ASIOSampleRate       sampleRate_      = 44100.0;

    // Diagnostic counters (public for test inspection)
public:
    std::atomic<uint32_t> diagCallbackCount_{0};  // audioCallback invocations
    std::atomic<uint32_t> diagSwitchCount_{0};     // buffer switches fired
    std::atomic<uint32_t> diagOutUnderruns_{0};    // output ring underruns
    std::atomic<uint32_t> diagOutOverflows_{0};    // output ring near-overflow
private:

    // Error message
    char                 errorMessage_[128] = {};
};
