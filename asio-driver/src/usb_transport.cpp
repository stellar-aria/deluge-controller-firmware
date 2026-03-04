// DelugeASIO — USB Transport Layer
// Handles libusbK isochronous I/O for speaker OUT and mic IN endpoints.
// The mic IN endpoint provides implicit feedback for rate adaptation.

#include "usb_transport.h"

#include <cstring>
#include <cmath>
#include <algorithm>
#include <cstdio>

// ============================================================================
// Construction / Destruction
// ============================================================================

UsbTransport::UsbTransport() {
    stopEvent_ = CreateEventW(nullptr, TRUE, FALSE, nullptr);
}

UsbTransport::~UsbTransport() {
    close();
    if (stopEvent_) {
        CloseHandle(stopEvent_);
        stopEvent_ = nullptr;
    }
}

// ============================================================================
// Device open/close
// ============================================================================

bool UsbTransport::open() {
    if (device_) return true;

    KLST_HANDLE deviceList = nullptr;
    if (!LstK_Init(&deviceList, KLST_FLAG_NONE)) {
        return false;
    }

    KLST_DEVINFO_HANDLE devInfo = nullptr;
    bool found = false;

    while (LstK_MoveNext(deviceList, &devInfo)) {
        if (devInfo->Common.Vid == deluge::kVendorId &&
            devInfo->Common.Pid == deluge::kProductId) {
            found = true;
            break;
        }
    }

    if (!found) {
        LstK_Free(deviceList);
        return false;
    }

    KUSB_HANDLE handle = nullptr;
    if (!UsbK_Init(&handle, devInfo)) {
        LstK_Free(deviceList);
        return false;
    }

    LstK_Free(deviceList);
    device_ = handle;

    // Claim the audio streaming interfaces.
    // UsbK_Init only claims the first interface (audio control, itf 0).
    // We must explicitly claim itf 1 (speaker) and itf 2 (mic) so that
    // SetAltInterface / ISO transfers work on their endpoints.
    if (!UsbK_ClaimInterface(device_, deluge::kItfAudioStreamSpk, FALSE)) {
        OutputDebugStringA("DelugeASIO: failed to claim speaker interface\n");
        UsbK_Free(device_);
        device_ = nullptr;
        return false;
    }
    if (!UsbK_ClaimInterface(device_, deluge::kItfAudioStreamMic, FALSE)) {
        // Mic interface may not exist in speaker-only firmware mode — non-fatal.
        OutputDebugStringA("DelugeASIO: could not claim mic interface (speaker-only mode?)\n");
    }

    return true;
}

void UsbTransport::close() {
    stop();

    if (device_) {
        UsbK_Free(device_);
        device_ = nullptr;
    }
}

// ============================================================================
// Streaming start/stop
// ============================================================================

bool UsbTransport::startDiagTone(uint8_t altSetting, double freq, double amplitude) {
    diagTonePhase_ = 0.0;
    diagToneFreq_  = freq;
    diagToneAmp_   = amplitude;

    // Use an internal callback that generates a sine directly into outSamples
    auto cb = [this](const int32_t* /*in*/, int32_t* out, uint32_t n) {
        constexpr double kTwoPi = 2.0 * 3.14159265358979323846;
        const double phaseInc = kTwoPi * diagToneFreq_ / deluge::kSampleRate;
        for (uint32_t i = 0; i < n; i++) {
            double sample = diagToneAmp_ * sin(diagTonePhase_);
            int32_t val = static_cast<int32_t>(sample * 2147483392.0); // 0x7FFFFF00
            out[i * 2]     = val; // Left
            out[i * 2 + 1] = val; // Right
            diagTonePhase_ += phaseInc;
        }
        // Wrap phase to avoid precision loss
        while (diagTonePhase_ >= kTwoPi) diagTonePhase_ -= kTwoPi;
    };

    return start(altSetting, cb);
}

bool UsbTransport::start(uint8_t altSetting, AudioCallback callback) {
    if (!device_ || streaming_.load()) return false;

    altSetting_ = altSetting;
    bytesPerSample_ = (altSetting == deluge::kAlt16Bit) ? 2 : 3;
    maxPacketSize_ = (altSetting == deluge::kAlt16Bit)
                         ? deluge::kMaxPacketSize16
                         : deluge::kMaxPacketSize24;
    callback_ = std::move(callback);

    // Set alt settings on both interfaces
    // UsbK_SetAltInterface(handle, NumberOrIndex, IsIndex, AltSettingNumber)
    if (!UsbK_SetAltInterface(device_, deluge::kItfAudioStreamSpk, FALSE, altSetting_)) {
        OutputDebugStringA("DelugeASIO: SetAltInterface failed on speaker\n");
        return false;
    }
    if (!UsbK_SetAltInterface(device_, deluge::kItfAudioStreamMic, FALSE, altSetting_)) {
        // Mic interface may not be present in speaker-only mode — continue without it
        OutputDebugStringA("DelugeASIO: SetAltInterface failed on mic (speaker-only mode?)\n");
    }

    // Set clock source sample rate via UAC2 SET_CUR control request.
    // TinyUSB's EP_IN flow control needs sample_rate_tx to be set; for UAC2
    // this only happens when the host sends a SET_CUR on the clock entity.
    // libusbK doesn't do this automatically (unlike usbaudio2.sys), so we
    // must send it explicitly.
    {
        WINUSB_SETUP_PACKET setup = {};
        setup.RequestType = 0x21; // Host-to-device, Class, Interface
        setup.Request     = 0x01; // AUDIO20_CS_REQ_CUR
        setup.Value       = 0x0100; // CS_SAM_FREQ_CONTROL << 8
        setup.Index       = (deluge::kClockSourceEntity << 8) | deluge::kItfAudioControl;
        setup.Length       = 4;
        uint32_t sampleRate = deluge::kSampleRate;
        UINT transferred = 0;
        if (!UsbK_ControlTransfer(device_, setup, reinterpret_cast<PUCHAR>(&sampleRate), 4, &transferred, nullptr)) {
            OutputDebugStringA("DelugeASIO: SET_CUR clock sample rate failed\n");
        }
    }

    // Create overlapped pool
    // We need enough overlapped handles for all pending transfers (IN + OUT)
    const UINT poolSize = deluge::kPendingTransfers * 2 + 4;
    if (!OvlK_Init(&ovlPool_, device_, poolSize, KOVL_POOL_FLAG_NONE)) {
        OutputDebugStringA("DelugeASIO: OvlK_Init failed\n");
        return false;
    }

    // Allocate scratch buffers (worst case: 8 microframes * 6 samples * 2 channels)
    const uint32_t maxSamplesPerTransfer = deluge::kMicroframesPerTransfer * 7;
    inScratch_.resize(maxSamplesPerTransfer * deluge::kNumChannels);
    outScratch_.resize(maxSamplesPerTransfer * deluge::kNumChannels);

    // Initialize ISO contexts
    const uint32_t transferSize = deluge::kMicroframesPerTransfer * maxPacketSize_;

    // Select speaker interface before initializing OUT ISO handles.
    // IsochK_Init uses UsbK_QueryPipe internally, which searches
    // the *currently selected* interface for the requested pipe ID.
    UsbK_SelectInterface(device_, deluge::kItfAudioStreamSpk, FALSE);

    for (uint32_t i = 0; i < deluge::kPendingTransfers; i++) {
        // OUT contexts
        outCtx_[i].buffer.resize(transferSize);
        outCtx_[i].submitted = false;
        if (!IsochK_Init(&outCtx_[i].isochHandle, device_, deluge::kEpAudioOut,
                         deluge::kMicroframesPerTransfer,
                         outCtx_[i].buffer.data(),
                         static_cast<UINT>(outCtx_[i].buffer.size()))) {
            OutputDebugStringA("DelugeASIO: IsochK_Init failed for OUT\n");
            return false;
        }
        IsochK_SetPacketOffsets(outCtx_[i].isochHandle, maxPacketSize_);
    }

    // Select mic interface before initializing IN ISO handles.
    UsbK_SelectInterface(device_, deluge::kItfAudioStreamMic, FALSE);

    for (uint32_t i = 0; i < deluge::kPendingTransfers; i++) {
        // IN contexts
        inCtx_[i].buffer.resize(transferSize);
        inCtx_[i].submitted = false;
        if (!IsochK_Init(&inCtx_[i].isochHandle, device_, deluge::kEpAudioIn,
                         deluge::kMicroframesPerTransfer,
                         inCtx_[i].buffer.data(),
                         static_cast<UINT>(inCtx_[i].buffer.size()))) {
            OutputDebugStringA("DelugeASIO: IsochK_Init failed for IN\n");
            return false;
        }
        IsochK_SetPacketOffsets(inCtx_[i].isochHandle, maxPacketSize_);
    }

    // Reset rate tracking
    outFractionalSamples_ = 0.0;
    measuredRate_.store(deluge::kSampleRate);
    rateAccum_ = 0.0;
    rateCount_ = 0;
    diagDumpCount_ = 0;

    // Start the ISO thread
    ResetEvent(stopEvent_);
    streaming_.store(true, std::memory_order_release);
    thread_ = CreateThread(nullptr, 0, isoThreadProc, this, 0, nullptr);
    if (!thread_) {
        streaming_.store(false);
        return false;
    }

    // Raise thread priority for low-latency audio
    SetThreadPriority(thread_, THREAD_PRIORITY_TIME_CRITICAL);
    return true;
}

void UsbTransport::stop() {
    if (!streaming_.load()) return;

    streaming_.store(false, std::memory_order_release);
    SetEvent(stopEvent_);

    if (thread_) {
        WaitForSingleObject(thread_, 3000);
        CloseHandle(thread_);
        thread_ = nullptr;
    }

    // Clean up ISO contexts
    for (uint32_t i = 0; i < deluge::kPendingTransfers; i++) {
        if (outCtx_[i].isochHandle) {
            IsochK_Free(outCtx_[i].isochHandle);
            outCtx_[i].isochHandle = nullptr;
        }
        if (inCtx_[i].isochHandle) {
            IsochK_Free(inCtx_[i].isochHandle);
            inCtx_[i].isochHandle = nullptr;
        }
        outCtx_[i].submitted = false;
        inCtx_[i].submitted = false;
    }

    // Clean up overlapped pool
    if (ovlPool_) {
        OvlK_Free(ovlPool_);
        ovlPool_ = nullptr;
    }

    // Reset alt settings to idle
    if (device_) {
        UsbK_SetAltInterface(device_, deluge::kItfAudioStreamSpk, FALSE, deluge::kAltIdle);
        UsbK_SetAltInterface(device_, deluge::kItfAudioStreamMic, FALSE, deluge::kAltIdle);
    }

    callback_ = nullptr;
}

// ============================================================================
// ISO thread
// ============================================================================

DWORD WINAPI UsbTransport::isoThreadProc(LPVOID param) {
    auto* self = static_cast<UsbTransport*>(param);
    self->isoThread();
    return 0;
}

void UsbTransport::isoThread() {
    // Submit initial IN and OUT transfers
    for (uint32_t i = 0; i < deluge::kPendingTransfers; i++) {
        // Fill initial OUT buffers with silence
        std::memset(outCtx_[i].buffer.data(), 0, outCtx_[i].buffer.size());
        submitOut(outCtx_[i]);
        submitIn(inCtx_[i]);
    }

    // Main ISO processing loop
    while (streaming_.load(std::memory_order_acquire)) {
        // Process IN completions
        for (uint32_t i = 0; i < deluge::kPendingTransfers; i++) {
            if (!inCtx_[i].submitted) continue;

            UINT transferred = 0;
            // OvlK_Wait with timeout=0 is a non-blocking check.
            // KOVL_WAIT_FLAG_RELEASE_ON_SUCCESS returns the handle to the pool on completion.
            // Do NOT use OvlK_WaitOrCancel here — it cancels incomplete transfers!
            if (OvlK_Wait(inCtx_[i].overlapped, 0, KOVL_WAIT_FLAG_RELEASE_ON_SUCCESS, &transferred)) {
                inCtx_[i].overlapped = nullptr;
                inCtx_[i].submitted = false;
                processInComplete(inCtx_[i]);
                submitIn(inCtx_[i]); // Resubmit (acquires new overlapped)
            }
        }

        // Process OUT completions
        for (uint32_t i = 0; i < deluge::kPendingTransfers; i++) {
            if (!outCtx_[i].submitted) continue;

            UINT transferred = 0;
            if (OvlK_Wait(outCtx_[i].overlapped, 0, KOVL_WAIT_FLAG_RELEASE_ON_SUCCESS, &transferred)) {
                outCtx_[i].overlapped = nullptr;
                outCtx_[i].submitted = false;
                processOutComplete(outCtx_[i]);
                submitOut(outCtx_[i]); // Resubmit (acquires new overlapped)
            }
        }

        // Brief sleep to avoid tight-spinning — the ISO transfers are 1ms each
        // so checking every 100µs is adequate
        if (WaitForSingleObject(stopEvent_, 0) == WAIT_OBJECT_0) break;
        Sleep(0); // Yield but don't burn CPU
    }

    // Cancel any pending transfers and release their overlapped handles
    for (uint32_t i = 0; i < deluge::kPendingTransfers; i++) {
        if (outCtx_[i].submitted && outCtx_[i].overlapped) {
            OvlK_WaitOrCancel(outCtx_[i].overlapped, 100, nullptr);
            OvlK_Release(outCtx_[i].overlapped);
            outCtx_[i].overlapped = nullptr;
            outCtx_[i].submitted = false;
        }
        if (inCtx_[i].submitted && inCtx_[i].overlapped) {
            OvlK_WaitOrCancel(inCtx_[i].overlapped, 100, nullptr);
            OvlK_Release(inCtx_[i].overlapped);
            inCtx_[i].overlapped = nullptr;
            inCtx_[i].submitted = false;
        }
    }
}

// ============================================================================
// ISO submission
// ============================================================================

bool UsbTransport::submitOut(IsoContext& ctx) {
    if (!OvlK_Acquire(&ctx.overlapped, ovlPool_)) return false;

    // Do NOT call IsochK_SetPacketOffsets here — processOutComplete already
    // set per-packet sizes via IsochK_SetPacket with contiguous offsets.

    // Use the actual packed byte count so the host sends exactly the right
    // number of bytes per microframe.  For the initial silence submission
    // (before processOutComplete has run), fall back to buffer.size().
    UINT transferLen = ctx.packedBytes > 0
        ? static_cast<UINT>(ctx.packedBytes)
        : static_cast<UINT>(ctx.buffer.size());
    BOOL ok = UsbK_IsochWritePipe(
        ctx.isochHandle,
        transferLen,
        nullptr,  // FrameNumber — use ASAP
        0,        // NumberOfPackets — use all
        reinterpret_cast<LPOVERLAPPED>(ctx.overlapped)
    );

    if (!ok && GetLastError() != ERROR_IO_PENDING) {
        OvlK_Release(ctx.overlapped);
        ctx.overlapped = nullptr;
        return false;
    }

    ctx.submitted = true;
    return true;
}

bool UsbTransport::submitIn(IsoContext& ctx) {
    if (!OvlK_Acquire(&ctx.overlapped, ovlPool_)) return false;

    IsochK_SetPacketOffsets(ctx.isochHandle, maxPacketSize_);

    UINT transferLen = static_cast<UINT>(ctx.buffer.size());
    BOOL ok = UsbK_IsochReadPipe(
        ctx.isochHandle,
        transferLen,
        nullptr,  // FrameNumber — use ASAP
        0,        // NumberOfPackets — use all
        reinterpret_cast<LPOVERLAPPED>(ctx.overlapped)
    );

    if (!ok && GetLastError() != ERROR_IO_PENDING) {
        OvlK_Release(ctx.overlapped);
        ctx.overlapped = nullptr;
        return false;
    }

    ctx.submitted = true;
    return true;
}

// ============================================================================
// Completion processing
// ============================================================================

void UsbTransport::processInComplete(IsoContext& ctx) {
    // Process each microframe's packet from the IN (mic) endpoint
    uint32_t totalSamples = 0;

    for (UINT pktIdx = 0; pktIdx < deluge::kMicroframesPerTransfer; pktIdx++) {
        UINT offset = 0;
        UINT length = 0;
        UINT status = 0;

        if (!IsochK_GetPacket(ctx.isochHandle, pktIdx, &offset, &length, &status)) continue;
        if (status != 0 || length == 0) continue;

        const uint32_t bytesPerFrame = bytesPerSample_ * deluge::kNumChannels;
        const uint32_t framesToRead = length / bytesPerFrame;

        if (bytesPerSample_ == 3) {
            unpackIn24(ctx.buffer.data() + offset,
                       inScratch_.data() + totalSamples * deluge::kNumChannels,
                       framesToRead * deluge::kNumChannels);
        } else {
            // 16-bit: sign-extend to int32 left-justified
            const int16_t* src = reinterpret_cast<const int16_t*>(ctx.buffer.data() + offset);
            int32_t* dst = inScratch_.data() + totalSamples * deluge::kNumChannels;
            for (uint32_t s = 0; s < framesToRead * deluge::kNumChannels; s++) {
                dst[s] = static_cast<int32_t>(src[s]) << 16;
            }
        }

        totalSamples += framesToRead;

        // Implicit feedback: track how many samples the device is actually producing
        // per microframe to derive its true sample rate
        rateAccum_ += static_cast<double>(framesToRead);
        rateCount_++;

        if (rateCount_ >= 8000) { // Update every ~1 second for HS
            double measured = (rateAccum_ / static_cast<double>(rateCount_)) * 8000.0;
            measuredRate_.store(measured, std::memory_order_relaxed);
            rateAccum_ = 0.0;
            rateCount_ = 0;
        }
    }

    // The callback will be invoked from processOutComplete which pairs IN/OUT
    // For now, just store the IN data for the next callback invocation
    (void)totalSamples;
}

void UsbTransport::processOutComplete(IsoContext& ctx) {
    if (!callback_) return;

    // Determine how many samples to write for each microframe based on
    // the measured device rate (implicit feedback from mic IN).
    // At 44100 Hz HS: 5.5125 samples/µframe, alternating 5 and 6.
    const double samplesPerUf = measuredRate_.load(std::memory_order_relaxed) / 8000.0;

    // Compute per-packet sample counts in a single pass to avoid divergence
    uint32_t samplesPerPacket[deluge::kMicroframesPerTransfer];
    uint32_t totalSamples = 0;

    for (UINT pktIdx = 0; pktIdx < deluge::kMicroframesPerTransfer; pktIdx++) {
        outFractionalSamples_ += samplesPerUf;
        samplesPerPacket[pktIdx] = static_cast<uint32_t>(outFractionalSamples_);
        outFractionalSamples_ -= samplesPerPacket[pktIdx];
        totalSamples += samplesPerPacket[pktIdx];
    }

    // Invoke the audio callback to fill the output buffer
    std::memset(outScratch_.data(), 0, totalSamples * deluge::kNumChannels * sizeof(int32_t));
    callback_(inScratch_.data(), outScratch_.data(), totalSamples);

    // Pack the output samples CONTIGUOUSLY into the ISO buffer.
    // Using contiguous offsets (not maxPacketSize-aligned) ensures the host
    // controller sends exactly packetBytes per microframe, not maxPacketSize.
    uint32_t sampleOffset = 0;
    uint32_t runningOffset = 0;

    for (UINT pktIdx = 0; pktIdx < deluge::kMicroframesPerTransfer; pktIdx++) {
        const uint32_t samplesThisPacket = samplesPerPacket[pktIdx];
        const uint32_t bytesPerFrame = bytesPerSample_ * deluge::kNumChannels;
        const uint32_t packetBytes = samplesThisPacket * bytesPerFrame;

        if (bytesPerSample_ == 3) {
            packOut24(outScratch_.data() + sampleOffset * deluge::kNumChannels,
                      ctx.buffer.data() + runningOffset,
                      samplesThisPacket * deluge::kNumChannels);
        } else {
            // 16-bit: extract from int32 left-justified
            const int32_t* src = outScratch_.data() + sampleOffset * deluge::kNumChannels;
            int16_t* dst = reinterpret_cast<int16_t*>(ctx.buffer.data() + runningOffset);
            for (uint32_t s = 0; s < samplesThisPacket * deluge::kNumChannels; s++) {
                dst[s] = static_cast<int16_t>(src[s] >> 16);
            }
        }

        // Update the ISO packet offset and length — contiguous, no gaps
        IsochK_SetPacket(ctx.isochHandle, pktIdx, runningOffset, packetBytes, 0);

        runningOffset += packetBytes;
        sampleOffset += samplesThisPacket;
    }
    ctx.packedBytes = runningOffset;

    diagDumpCount_++;
}

// ============================================================================
// Sample conversion
// ============================================================================

void UsbTransport::unpackIn24(const uint8_t* src, int32_t* dst, uint32_t count) {
    // 24-bit LE packed → int32 left-justified (top 24 bits)
    for (uint32_t i = 0; i < count; i++) {
        dst[i] = (static_cast<int32_t>(src[0])       )
               | (static_cast<int32_t>(src[1]) << 8   )
               | (static_cast<int32_t>(src[2]) << 16  );
        // Sign-extend and left-justify
        dst[i] <<= 8;
        src += 3;
    }
}

void UsbTransport::packOut24(const int32_t* src, uint8_t* dst, uint32_t count) {
    // int32 left-justified (top 24 bits) → 24-bit LE packed
    for (uint32_t i = 0; i < count; i++) {
        int32_t val = src[i] >> 8; // Right-shift to get 24-bit value
        dst[0] = static_cast<uint8_t>(val);
        dst[1] = static_cast<uint8_t>(val >> 8);
        dst[2] = static_cast<uint8_t>(val >> 16);
        dst += 3;
    }
}
