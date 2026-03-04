// DelugeASIO — ASIO Driver Implementation
// Bridges USB audio transport (libusbK ISO) to ASIO buffer switch callbacks.
//
// Licensed under GPL-3.0 (same as Deluge firmware and ASIO SDK GPL option).

#include "asio_driver.h"
#include "deluge_usb.h"

#include <mmsystem.h>   // timeGetTime
#include <cstring>
#include <cmath>
#include <cstdio>
#include <algorithm>

// ============================================================================
// Timestamp helper
// ============================================================================

static const double kTwoRaisedTo32 = 4294967296.0;

static void getNanoSeconds(ASIOTimeStamp* ts) {
    double nanoSeconds = static_cast<double>(timeGetTime()) * 1000000.0;
    ts->hi = static_cast<unsigned long>(nanoSeconds / kTwoRaisedTo32);
    ts->lo = static_cast<unsigned long>(nanoSeconds - (ts->hi * kTwoRaisedTo32));
}

// ============================================================================
// COM Factory
// ============================================================================

CUnknown* DelugeASIO::CreateInstance(LPUNKNOWN pUnk, HRESULT* phr) {
    return new DelugeASIO(pUnk, phr);
}

// ============================================================================
// Construction / Destruction
// ============================================================================

DelugeASIO::DelugeASIO(LPUNKNOWN pUnk, HRESULT* phr)
    : CUnknown(TEXT("DelugeASIO"), pUnk, phr) {
}

DelugeASIO::~DelugeASIO() {
    stop();
    disposeBuffers();
    usb_.close();
}

// ============================================================================
// IUnknown
// ============================================================================

HRESULT STDMETHODCALLTYPE DelugeASIO::NonDelegatingQueryInterface(REFIID riid, void** ppvObject) {
    if (riid == CLSID_DelugeASIO || riid == IID_IUnknown) {
        return GetInterface(static_cast<IASIO*>(this), ppvObject);
    }
    return CUnknown::NonDelegatingQueryInterface(riid, ppvObject);
}

// ============================================================================
// ASIO: init / info
// ============================================================================

ASIOBool DelugeASIO::init(void* sysHandle) {
    if (initialized_) return ASIOTrue;

    if (!usb_.open()) {
        strcpy_s(errorMessage_, "Could not open Deluge USB device. "
                                "Ensure libusbK driver is installed via Zadig.");
        return ASIOFalse;
    }

    sampleRate_ = deluge::kSampleRate;
    initialized_ = true;
    strcpy_s(errorMessage_, "");
    return ASIOTrue;
}

void DelugeASIO::getDriverName(char* name) {
    strcpy_s(name, 32, "Deluge ASIO");
}

long DelugeASIO::getDriverVersion() {
    return 1;
}

void DelugeASIO::getErrorMessage(char* string) {
    strcpy_s(string, 128, errorMessage_);
}

// ============================================================================
// ASIO: channels / buffers / sample rate
// ============================================================================

ASIOError DelugeASIO::getChannels(long* numInputChannels, long* numOutputChannels) {
    if (!initialized_) return ASE_NotPresent;
    *numInputChannels = kNumInputChannels;
    *numOutputChannels = kNumOutputChannels;
    return ASE_OK;
}

ASIOError DelugeASIO::getBufferSize(long* minSize, long* maxSize,
                                     long* preferredSize, long* granularity) {
    // Offer power-of-2 buffer sizes from 64 to 2048
    *minSize       = 64;
    *maxSize       = 2048;
    *preferredSize = 256;
    *granularity   = -1; // Power of 2
    return ASE_OK;
}

ASIOError DelugeASIO::getLatencies(long* inputLatency, long* outputLatency) {
    if (!initialized_) return ASE_NotPresent;

    // Latency = buffer size + USB transport latency (~2ms for double-buffered ISO)
    long usbLatencySamples = static_cast<long>(sampleRate_ * 0.002); // ~2ms
    long bs = bufferSize_ > 0 ? bufferSize_ : 256;

    *inputLatency  = bs + usbLatencySamples;
    *outputLatency = bs + usbLatencySamples;
    return ASE_OK;
}

ASIOError DelugeASIO::canSampleRate(ASIOSampleRate sampleRate) {
    // Deluge codec is fixed at 44100 Hz
    if (std::fabs(sampleRate - 44100.0) < 1.0) {
        return ASE_OK;
    }
    return ASE_NoClock;
}

ASIOError DelugeASIO::getSampleRate(ASIOSampleRate* sampleRate) {
    *sampleRate = sampleRate_;
    return ASE_OK;
}

ASIOError DelugeASIO::setSampleRate(ASIOSampleRate sampleRate) {
    if (std::fabs(sampleRate - 44100.0) < 1.0) {
        sampleRate_ = 44100.0;
        return ASE_OK;
    }
    return ASE_NoClock;
}

ASIOError DelugeASIO::getClockSources(ASIOClockSource* clocks, long* numSources) {
    if (!clocks || !numSources) return ASE_InvalidParameter;
    clocks[0].index = 0;
    clocks[0].associatedChannel = -1;
    clocks[0].associatedGroup = -1;
    clocks[0].isCurrentSource = ASIOTrue;
    strcpy_s(clocks[0].name, 32, "Internal Crystal");
    *numSources = 1;
    return ASE_OK;
}

ASIOError DelugeASIO::setClockSource(long reference) {
    return (reference == 0) ? ASE_OK : ASE_InvalidParameter;
}

ASIOError DelugeASIO::getChannelInfo(ASIOChannelInfo* info) {
    if (!initialized_) return ASE_NotPresent;

    if (info->isInput) {
        if (info->channel < 0 || info->channel >= kNumInputChannels)
            return ASE_InvalidParameter;
        info->type = ASIOSTInt32LSB;
        info->channelGroup = 0;
        info->isActive = ASIOFalse;
        // Check if this channel has been activated
        for (auto& cb : channelBuffers_) {
            if (cb.isInput && cb.channel == info->channel) {
                info->isActive = ASIOTrue;
                break;
            }
        }
        sprintf_s(info->name, 32, "Mic %s", info->channel == 0 ? "L" : "R");
    } else {
        if (info->channel < 0 || info->channel >= kNumOutputChannels)
            return ASE_InvalidParameter;
        info->type = ASIOSTInt32LSB;
        info->channelGroup = 0;
        info->isActive = ASIOFalse;
        for (auto& cb : channelBuffers_) {
            if (!cb.isInput && cb.channel == info->channel) {
                info->isActive = ASIOTrue;
                break;
            }
        }
        sprintf_s(info->name, 32, "Speaker %s", info->channel == 0 ? "L" : "R");
    }
    return ASE_OK;
}

// ============================================================================
// ASIO: createBuffers / disposeBuffers
// ============================================================================

ASIOError DelugeASIO::createBuffers(ASIOBufferInfo* bufferInfos, long numChannels,
                                     long bufferSize, ASIOCallbacks* callbacks) {
    if (!initialized_) return ASE_NotPresent;
    if (buffersCreated_) disposeBuffers();
    if (!callbacks) return ASE_InvalidParameter;

    bufferSize_ = bufferSize;
    callbacks_ = callbacks;

    // Allocate double-buffered channel storage
    channelBuffers_.resize(numChannels);

    for (long i = 0; i < numChannels; i++) {
        auto& cb = channelBuffers_[i];
        cb.isInput = (bufferInfos[i].isInput != 0);
        cb.channel = bufferInfos[i].channelNum;

        // Allocate two buffers per channel (double-buffer)
        cb.buffers[0] = new int32_t[bufferSize]();
        cb.buffers[1] = new int32_t[bufferSize]();

        // Return buffer pointers to host
        bufferInfos[i].buffers[0] = cb.buffers[0];
        bufferInfos[i].buffers[1] = cb.buffers[1];
    }

    // Allocate ring buffers for USB ↔ ASIO bridging
    // Ring needs to hold several ASIO buffer sizes to absorb USB timing jitter
    ringSize_ = bufferSize * 4 * kNumInputChannels; // 4× the buffer size, interleaved
    inRing_.resize(ringSize_, 0);
    outRing_.resize(ringSize_, 0);
    inRingWr_ = inRingRd_ = 0;
    outRingWr_ = outRingRd_ = 0;

    toggle_ = 0;
    samplePosition_ = 0.0;
    buffersCreated_ = true;

    return ASE_OK;
}

ASIOError DelugeASIO::disposeBuffers() {
    if (started_) stop();

    for (auto& cb : channelBuffers_) {
        delete[] cb.buffers[0];
        delete[] cb.buffers[1];
        cb.buffers[0] = cb.buffers[1] = nullptr;
    }
    channelBuffers_.clear();

    inRing_.clear();
    outRing_.clear();
    ringSize_ = 0;
    callbacks_ = nullptr;
    buffersCreated_ = false;

    return ASE_OK;
}

// ============================================================================
// ASIO: start / stop
// ============================================================================

ASIOError DelugeASIO::start() {
    if (!initialized_ || !buffersCreated_) return ASE_NotPresent;
    if (started_) return ASE_OK;

    samplePosition_ = 0.0;
    toggle_ = 0;

    // Pre-fill the output ring with 2 ASIO buffers of silence.
    // This prevents underruns caused by the mismatch between ASIO buffer size
    // (256 samples ≈ 5.8 USB transfers) and USB transfer granularity (~44 samples).
    // Without this head-start, the output ring drains faster than it refills,
    // causing periodic 1-transfer gaps (~170 Hz buzz).
    inRingWr_ = inRingRd_ = 0;
    outRingWr_ = outRingRd_ = 0;
    const uint32_t prefillSamples = static_cast<uint32_t>(bufferSize_) * 2 * kNumOutputChannels;
    for (uint32_t i = 0; i < prefillSamples; i++) {
        outRing_[outRingWr_] = 0;
        outRingWr_ = (outRingWr_ + 1) % ringSize_;
    }

    // Start USB streaming with 24-bit alt setting
    auto cb = [this](const int32_t* in, int32_t* out, uint32_t n) {
        this->audioCallback(in, out, n);
    };

    if (!usb_.start(deluge::kAlt24Bit, cb)) {
        strcpy_s(errorMessage_, "Failed to start USB audio streaming.");
        return ASE_HWMalfunction;
    }

    started_ = true;
    return ASE_OK;
}

ASIOError DelugeASIO::stop() {
    if (!started_) return ASE_OK;

    usb_.stop();
    started_ = false;
    return ASE_OK;
}

// ============================================================================
// ASIO: sample position
// ============================================================================

ASIOError DelugeASIO::getSamplePosition(ASIOSamples* sPos, ASIOTimeStamp* tStamp) {
    if (!initialized_) return ASE_NotPresent;

    // Convert double position to ASIOSamples (hi:lo pair)
    double pos = samplePosition_;
    sPos->hi = static_cast<unsigned long>(pos / kTwoRaisedTo32);
    sPos->lo = static_cast<unsigned long>(pos - (sPos->hi * kTwoRaisedTo32));

    getNanoSeconds(tStamp);
    return ASE_OK;
}

// ============================================================================
// ASIO: misc
// ============================================================================

ASIOError DelugeASIO::controlPanel() {
    // No control panel UI
    return ASE_OK;
}

ASIOError DelugeASIO::future(long selector, void* opt) {
    return ASE_InvalidParameter;
}

ASIOError DelugeASIO::outputReady() {
    // We support outputReady — it tells us the host has finished writing
    // output buffers, so we can push data immediately
    return ASE_OK;
}

// ============================================================================
// Audio callback — called from USB transport thread
// ============================================================================

void DelugeASIO::audioCallback(const int32_t* inSamples, int32_t* outSamples, uint32_t numSamples) {
    if (!started_ || !callbacks_) return;

    diagCallbackCount_.fetch_add(1, std::memory_order_relaxed);

    std::lock_guard<std::mutex> lock(ringMutex_);

    // Write incoming mic samples to input ring buffer (interleaved)
    for (uint32_t i = 0; i < numSamples * kNumInputChannels; i++) {
        inRing_[inRingWr_] = inSamples[i];
        inRingWr_ = (inRingWr_ + 1) % ringSize_;
    }

    // Check if we have enough input samples for a complete ASIO buffer
    uint32_t inAvail = (inRingWr_ >= inRingRd_)
                           ? (inRingWr_ - inRingRd_)
                           : (ringSize_ - inRingRd_ + inRingWr_);

    const uint32_t neededSamples = static_cast<uint32_t>(bufferSize_) * kNumInputChannels;

    while (inAvail >= neededSamples) {
        // Deinterleave input ring → ASIO input channel buffers
        for (long s = 0; s < bufferSize_; s++) {
            int32_t left  = inRing_[inRingRd_]; inRingRd_ = (inRingRd_ + 1) % ringSize_;
            int32_t right = inRing_[inRingRd_]; inRingRd_ = (inRingRd_ + 1) % ringSize_;

            for (auto& cb : channelBuffers_) {
                if (cb.isInput && cb.channel == 0) cb.buffers[toggle_][s] = left;
                if (cb.isInput && cb.channel == 1) cb.buffers[toggle_][s] = right;
            }
        }

        // Invoke host buffer switch callback
        samplePosition_ += bufferSize_;

        if (callbacks_->bufferSwitchTimeInfo) {
            ASIOTime asioTime = {};
            asioTime.timeInfo.flags = kSystemTimeValid | kSamplePositionValid | kSampleRateValid;
            asioTime.timeInfo.sampleRate = sampleRate_;

            double pos = samplePosition_;
            asioTime.timeInfo.samplePosition.hi = static_cast<unsigned long>(pos / kTwoRaisedTo32);
            asioTime.timeInfo.samplePosition.lo = static_cast<unsigned long>(pos - (asioTime.timeInfo.samplePosition.hi * kTwoRaisedTo32));
            getNanoSeconds(&asioTime.timeInfo.systemTime);

            callbacks_->bufferSwitchTimeInfo(&asioTime, toggle_, ASIOTrue);
        } else if (callbacks_->bufferSwitch) {
            callbacks_->bufferSwitch(toggle_, ASIOTrue);
        }

        // Read output from the host's output buffers → output ring
        for (long s = 0; s < bufferSize_; s++) {
            int32_t left = 0, right = 0;
            for (auto& cb : channelBuffers_) {
                if (!cb.isInput && cb.channel == 0) left  = cb.buffers[toggle_][s];
                if (!cb.isInput && cb.channel == 1) right = cb.buffers[toggle_][s];
            }
            outRing_[outRingWr_] = left;  outRingWr_ = (outRingWr_ + 1) % ringSize_;
            outRing_[outRingWr_] = right; outRingWr_ = (outRingWr_ + 1) % ringSize_;
        }

        // Flip double-buffer toggle
        toggle_ ^= 1;

        diagSwitchCount_.fetch_add(1, std::memory_order_relaxed);

        inAvail -= neededSamples;
    }

    // Read from output ring → USB out buffer
    uint32_t outAvail = (outRingWr_ >= outRingRd_)
                            ? (outRingWr_ - outRingRd_)
                            : (ringSize_ - outRingRd_ + outRingWr_);

    const uint32_t outNeeded = numSamples * kNumOutputChannels;
    if (outAvail >= outNeeded) {
        for (uint32_t i = 0; i < outNeeded; i++) {
            outSamples[i] = outRing_[outRingRd_];
            outRingRd_ = (outRingRd_ + 1) % ringSize_;
        }
    } else {
        // Not enough output data — fill with silence
        diagOutUnderruns_.fetch_add(1, std::memory_order_relaxed);
        std::memset(outSamples, 0, numSamples * kNumOutputChannels * sizeof(int32_t));
    }
}
