/*
* File: heartrate.h
* Author: Daniel Bishara
* Date: March 6, 2026
* Description: declare class methods for the HeartRateManager class
*/

#pragma once

#include <zephyr/kernel.h>

#include "errorCode.h"
#include "ringbuffer.h"

// PPG ring buffer: 1250 PpgSamples (8 bytes each) = ~6.25 s at 200 Hz
// Sized to hold at least 2 peak intervals at the minimum detectable rate (~30 BPM = 2 s/peak)
#define HEARTRATE_BUFFER_BYTES      10000
#define HEARTRATE_THREAD_STACK      1024
#define HEARTRATE_BPM_BUFFER_BYTES  80    // holds 20 BPM float readings

struct PpgSample
{
    float    value;
    uint32_t timestampMs;
};

struct FilterState
{
    float dcEstimate;
    float lpState;
};

class HeartRateManager
{
public:
    static HeartRateManager& Instance( void ) { static HeartRateManager instance; return instance; }
    ErrCode_t init( float sampleRateHz );
    void      pushRedLedSample( float sample );
    void      pushIrLedSample( float sample );
    void      pushGreenLedSample( float sample );
    ErrCode_t calculate( float *outBpm );
    float     getLastBpm( void ) const { return lastBpm; }
    bool      popBpm( float *outBpm );

private:
    HeartRateManager( void ) = default;
    ~HeartRateManager( void ) = default;
    static void threadFunc( void *p1, void *p2, void *p3 );
    void applyBandpassFilter( PpgSample *samples, uint16_t count, FilterState &state );
    bool isInit;
    float sampleRate;
    float lastBpm;
    float hpAlpha;          // IIR high-pass coefficient (DC blocker, ~0.5 Hz)
    float lpAlpha;          // IIR low-pass coefficient (~4 Hz)
    FilterState greenFilter; // independent filter state for the Green LED channel
    FilterState irFilter;    // independent filter state for the IR LED channel
    RingBuffer *redLedBuffer;
    RingBuffer *irLedBuffer;
    RingBuffer *greenLedBuffer;
    RingBuffer *bpmBuffer;
    struct k_thread thread;
};
