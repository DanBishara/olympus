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

#define HEARTRATE_BUFFER_BYTES      20000 // holds 250 PpgSample structs (8 bytes each)
#define HEARTRATE_THREAD_STACK      1024
#define HEARTRATE_BPM_BUFFER_BYTES  800   // holds 20 BPM measurements

struct PpgSample
{
    float    value;
    uint32_t timestampMs;
};

class HeartRateManager
{
public:
    static HeartRateManager& Instance( void ) { static HeartRateManager instance; return instance; }
    ErrCode_t init( float sampleRateHz );
    void      pushSample( float sample );
    ErrCode_t calculate( float *outBpm );
    float     getLastBpm( void ) const { return lastBpm; }
    bool      popBpm( float *outBpm );

private:
    HeartRateManager( void ) = default;
    ~HeartRateManager( void ) = default;
    float rollingAverage( float inNewSample );
    float calculateBaselineCurrent( float inNewSample );
    static void threadFunc( void *p1, void *p2, void *p3 );
    bool isInit;
    float sampleRate;
    float lastBpm;
    float smoothingBuffer[5];
    float baselineBuffer[100];
    RingBuffer *ppgBuffer;
    RingBuffer *bpmBuffer;
    struct k_thread thread;
};
