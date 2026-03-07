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

#define HEARTRATE_BUFFER_BYTES      ( 2000 ) // holds 500 float samples
#define HEARTRATE_THREAD_STACK      ( 1024 )
#define HEARTRATE_BPM_BUFFER_BYTES  ( 80 )   // holds 20 BPM measurements

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
    HeartRateManager( void ) : ppgBuffer( backingBuffer, HEARTRATE_BUFFER_BYTES ),
                               bpmBuffer( backingBpmBuffer, HEARTRATE_BPM_BUFFER_BYTES ) {}
    ~HeartRateManager( void ) = default;
    float rollingAverage( float inNewSample );
    float calculateBaselineCurrent( float inNewSample );
    static void threadFunc( void *p1, void *p2, void *p3 );
    bool isInit = false;
    float sampleRate = 0.0f;
    float lastBpm = 0.0f;
    float smoothingBuffer[5] = {0};
    float baselineBuffer[100] = {0};
    uint8_t backingBuffer[HEARTRATE_BUFFER_BYTES];
    uint8_t backingBpmBuffer[HEARTRATE_BPM_BUFFER_BYTES];
    RingBuffer ppgBuffer;
    RingBuffer bpmBuffer;
    struct k_thread thread;
    K_THREAD_STACK_MEMBER( threadStack, HEARTRATE_THREAD_STACK );
};
