/*
* File: heartrate.h
* Author: Daniel Bishara
* Date: March 6, 2026
* Description: declare class methods for the HeartRateManager class
*/

#pragma once

#include "errorCode.h"
#include "ringbuffer.h"

class HeartRateManager
{
public:
    static HeartRateManager& Instance( void ) { static HeartRateManager instance; return instance; }
    ErrCode_t init( float sampleRateHz );
    void      pushSample( float sample );
    ErrCode_t calculate( float *outBpm );

private:
    static constexpr uint16_t BUFFER_BYTES = 2000; // holds 500 float samples
    HeartRateManager( void ) : ppgBuffer( backingBuffer, BUFFER_BYTES ) {}
    ~HeartRateManager( void ) = default;
    bool isInit = false;
    float sampleRate = 0.0f;
    uint8_t backingBuffer[BUFFER_BYTES];
    RingBuffer ppgBuffer;
};
