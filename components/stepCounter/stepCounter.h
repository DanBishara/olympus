/*
* File: stepCounter.h
* Author: Daniel Bishara
* Date: March 6, 2026
* Description: declare class methods for the step counter component
*/

#pragma once

#include <stdint.h>
#include <zephyr/kernel.h>

#include "errorCode.h"
#include "ringbuffer.h"

class StepCounter
{
private:
    StepCounter( void );
    ~StepCounter( void ) = default;

    RingBuffer      accelBuf;
    struct k_thread processingThread;

    uint32_t stepCount;
    float    filteredMag;
    bool     aboveThreshold;
    int64_t  lastStepTimeMs;

    static void processingThreadFn( void *p1, void *p2, void *p3 );
    void        processLoop( void );
    void        processUpdate( float x, float y, float z );

public:
    static StepCounter& Instance( void ) { static StepCounter instance; return instance; }
    ErrCode_t init( void );
    void      pushSample( float x, float y, float z );
    uint32_t  getStepCount( void ) const;
    ErrCode_t reset( void );
};
