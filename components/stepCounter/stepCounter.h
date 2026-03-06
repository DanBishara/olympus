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
    StepCounter( void ) = default;
    ~StepCounter( void ) = default;

    RingBuffer      imuBuf;
    struct k_thread processingThread;

    uint32_t stepCount;
    float    filteredAccel;
    float    filteredGyro;
    bool     aboveAccelThreshold;
    int64_t  lastStepTimeMs;

    static void processingThreadFn( void *p1, void *p2, void *p3 );
    void        processLoop( void );
    void        processUpdate( float ax, float ay, float az,
                               float gx, float gy, float gz );

public:
    static StepCounter& Instance( void ) { static StepCounter instance; return instance; }
    ErrCode_t init( void );
    void      pushSample( float ax, float ay, float az,
                          float gx, float gy, float gz );
    uint32_t  getStepCount( void ) const;
    ErrCode_t reset( void );
};
