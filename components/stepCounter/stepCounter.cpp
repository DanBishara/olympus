/*
* File: stepCounter.cpp
* Author: Daniel Bishara
* Date: March 6, 2026
* Description: define class methods for the step counter component
*
* Architecture:
*   The accelerometer trigger callback calls pushSample(), which is the only
*   operation done in that context. Samples are packed into a RingBuffer as
*   ImuSample structs. A dedicated processing thread blocks on waitForData()
*   and drains the buffer, running the step detection algorithm.
*
* Algorithm: exponential moving average (EMA) low-pass filter on acceleration
* magnitude, followed by rising-edge peak detection with a debounce guard.
*
* At rest the LSM6DSOX reports ~9.81 m/s² (gravity). Each footfall creates a
* magnitude spike above STEP_THRESHOLD_MS2. The EMA smooths high-frequency
* vibration noise while preserving the ~1-2 Hz walking cadence.
*/

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <math.h>

#include "stepCounter.h"

// Magnitude threshold in m/s². Resting magnitude ≈ 9.81 (gravity).
// Walking peaks typically reach 11-15 m/s².
#define STEP_THRESHOLD_MS2      ( 11.5f )

// Minimum time between consecutive steps in ms (guards against double-counting)
#define MIN_STEP_INTERVAL_MS    ( 250 )

// EMA coefficient. Higher = faster response, lower = more smoothing.
// At 12.5 Hz sampling, 0.3 gives ~0.7 Hz cutoff — passes walking cadence (1-2 Hz).
#define LPF_ALPHA               ( 0.3f )

// Ring buffer sizing: each message is sizeof(ImuSample) payload + 1 byte header
#define ACCEL_BUF_SAMPLES       ( 32 )
#define ACCEL_BUF_CAPACITY      ( ACCEL_BUF_SAMPLES * ( sizeof(ImuSample) + sizeof(uint8_t) ) )

#define STEP_COUNTER_STACK_SIZE ( 1024 )
#define STEP_COUNTER_PRIORITY   ( 5 )

LOG_MODULE_REGISTER( StepCounter, CONFIG_LOG_DEFAULT_LEVEL );

struct ImuSample
{
    float x;
    float y;
    float z;
};

static uint8_t accelBufMem[ ACCEL_BUF_SAMPLES * ( sizeof(ImuSample) + sizeof(uint8_t) ) ];
K_THREAD_STACK_DEFINE( stepCounterStack, STEP_COUNTER_STACK_SIZE );

StepCounter::StepCounter( void ) : accelBuf( accelBufMem, sizeof(accelBufMem) )
{
    stepCount      = 0;
    filteredMag    = 0.0f;
    aboveThreshold = false;
    lastStepTimeMs = 0;
}

/// @brief Initialize the step counter and start the processing thread
/// @return Error code
ErrCode_t StepCounter::init( void )
{
    k_thread_create( &processingThread,
                     stepCounterStack,
                     K_THREAD_STACK_SIZEOF( stepCounterStack ),
                     processingThreadFn,
                     this, NULL, NULL,
                     STEP_COUNTER_PRIORITY, 0, K_NO_WAIT );

    k_thread_name_set( &processingThread, "step_counter" );

    LOG_INF( "Step counter initialized!" );
    return ErrCode_Success;
}

/// @brief Push a raw accelerometer sample into the ring buffer.
///        Safe to call from the sensor trigger callback context.
/// @param x Acceleration in m/s² along the X axis
/// @param y Acceleration in m/s² along the Y axis
/// @param z Acceleration in m/s² along the Z axis
void StepCounter::pushSample( float x, float y, float z )
{
    ImuSample sample = { x, y, z };

    if( !accelBuf.push( &sample, sizeof(sample) ) )
    {
        LOG_WRN( "Accel buffer full, sample dropped!" );
    }
}

/// @brief Get the current accumulated step count
/// @return Number of steps counted since init() or the last reset()
uint32_t StepCounter::getStepCount( void ) const
{
    return stepCount;
}

/// @brief Reset the step count and flush any buffered samples
/// @return Error code
ErrCode_t StepCounter::reset( void )
{
    accelBuf.clear();
    stepCount      = 0;
    aboveThreshold = false;
    lastStepTimeMs = 0;

    LOG_INF( "Step count reset!" );
    return ErrCode_Success;
}

/// @brief Zephyr thread entry point
void StepCounter::processingThreadFn( void *p1, void *p2, void *p3 )
{
    static_cast<StepCounter *>( p1 )->processLoop();
}

/// @brief Drains the ring buffer and runs step detection on each sample
void StepCounter::processLoop( void )
{
    ImuSample sample;
    uint8_t   size;

    while( 1 )
    {
        accelBuf.waitForData( K_FOREVER );

        while( !accelBuf.isEmpty() )
        {
            if( accelBuf.pop( &sample, &size ) )
            {
                processUpdate( sample.x, sample.y, sample.z );
            }
        }
    }
}

/// @brief Run step detection on a single sample
/// @param x Acceleration in m/s² along the X axis
/// @param y Acceleration in m/s² along the Y axis
/// @param z Acceleration in m/s² along the Z axis
void StepCounter::processUpdate( float x, float y, float z )
{
    float magnitude = sqrtf( x*x + y*y + z*z );

    // Exponential moving average low-pass filter
    filteredMag = LPF_ALPHA * magnitude + ( 1.0f - LPF_ALPHA ) * filteredMag;

    int64_t now = k_uptime_get();

    // Rising-edge detection: count a step on the transition from below to above threshold
    if( filteredMag > STEP_THRESHOLD_MS2 && !aboveThreshold )
    {
        aboveThreshold = true;
        if( ( now - lastStepTimeMs ) >= MIN_STEP_INTERVAL_MS )
        {
            stepCount++;
            lastStepTimeMs = now;
            LOG_DBG( "Step detected! Total: %u", stepCount );
        }
    }
    else if( filteredMag <= STEP_THRESHOLD_MS2 )
    {
        aboveThreshold = false;
    }
}
