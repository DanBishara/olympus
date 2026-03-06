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
* Algorithm: dual-gate step detection using accelerometer and gyroscope.
*   Both signals are smoothed with an exponential moving average (EMA).
*   A step is counted on the rising edge of the accel magnitude crossing
*   STEP_ACCEL_THRESHOLD_MS2, but only when the gyro magnitude is also
*   above STEP_GYRO_THRESHOLD_RPS. The gyro gate rejects false positives
*   (e.g. device impacts) that produce an accel spike without the angular
*   velocity characteristic of an arm swing during walking.
*/

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <math.h>

#include "stepCounter.h"

// Accel magnitude threshold in m/s². Resting magnitude ≈ 9.81 (gravity).
// Walking peaks typically reach 11-15 m/s².
#define STEP_ACCEL_THRESHOLD_MS2    ( 11.5f )

// Gyro magnitude threshold in rad/s. Arm swing during walking produces ~1-3 rad/s.
// Motions without a corresponding arm swing (e.g. device impact) fall below this.
#define STEP_GYRO_THRESHOLD_RPS     ( 1.0f )

// Minimum time between consecutive steps in ms (guards against double-counting)
#define MIN_STEP_INTERVAL_MS        ( 250 )

// EMA coefficient. Higher = faster response, lower = more smoothing.
// At 12.5 Hz sampling, 0.3 gives ~0.7 Hz cutoff — passes walking cadence (1-2 Hz).
#define LPF_ALPHA                   ( 0.3f )

// Ring buffer sizing: each message is sizeof(ImuSample) payload + 1 byte header
#define IMU_BUF_SAMPLES             ( 32 )

#define STEP_COUNTER_STACK_SIZE     ( 1024 )
#define STEP_COUNTER_PRIORITY       ( 5 )

LOG_MODULE_REGISTER( StepCounter, CONFIG_LOG_DEFAULT_LEVEL );

struct ImuSample
{
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
};

static uint8_t imuBufMem[ IMU_BUF_SAMPLES * ( sizeof(ImuSample) + sizeof(uint8_t) ) ];
K_THREAD_STACK_DEFINE( stepCounterStack, STEP_COUNTER_STACK_SIZE );

/// @brief Initialize the step counter and start the processing thread
/// @return Error code
ErrCode_t StepCounter::init( void )
{
    imuBuf.init( imuBufMem, sizeof(imuBufMem) );

    stepCount           = 0;
    filteredAccel       = 0.0f;
    filteredGyro        = 0.0f;
    aboveAccelThreshold = false;
    lastStepTimeMs      = 0;

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

/// @brief Push a raw IMU sample into the ring buffer.
///        Safe to call from the sensor trigger callback context.
/// @param ax Acceleration in m/s² along the X axis
/// @param ay Acceleration in m/s² along the Y axis
/// @param az Acceleration in m/s² along the Z axis
/// @param gx Angular velocity in rad/s around the X axis
/// @param gy Angular velocity in rad/s around the Y axis
/// @param gz Angular velocity in rad/s around the Z axis
void StepCounter::pushSample( float ax, float ay, float az,
                               float gx, float gy, float gz )
{
    ImuSample sample = { ax, ay, az, gx, gy, gz };

    if( !imuBuf.push( &sample, sizeof(sample) ) )
    {
        LOG_WRN( "IMU buffer full, sample dropped!" );
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
    imuBuf.clear();
    stepCount           = 0;
    filteredAccel       = 0.0f;
    filteredGyro        = 0.0f;
    aboveAccelThreshold = false;
    lastStepTimeMs      = 0;

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
        imuBuf.waitForData( K_FOREVER );

        while( !imuBuf.isEmpty() )
        {
            if( imuBuf.pop( &sample, &size ) )
            {
                processUpdate( sample.ax, sample.ay, sample.az,
                               sample.gx, sample.gy, sample.gz );
            }
        }
    }
}

/// @brief Run dual-gate step detection on a single IMU sample
/// @param ax Acceleration in m/s² along the X axis
/// @param ay Acceleration in m/s² along the Y axis
/// @param az Acceleration in m/s² along the Z axis
/// @param gx Angular velocity in rad/s around the X axis
/// @param gy Angular velocity in rad/s around the Y axis
/// @param gz Angular velocity in rad/s around the Z axis
void StepCounter::processUpdate( float ax, float ay, float az,
                                  float gx, float gy, float gz )
{
    float accelMag = sqrtf( ax*ax + ay*ay + az*az );
    float gyroMag  = sqrtf( gx*gx + gy*gy + gz*gz );

    filteredAccel = LPF_ALPHA * accelMag + ( 1.0f - LPF_ALPHA ) * filteredAccel;
    filteredGyro  = LPF_ALPHA * gyroMag  + ( 1.0f - LPF_ALPHA ) * filteredGyro;

    int64_t now = k_uptime_get();

    bool accelPeak  = filteredAccel > STEP_ACCEL_THRESHOLD_MS2 && !aboveAccelThreshold;
    bool gyroActive = filteredGyro  > STEP_GYRO_THRESHOLD_RPS;

    // Rising-edge detection: accel peak must coincide with active arm swing (gyro gate)
    if( accelPeak && gyroActive )
    {
        aboveAccelThreshold = true;
        if( ( now - lastStepTimeMs ) >= MIN_STEP_INTERVAL_MS )
        {
            stepCount++;
            lastStepTimeMs = now;
            LOG_DBG( "Step detected! Total: %u", stepCount );
        }
    }
    else if( filteredAccel <= STEP_ACCEL_THRESHOLD_MS2 )
    {
        aboveAccelThreshold = false;
    }
}
