/*
* File: heartrate.cpp
* Author: Daniel Bishara
* Date: March 6, 2026
* Description: define class methods for the HeartRateManager class
*/

#include <zephyr/logging/log.h>

#include "heartrate.h"

LOG_MODULE_REGISTER( HeartRateManager, CONFIG_LOG_DEFAULT_LEVEL );

K_THREAD_STACK_DEFINE( heartrateThreadStack, HEARTRATE_THREAD_STACK );

static uint8_t ppgBackingBuffer[HEARTRATE_BUFFER_BYTES];
static uint8_t bpmBackingBuffer[HEARTRATE_BPM_BUFFER_BYTES];
static RingBuffer ppgRingBuffer( ppgBackingBuffer, HEARTRATE_BUFFER_BYTES );
static RingBuffer bpmRingBuffer( bpmBackingBuffer, HEARTRATE_BPM_BUFFER_BYTES );

ErrCode_t HeartRateManager::init( float sampleRateHz )
{
    ErrCode_t errCode = ErrCode_Internal;

    if ( isInit )
    {
        LOG_WRN( "HeartRateManager already initialized!" );
        errCode = ErrCode_Success;
        goto exit;
    }

    if ( sampleRateHz <= 0.0f )
    {
        LOG_ERR( "Invalid sample rate!" );
        goto exit;
    }

    ppgBuffer = &ppgRingBuffer;
    bpmBuffer = &bpmRingBuffer;

    sampleRate = sampleRateHz;

    k_thread_create( &thread, heartrateThreadStack, HEARTRATE_THREAD_STACK,
                     threadFunc, NULL, NULL, NULL,
                     K_PRIO_PREEMPT( 10 ), 0, K_NO_WAIT );

    LOG_INF( "HeartRateManager initialized at %.1f Hz", sampleRateHz );

    isInit = true;
    errCode = ErrCode_Success;
exit:
    return errCode;
}

void HeartRateManager::threadFunc( void *p1, void *p2, void *p3 )
{
    while ( true )
    {
        k_sleep( K_SECONDS( 3 ) );
        HeartRateManager::Instance().calculate( &HeartRateManager::Instance().lastBpm );
    }
}

float HeartRateManager::rollingAverage( float inNewSample )
{
    for ( int i = sizeof(smoothingBuffer)/sizeof(smoothingBuffer[0]) - 1; i > 0; i-- )
    {
        smoothingBuffer[i] = smoothingBuffer[i - 1];
    }
    smoothingBuffer[0] = inNewSample;

    float sum = 0;
    for ( int i = 0; i < sizeof(smoothingBuffer)/sizeof(smoothingBuffer[0]); i++ )
    {
        sum += smoothingBuffer[i];
    }
    return sum / ( sizeof(smoothingBuffer)/sizeof(smoothingBuffer[0]) );
}

float HeartRateManager::calculateBaselineCurrent( float inNewSample )
{
    for ( int i = sizeof(baselineBuffer)/sizeof(baselineBuffer[0]) - 1; i > 0; i-- )
    {
        baselineBuffer[i] = baselineBuffer[i - 1];
    }
    baselineBuffer[0] = inNewSample;

    float sum = 0;
    for ( int i = 0; i < sizeof(baselineBuffer)/sizeof(baselineBuffer[0]); i++ )
    {
        sum += baselineBuffer[i];
    }
    return sum / ( sizeof(baselineBuffer)/sizeof(baselineBuffer[0]) );
}

bool HeartRateManager::popBpm( float *outBpm )
{
    if ( !outBpm ) { return false; }
    uint8_t size = sizeof( float );
    return bpmBuffer->pop( outBpm, &size );
}

void HeartRateManager::pushSample( float sample )
{
    if ( !isInit ) { LOG_ERR( "HeartRateManager not initialized!" ); return; }
    PpgSample ppgSample = { sample, k_uptime_get_32() };
    ppgBuffer->push( &ppgSample, sizeof( PpgSample ) );
}

// @brief Calculate heart rate from buffered PPG samples using peak detection
// @param outBpm pointer to store the calculated heart rate in BPM
// @return Error code
ErrCode_t HeartRateManager::calculate( float *outBpm )
{
    ErrCode_t errCode = ErrCode_Internal;

    static constexpr uint16_t MAX_SAMPLES = HEARTRATE_BUFFER_BYTES / sizeof( PpgSample );
    static PpgSample samples[MAX_SAMPLES];
    uint16_t count = 0;

    if ( !isInit ) { LOG_ERR( "HeartRateManager not initialized!" ); goto exit; }

    if ( !outBpm )
    {
        LOG_ERR( "Invalid output pointer!" );
        goto exit;
    }

    // Drain ring buffer into local array
    while ( !ppgBuffer->isEmpty() && count < MAX_SAMPLES )
    {
        uint8_t size = sizeof( PpgSample );
        if ( !ppgBuffer->pop( &samples[count], &size ) ) { break; }
        count++;
    }

    if ( count < 2 )
    {
        LOG_WRN( "Not enough samples to calculate heart rate (%u samples)", count );
        goto exit;
    }

    {
        // Apply rolling average then baseline correction to each sample value in place
        for ( uint16_t i = 0; i < count; i++ )
        {
            float smoothed     = rollingAverage( samples[i].value );
            float baseline     = calculateBaselineCurrent( samples[i].value );
            samples[i].value   = smoothed - baseline;
        }

        // Calculate mean for peak detection threshold
        float mean = 0.0f;
        for ( uint16_t i = 0; i < count; i++ ) { mean += samples[i].value; }
        mean /= count;

        // Minimum time between peaks at physiological maximum of 220 BPM
        static constexpr uint32_t MIN_PEAK_INTERVAL_MS = 60000U / 220U;

        // Find local maxima above mean, enforcing minimum peak interval by timestamp
        uint16_t peakCount = 0;
        uint16_t lastPeakIdx = 0;
        uint32_t peakIntervalSumMs = 0;

        for ( uint16_t i = 1; i < count - 1; i++ )
        {
            uint32_t intervalMs = samples[i].timestampMs - samples[lastPeakIdx].timestampMs;

            if ( samples[i].value > mean &&
                 samples[i].value > samples[i - 1].value &&
                 samples[i].value >= samples[i + 1].value &&
                 ( peakCount == 0 || intervalMs >= MIN_PEAK_INTERVAL_MS ) )
            {
                if ( peakCount > 0 )
                {
                    peakIntervalSumMs += intervalMs;
                }
                lastPeakIdx = i;
                peakCount++;
            }
        }

        if ( peakCount < 2 )
        {
            LOG_WRN( "Not enough peaks detected (%u peaks)", peakCount );
            goto exit;
        }

        float avgIntervalMs = ( float )peakIntervalSumMs / ( peakCount - 1 );
        *outBpm = 60000.0f / avgIntervalMs;

        bpmBuffer->push( outBpm, sizeof( float ) );
        LOG_INF( "Heart rate: %.1f BPM (%u peaks over %u samples)", *outBpm, peakCount, count );
    }

    errCode = ErrCode_Success;
exit:
    return errCode;
}
