/*
* File: heartrate.cpp
* Author: Daniel Bishara
* Date: March 6, 2026
* Description: define class methods for the HeartRateManager class
*/

#include <zephyr/logging/log.h>
#include <math.h>

#include "heartrate.h"

#define PI 3.14159265f

LOG_MODULE_REGISTER( HeartRateManager, CONFIG_LOG_DEFAULT_LEVEL );

K_THREAD_STACK_DEFINE( heartrateThreadStack, HEARTRATE_THREAD_STACK );

static uint8_t redLedBackingBuffer[HEARTRATE_BUFFER_BYTES];
static uint8_t irLedBackingBuffer[HEARTRATE_BUFFER_BYTES];
static uint8_t greenLedBackingBuffer[HEARTRATE_BUFFER_BYTES];
static uint8_t bpmBackingBuffer[HEARTRATE_BPM_BUFFER_BYTES];
static RingBuffer redLedRingBuffer( redLedBackingBuffer, HEARTRATE_BUFFER_BYTES );
static RingBuffer irLedRingBuffer( irLedBackingBuffer, HEARTRATE_BUFFER_BYTES );
static RingBuffer greenLedRingBuffer( greenLedBackingBuffer, HEARTRATE_BUFFER_BYTES );
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

    redLedBuffer   = &redLedRingBuffer;
    irLedBuffer    = &irLedRingBuffer;
    greenLedBuffer = &greenLedRingBuffer;
    bpmBuffer      = &bpmRingBuffer;

    sampleRate  = sampleRateHz;

    // IIR bandpass: high-pass at 0.5 Hz (DC blocker) + low-pass at 4 Hz
    hpAlpha    = 1.0f - ( 2.0f * PI * 0.5f / sampleRateHz );
    lpAlpha    = ( 2.0f * PI * 4.0f ) / ( 2.0f * PI * 4.0f + sampleRateHz );
    dcEstimate = 0.0f;
    lpState    = 0.0f;

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
        k_sleep( K_SECONDS( 5 ) );
        HeartRateManager::Instance().calculate( &HeartRateManager::Instance().lastBpm );
    }
}


bool HeartRateManager::popBpm( float *outBpm )
{
    if ( !outBpm ) { return false; }
    uint8_t size = sizeof( float );
    return bpmBuffer->pop( outBpm, &size );
}

void HeartRateManager::pushRedLedSample( float sample )
{
    if ( !isInit ) { LOG_ERR( "HeartRateManager not initialized!" ); return; }
    PpgSample ppgSample = { sample, k_uptime_get_32() };
    redLedBuffer->push( &ppgSample, sizeof( PpgSample ) );
}

void HeartRateManager::pushIrLedSample( float sample )
{
    if ( !isInit ) { LOG_ERR( "HeartRateManager not initialized!" ); return; }
    PpgSample ppgSample = { sample, k_uptime_get_32() };
    irLedBuffer->push( &ppgSample, sizeof( PpgSample ) );
}

void HeartRateManager::pushGreenLedSample( float sample )
{
    if ( !isInit ) { LOG_ERR( "HeartRateManager not initialized!" ); return; }
    PpgSample ppgSample = { sample, k_uptime_get_32() };
    greenLedBuffer->push( &ppgSample, sizeof( PpgSample ) );
}

// @brief Apply IIR bandpass filter in-place to an array of PPG samples (HP at ~0.5 Hz, LP at ~4 Hz)
// @param samples array of PpgSamples to filter
// @param count number of samples
void HeartRateManager::applyBandpassFilter( PpgSample *samples, uint16_t count )
{
    for ( uint16_t i = 0; i < count; i++ )
    {
        float raw      = samples[i].value;
        dcEstimate     = hpAlpha * dcEstimate + ( 1.0f - hpAlpha ) * raw;
        float acSignal = raw - dcEstimate;
        lpState        = lpAlpha * acSignal + ( 1.0f - lpAlpha ) * lpState;
        samples[i].value = lpState;
    }
}

// @brief Calculate heart rate from buffered PPG samples using peak detection
// @param outBpm pointer to store the calculated heart rate in BPM
// @return Error code
ErrCode_t HeartRateManager::calculate( float *outBpm )
{
    ErrCode_t errCode = ErrCode_Internal;

    static constexpr uint16_t MAX_SAMPLES = HEARTRATE_BUFFER_BYTES / sizeof( PpgSample );
    static PpgSample redSamples[MAX_SAMPLES];
    static PpgSample irSamples[MAX_SAMPLES];
    static PpgSample greenSamples[MAX_SAMPLES];
    uint16_t count = 0;

    if ( !isInit ) { LOG_ERR( "HeartRateManager not initialized!" ); goto exit; }

    if ( !outBpm )
    {
        LOG_ERR( "Invalid output pointer!" );
        goto exit;
    }

    // Drain all three ring buffers into local arrays using the same count
    while ( !redLedBuffer->isEmpty() && !irLedBuffer->isEmpty() && !greenLedBuffer->isEmpty() && count < MAX_SAMPLES )
    {
        uint8_t size = sizeof( PpgSample );
        if ( !redLedBuffer->pop( &redSamples[count], &size ) )     { break; }
        if ( !irLedBuffer->pop( &irSamples[count], &size ) )       { break; }
        if ( !greenLedBuffer->pop( &greenSamples[count], &size ) ) { break; }
        count++;
    }


    if ( count < 2 )
    {
        LOG_WRN( "Not enough samples to calculate heart rate (%u samples)", count );
        goto exit;
    }

    {
        applyBandpassFilter( redSamples, count );
        applyBandpassFilter( irSamples, count );
        applyBandpassFilter( greenSamples, count );

        // Calculate mean and stddev for adaptive peak detection threshold
        float mean = 0.0f;
        for ( uint16_t i = 0; i < count; i++ ) { mean += redSamples[i].value; }
        mean /= count;

        float variance = 0.0f;
        for ( uint16_t i = 0; i < count; i++ )
        {
            float d = redSamples[i].value - mean;
            variance += d * d;
        }
        float threshold = mean + 0.3f * sqrtf( variance / count );
        LOG_DBG( "Peak threshold: %.4f (mean=%.4f)", threshold, mean );

        // Minimum time between peaks at physiological maximum of 220 BPM
        static constexpr uint32_t MIN_PEAK_INTERVAL_MS = 60000U / 220U;

        // Peak detection with valley-crossing hysteresis.
        // After accepting a peak the signal must drop back below the threshold before
        // the next peak can be considered. This prevents multiple detections on
        // the same broad PPG pulse at high sample rates.
        uint16_t peakCount = 0;
        uint16_t lastPeakIdx = 0;
        uint32_t peakIntervalSumMs = 0;
        bool lookingForPeak = true;

        for ( uint16_t i = 1; i < count - 1; i++ )
        {
            if ( lookingForPeak )
            {
                if ( redSamples[i].value > threshold &&
                     redSamples[i].value >= redSamples[i - 1].value &&
                     redSamples[i].value >= redSamples[i + 1].value )
                {
                    uint32_t intervalMs = redSamples[i].timestampMs - redSamples[lastPeakIdx].timestampMs;
                    if ( peakCount == 0 || intervalMs >= MIN_PEAK_INTERVAL_MS )
                    {
                        if ( peakCount > 0 ) { peakIntervalSumMs += intervalMs; }
                        lastPeakIdx = i;
                        peakCount++;
                        lookingForPeak = false; // wait for valley before next peak
                    }
                }
            }
            else
            {
                // Wait for signal to fall back below threshold (valley crossing)
                if ( redSamples[i].value < threshold ) { lookingForPeak = true; }
            }
        }

        if ( peakCount < 2 )
        {
            LOG_WRN( "Not enough peaks detected (%u peaks over %u samples)", peakCount, count );
            goto exit;
        }

        float avgIntervalMs = ( float )peakIntervalSumMs / ( peakCount - 1 );
        *outBpm = 60000.0f / avgIntervalMs;

        if ( *outBpm < 40.0f || *outBpm > 180.0f )
        {
            LOG_WRN( "BPM out of physiological range: %.1f (discarding)", *outBpm );
            goto exit;
        }

        bpmBuffer->push( outBpm, sizeof( float ) );
        LOG_INF( "Heart rate: %.1f BPM (%u peaks over %u samples)", *outBpm, peakCount, count );
    }

    errCode = ErrCode_Success;
exit:
    return errCode;
}
