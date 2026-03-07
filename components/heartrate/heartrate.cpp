/*
* File: heartrate.cpp
* Author: Daniel Bishara
* Date: March 6, 2026
* Description: define class methods for the HeartRateManager class
*/

#include <zephyr/logging/log.h>

#include "heartrate.h"

LOG_MODULE_REGISTER( HeartRateManager, CONFIG_LOG_DEFAULT_LEVEL );

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

    sampleRate = sampleRateHz;

    LOG_INF( "HeartRateManager initialized at %.1f Hz", sampleRateHz );

    isInit = true;
    errCode = ErrCode_Success;
exit:
    return errCode;
}

void HeartRateManager::pushSample( float sample )
{
    if ( !isInit ) { LOG_ERR( "HeartRateManager not initialized!" ); return; }
    ppgBuffer.push( &sample, sizeof( float ) );
}

// @brief Calculate heart rate from buffered PPG samples using peak detection
// @param outBpm pointer to store the calculated heart rate in BPM
// @return Error code
ErrCode_t HeartRateManager::calculate( float *outBpm )
{
    ErrCode_t errCode = ErrCode_Internal;

    static constexpr uint16_t MAX_SAMPLES = 500;
    static float samples[MAX_SAMPLES];
    uint16_t count = 0;

    if ( !isInit ) { LOG_ERR( "HeartRateManager not initialized!" ); goto exit; }

    if ( !outBpm )
    {
        LOG_ERR( "Invalid output pointer!" );
        goto exit;
    }

    // Drain ring buffer into local array
    while ( !ppgBuffer.isEmpty() && count < MAX_SAMPLES )
    {
        uint8_t size = sizeof( float );
        if ( !ppgBuffer.pop( &samples[count], &size ) ) { break; }
        count++;
    }

    if ( count < 2 )
    {
        LOG_WRN( "Not enough samples to calculate heart rate (%u samples)", count );
        goto exit;
    }

    {
        // Calculate mean for peak detection threshold
        float mean = 0.0f;
        for ( uint16_t i = 0; i < count; i++ ) { mean += samples[i]; }
        mean /= count;

        // Find local maxima above mean
        uint16_t peakCount = 0;
        uint16_t lastPeakIdx = 0;
        uint32_t peakIntervalSum = 0;

        for ( uint16_t i = 1; i < count - 1; i++ )
        {
            if ( samples[i] > mean &&
                 samples[i] > samples[i - 1] &&
                 samples[i] >= samples[i + 1] )
            {
                if ( peakCount > 0 )
                {
                    peakIntervalSum += ( i - lastPeakIdx );
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

        float avgIntervalSamples = ( float )peakIntervalSum / ( peakCount - 1 );
        float avgIntervalSeconds = avgIntervalSamples / sampleRate;
        *outBpm = 60.0f / avgIntervalSeconds;

        LOG_INF( "Heart rate: %.1f BPM (%u peaks over %u samples)", *outBpm, peakCount, count );
    }

    errCode = ErrCode_Success;
exit:
    return errCode;
}
