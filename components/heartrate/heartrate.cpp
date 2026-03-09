/*
* File: heartrate.cpp
* Author: Daniel Bishara
* Date: March 6, 2026
* Description: define class methods for the HeartRateManager class
*/

#include <zephyr/logging/log.h>
#include <math.h>

#include "heartrate.h"
#include "ppg_processor.h"

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
    greenFilter = { 0.0f, 0.0f };
    irFilter    = { 0.0f, 0.0f };

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
void HeartRateManager::applyBandpassFilter( PpgSample *samples, uint16_t count, FilterState &state )
{
    for ( uint16_t i = 0; i < count; i++ )
    {
        float raw          = samples[i].value;
        state.dcEstimate   = hpAlpha * state.dcEstimate + ( 1.0f - hpAlpha ) * raw;
        float acSignal     = raw - state.dcEstimate;
        state.lpState      = lpAlpha * acSignal + ( 1.0f - lpAlpha ) * state.lpState;
        samples[i].value   = state.lpState;
    }
}

// @brief Calculate heart rate from IR and Green LED samples using process_ppg_data
// @param outBpm pointer to store the calculated heart rate in BPM
// @return Error code
ErrCode_t HeartRateManager::calculate( float *outBpm )
{
    ErrCode_t errCode = ErrCode_Internal;

    static constexpr uint16_t MAX_SAMPLES = HEARTRATE_BUFFER_BYTES / sizeof( PpgSample );
    static PpgSample irSamples   [MAX_SAMPLES];
    static PpgSample greenSamples[MAX_SAMPLES];
    static float     irFloats    [MAX_SAMPLES];
    static float     greenFloats [MAX_SAMPLES];
    uint16_t count = 0;

    if ( !isInit ) { LOG_ERR( "HeartRateManager not initialized!" ); goto exit; }

    if ( !outBpm )
    {
        LOG_ERR( "Invalid output pointer!" );
        goto exit;
    }

    // Drain IR and Green ring buffers into local arrays using the same count
    while ( !irLedBuffer->isEmpty() && !greenLedBuffer->isEmpty() && count < MAX_SAMPLES )
    {
        uint8_t size = sizeof( PpgSample );
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
        // Filter each channel independently using its own filter state
        applyBandpassFilter( irSamples,    count, irFilter    );
        applyBandpassFilter( greenSamples, count, greenFilter );

        // Power-of-2 windowing: take the most recent N samples where N is the
        // largest power of 2 <= count, so process_ppg_data receives a valid FFT size
        uint16_t fftCount = 1;
        while ( ( fftCount << 1 ) <= count ) { fftCount <<= 1; }
        uint16_t offset = count - fftCount;

        for ( uint16_t i = 0; i < fftCount; i++ )
        {
            irFloats[i]    = irSamples   [offset + i].value;
            greenFloats[i] = greenSamples[offset + i].value;
        }

        float confidence = 0.0f;
        process_ppg_data( greenFloats, irFloats, fftCount, outBpm, &confidence );

        if ( confidence <= 0.0f )
        {
            LOG_WRN( "Low confidence (%.2f), discarding BPM %.1f", confidence, *outBpm );
            goto exit;
        }

        bpmBuffer->push( outBpm, sizeof( float ) );
        LOG_INF( "Heart rate: %.1f BPM (confidence: %.2f, %u/%u samples used)", *outBpm, confidence, fftCount, count );
    }

    errCode = ErrCode_Success;
exit:
    return errCode;
}
