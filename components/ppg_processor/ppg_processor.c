/*
 * File: ppg_processor.c
 * Author: Daniel Bishara
 * Description: Multi-channel PPG signal processing pipeline.
 *
 *   Step 1 – NCC:          Normalized Cross-Correlation between Green and IR
 *                          as a motion/signal quality indicator.
 *   Step 2 – Peak detect:  Time-domain BPM from Green signal peak intervals.
 *   Step 3 – FFT fallback: When NCC < NCC_FFT_THRESH, use IR as noise reference
 *                          and find the Green spectral peak that avoids it.
 *   Step 4 – Fusion:       Select BPM source and compute confidence based on NCC.
 *   Step 5 – Plausibility: Drop confidence to 0 if BPM is outside [40, 220].
 *
 * Target: nRF54L15 (Cortex-M33) with Zephyr RTOS + CMSIS-DSP
 */

#include "ppg_processor.h"

#include <arm_math.h>
#include <math.h>
#include <string.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER( ppg_processor, CONFIG_LOG_DEFAULT_LEVEL );

/* ── Constants ─────────────────────────────────────────────────────────────── */

#define SAMPLE_RATE_HZ      100.0f

#define BPM_MIN             40.0f
#define BPM_MAX             220.0f

/* NCC thresholds for sensor fusion */
#define NCC_HIGH            0.8f    /* trust time-domain BPM above this */
#define NCC_LOW             0.5f    /* signal unreliable below this      */

/*
 * Maximum FFT size — static scratch buffers are allocated to this.
 * Must be a power of 2. 1024 covers up to 1024 samples (~5 s at 200 Hz).
 * The FFT operates on min(buf_size, PPG_MAX_FFT_SIZE) samples rounded down
 * to the nearest power of 2, so buf_size does not need to be a power of 2.
 */
#define PPG_MAX_FFT_SIZE    1024U

/*
 * ±5 BPM exclusion window around the IR noise peak (in bins).
 * Computed at runtime from buf_size, but clamped to at least 1.
 */
#define FFT_NOISE_BPM_WINDOW  5.0f

#define MAX_PEAKS           64U

/* ── Module state ───────────────────────────────────────────────────────────── */

static float                       s_lastValidBpm  = 0.0f; /* 0 = no valid BPM yet */
static arm_rfft_fast_instance_f32  s_fftInstance;
static uint32_t                    s_fftInitedSize = 0;    /* 0 = not yet inited */

/* Returns the largest power of 2 that is <= n, capped at PPG_MAX_FFT_SIZE */
static uint32_t floorPow2( uint32_t n )
{
    if ( n > PPG_MAX_FFT_SIZE ) { n = PPG_MAX_FFT_SIZE; }
    uint32_t p = 1;
    while ( ( p << 1 ) <= n ) { p <<= 1; }
    return p;
}

/* Static scratch buffers sized to the maximum supported buffer (kept off stack) */
static float32_t s_greenFftIn [PPG_MAX_FFT_SIZE];
static float32_t s_irFftIn    [PPG_MAX_FFT_SIZE];
static float32_t s_greenFftOut[PPG_MAX_FFT_SIZE];
static float32_t s_irFftOut   [PPG_MAX_FFT_SIZE];

/* ── Step 1: Normalized Cross-Correlation ───────────────────────────────────── */

/*
 * NCC = dot(a, b) / sqrt(power(a) * power(b))
 * For zero-centered buffers this is the Pearson correlation coefficient.
 * Returns a value in [-1, 1]; 1 = identical waveform shapes.
 */
static float computeNcc( const float *a, const float *b, uint32_t len )
{
    float dot, powerA, powerB;
    arm_dot_prod_f32( a, b, len, &dot );
    arm_power_f32( a, len, &powerA );
    arm_power_f32( b, len, &powerB );
    float denom = sqrtf( powerA * powerB );
    return ( denom < 1e-6f ) ? 0.0f : ( dot / denom );
}

/* ── Step 2: Time-domain peak detection ─────────────────────────────────────── */

/*
 * Detects peaks in a zero-centered (AC) buffer using threshold + hysteresis.
 *
 * @param outStability  Output RR-interval stability score [0, 1]; 1 = perfectly regular.
 * @return              BPM computed from mean RR interval, or 0 if detection failed.
 */
static float peakDetectionBpm( const float *buf, uint32_t len, float *outStability )
{
    float    maxVal;
    uint32_t maxIdx;
    arm_max_f32( buf, len, &maxVal, &maxIdx );

    if ( maxVal < 1e-6f ) { *outStability = 0.0f; return 0.0f; }

    /* Threshold at 30 % of signal peak (works well for AC-only signals) */
    float threshold = 0.3f * maxVal;

    /* Minimum samples between peaks based on physiological max (220 BPM) */
    uint32_t minPeakDist = (uint32_t)( SAMPLE_RATE_HZ * 60.0f / BPM_MAX );

    uint32_t peakIdx[MAX_PEAKS];
    uint16_t peakCount      = 0;
    bool     lookingForPeak = true;

    for ( uint32_t i = 1; i < len - 1 && peakCount < MAX_PEAKS; i++ )
    {
        if ( lookingForPeak )
        {
            if ( buf[i] > threshold   &&
                 buf[i] >= buf[i - 1] &&
                 buf[i] >= buf[i + 1] )
            {
                if ( peakCount == 0 || ( i - peakIdx[peakCount - 1] ) >= minPeakDist )
                {
                    peakIdx[peakCount++] = i;
                    lookingForPeak = false;
                }
            }
        }
        else
        {
            /* Wait for signal to drop below threshold before accepting next peak */
            if ( buf[i] < threshold ) { lookingForPeak = true; }
        }
    }

    if ( peakCount < 2 ) { *outStability = 0.0f; return 0.0f; }

    /* Compute mean RR interval and coefficient of variation */
    uint16_t rrCount = peakCount - 1;
    float    rrIntervals[MAX_PEAKS - 1];
    float    rrSum = 0.0f;

    for ( uint16_t i = 0; i < rrCount; i++ )
    {
        rrIntervals[i]  = (float)( peakIdx[i + 1] - peakIdx[i] );
        rrSum          += rrIntervals[i];
    }

    float avgRR = rrSum / (float)rrCount;

    float rrVariance = 0.0f;
    for ( uint16_t i = 0; i < rrCount; i++ )
    {
        float d    = rrIntervals[i] - avgRR;
        rrVariance += d * d;
    }

    /* CV = std / mean.  CV = 0 → perfectly stable. */
    float cv      = sqrtf( rrVariance / (float)rrCount ) / avgRR;
    *outStability = ( cv >= 1.0f ) ? 0.0f : ( 1.0f - cv );

    return 60.0f * SAMPLE_RATE_HZ / avgRR;
}

/* ── Step 3: Frequency-domain BPM with noise cancellation ──────────────────── */

/*
 * Runs an N-point real FFT on both buffers (N = buf_size).
 * Uses the IR spectrum as a motion-noise reference and finds the Green
 * spectral peak that avoids it.
 *
 * FFT output packing (arm_rfft_fast_f32):
 *   pOut[0]      = DC   (real only)
 *   pOut[1]      = Nyquist (real only)
 *   pOut[2k]     = real part of bin k   (k = 1 … N/2-1)
 *   pOut[2k + 1] = imag part of bin k
 *
 * Bin-to-BPM:  BPM[k] = k * SAMPLE_RATE_HZ * 60 / buf_size
 */
static float fftBpm( const float *greenBuf, const float *irBuf, uint32_t buf_size )
{
    /* Clamp to a valid power-of-2 FFT size ≤ PPG_MAX_FFT_SIZE */
    uint32_t fftSize = floorPow2( buf_size );

    if ( fftSize != s_fftInitedSize )
    {
        arm_rfft_fast_init_f32( &s_fftInstance, fftSize );
        s_fftInitedSize = fftSize;
    }

    /* Only copy fftSize samples — safe to stay within scratch buffer bounds */
    memcpy( s_greenFftIn, greenBuf, fftSize * sizeof( float32_t ) );
    memcpy( s_irFftIn,    irBuf,    fftSize * sizeof( float32_t ) );

    arm_rfft_fast_f32( &s_fftInstance, s_greenFftIn, s_greenFftOut, 0 );
    arm_rfft_fast_f32( &s_fftInstance, s_irFftIn,    s_irFftOut,    0 );

    /* Compute physiological bin range from fftSize at runtime */
    float binsPerBpm = (float)fftSize / ( SAMPLE_RATE_HZ * 60.0f );
    int   binMin     = (int)ceilf ( BPM_MIN * binsPerBpm );
    int   binMax     = (int)floorf( BPM_MAX * binsPerBpm );
    int   noiseWin   = (int)ceilf ( FFT_NOISE_BPM_WINDOW * binsPerBpm );
    if ( noiseWin < 1 ) { noiseWin = 1; }

    /* Find dominant IR bin in physiological range (noise reference) */
    float irMaxMag  = 0.0f;
    int   irPeakBin = -1;

    for ( int k = binMin; k <= binMax; k++ )
    {
        float re  = s_irFftOut[2 * k];
        float im  = s_irFftOut[2 * k + 1];
        float mag = re * re + im * im;
        if ( mag > irMaxMag ) { irMaxMag = mag; irPeakBin = k; }
    }

    /* Find dominant Green bin that does not overlap the IR noise peak */
    float greenMaxMag  = 0.0f;
    int   greenPeakBin = -1;

    for ( int k = binMin; k <= binMax; k++ )
    {
        if ( irPeakBin >= 0                        &&
             k >= ( irPeakBin - noiseWin )         &&
             k <= ( irPeakBin + noiseWin ) )
        {
            continue; /* skip bins contaminated by IR noise */
        }

        float re  = s_greenFftOut[2 * k];
        float im  = s_greenFftOut[2 * k + 1];
        float mag = re * re + im * im;
        if ( mag > greenMaxMag ) { greenMaxMag = mag; greenPeakBin = k; }
    }

    if ( greenPeakBin < 0 ) { return 0.0f; }

    float freqHz = (float)greenPeakBin * SAMPLE_RATE_HZ / (float)fftSize;
    return freqHz * 60.0f;
}

/* ── Public API ─────────────────────────────────────────────────────────────── */

void process_ppg_data( float *green_buf, float *ir_buf, uint32_t buf_size,
                       float *out_bpm,   float *out_confidence )
{
    /* Guard: buf_size must be a non-zero power of 2 for arm_rfft_fast_f32 */
    if ( buf_size == 0 || ( buf_size & ( buf_size - 1 ) ) != 0 )
    {
        LOG_ERR( "buf_size %u is not a power of 2 — aborting", buf_size );
        *out_bpm        = 0.0f;
        *out_confidence = 0.0f;
        return;
    }

    /* ── Step 1: Signal correlation ──────────────────────────────────────── */
    float r = computeNcc( green_buf, ir_buf, buf_size );
    /* Clamp to [0, 1]: negative NCC (anti-correlated) is treated as no correlation */
    if ( r < 0.0f ) { r = 0.0f; }
    LOG_DBG( "NCC (r): %.3f", r );

    /* ── Step 2: Time-domain peak detection ──────────────────────────────── */
    float rrStability = 0.0f;
    float tdBpm       = peakDetectionBpm( green_buf, buf_size, &rrStability );
    LOG_DBG( "TD BPM: %.1f  RR stability: %.3f", tdBpm, rrStability );

    float finalBpm   = 0.0f;
    float confidence = 0.0f;

    /* ── Steps 3 & 4: Sensor fusion ──────────────────────────────────────── */
    if ( r > NCC_HIGH )
    {
        /* High correlation — waveforms are coherent, trust time-domain BPM.
         * Confidence scales with both signal quality and RR regularity. */
        finalBpm   = tdBpm;
        confidence = r * rrStability;
        LOG_DBG( "Fusion: high NCC → TD BPM %.1f (conf %.2f)", finalBpm, confidence );
    }
    else if ( r > NCC_LOW )
    {
        /* Medium correlation — motion artifacts present.
         * Use FFT-based BPM with IR noise cancellation. */
        float fdBpm = fftBpm( green_buf, ir_buf, buf_size );
        LOG_DBG( "Fusion: mid NCC → FFT BPM %.1f", fdBpm );
        finalBpm   = ( fdBpm > 0.0f ) ? fdBpm : tdBpm;
        confidence = r * 0.8f;
    }
    else
    {
        /* Low correlation — signal unreliable.
         * Return last valid BPM if one exists; confidence stays below 20 % (r < 0.5 → r*0.4 < 0.2).
         * If no valid BPM has ever been computed, leave finalBpm = 0 so the
         * plausibility check below zeroes confidence and the caller discards it. */
        finalBpm   = s_lastValidBpm;
        confidence = ( s_lastValidBpm > 0.0f ) ? ( r * 0.4f ) : 0.0f;
        LOG_DBG( "Fusion: low NCC → last valid BPM %.1f (conf %.2f)", finalBpm, confidence );
    }

    /* ── Step 5: Physiological plausibility check ────────────────────────── */
    if ( finalBpm < BPM_MIN || finalBpm > BPM_MAX )
    {
        LOG_WRN( "BPM %.1f outside physiological range [%.0f, %.0f] — confidence zeroed",
                 finalBpm, BPM_MIN, BPM_MAX );
        confidence = 0.0f;
    }
    else
    {
        s_lastValidBpm = finalBpm;
    }

    *out_bpm        = finalBpm;
    *out_confidence = confidence;
}
