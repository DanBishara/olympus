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

#define PPG_BUF_SIZE        256U
#define SAMPLE_RATE_HZ      100.0f

#define BPM_MIN             40.0f
#define BPM_MAX             220.0f

/* NCC thresholds for sensor fusion */
#define NCC_HIGH            0.8f    /* trust time-domain BPM above this */
#define NCC_LOW             0.5f    /* signal unreliable below this      */
#define NCC_FFT_THRESH      0.7f    /* trigger FFT fallback below this   */

/*
 * Physiological BPM bin range for a 256-point FFT at 100 Hz:
 *   freq_hz[k] = k * 100 / 256
 *   BPM[k]     = k * 6000 / 256
 *
 *   k_min = ceil (40  * 256 / 6000) = 2
 *   k_max = floor(220 * 256 / 6000) = 9
 */
#define FFT_BIN_MIN         2
#define FFT_BIN_MAX         9

/*
 * ±5 BPM exclusion window around the IR noise peak:
 *   Δk = 5 BPM * 256 / 6000 ≈ 0.21 bins → round up to 1 bin
 */
#define FFT_NOISE_WINDOW    1

#define MAX_PEAKS           64U

/* ── Module state ───────────────────────────────────────────────────────────── */

static float                       s_lastValidBpm = 60.0f;
static arm_rfft_fast_instance_f32  s_fftInstance;
static bool                        s_fftReady     = false;

/* Static scratch buffers (keep off the stack) */
static float32_t s_greenFftIn [PPG_BUF_SIZE];
static float32_t s_irFftIn    [PPG_BUF_SIZE];
static float32_t s_greenFftOut[PPG_BUF_SIZE];
static float32_t s_irFftOut   [PPG_BUF_SIZE];

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
 * Detects peaks in a zero-centered (AC) buffer using threshold + hysteresis,
 * matching the existing HeartRateManager approach.
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
    uint16_t peakCount   = 0;
    bool     lookingForPeak = true;

    for ( uint32_t i = 1; i < len - 1 && peakCount < MAX_PEAKS; i++ )
    {
        if ( lookingForPeak )
        {
            if ( buf[i] > threshold      &&
                 buf[i] >= buf[i - 1]    &&
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

    /* Compute mean RR interval and its coefficient of variation */
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

    /* Coefficient of variation: std / mean.  CV = 0 → perfectly stable. */
    float cv       = sqrtf( rrVariance / (float)rrCount ) / avgRR;
    *outStability  = ( cv >= 1.0f ) ? 0.0f : ( 1.0f - cv );

    return 60.0f * SAMPLE_RATE_HZ / avgRR;
}

/* ── Step 3: Frequency-domain BPM with noise cancellation ──────────────────── */

/*
 * Runs a 256-point real FFT on both buffers.
 * Uses the IR spectrum as a motion-noise reference and finds the Green
 * spectral peak that avoids the IR dominant frequency (±FFT_NOISE_WINDOW bins).
 *
 * FFT output packing (arm_rfft_fast_f32, N = 256):
 *   pOut[0]      = DC   (real only)
 *   pOut[1]      = Nyquist (real only)
 *   pOut[2k]     = real part of bin k   (k = 1 … N/2-1)
 *   pOut[2k + 1] = imag part of bin k
 */
static float fftBpm( const float *greenBuf, const float *irBuf )
{
    memcpy( s_greenFftIn, greenBuf, PPG_BUF_SIZE * sizeof( float32_t ) );
    memcpy( s_irFftIn,    irBuf,    PPG_BUF_SIZE * sizeof( float32_t ) );

    arm_rfft_fast_f32( &s_fftInstance, s_greenFftIn, s_greenFftOut, 0 );
    arm_rfft_fast_f32( &s_fftInstance, s_irFftIn,    s_irFftOut,    0 );

    /* Find dominant IR bin in physiological range (noise reference) */
    float irMaxMag  = 0.0f;
    int   irPeakBin = -1;

    for ( int k = FFT_BIN_MIN; k <= FFT_BIN_MAX; k++ )
    {
        float re  = s_irFftOut[2 * k];
        float im  = s_irFftOut[2 * k + 1];
        float mag = re * re + im * im;
        if ( mag > irMaxMag ) { irMaxMag = mag; irPeakBin = k; }
    }

    /* Find dominant Green bin that does not overlap the IR noise peak */
    float greenMaxMag  = 0.0f;
    int   greenPeakBin = -1;

    for ( int k = FFT_BIN_MIN; k <= FFT_BIN_MAX; k++ )
    {
        if ( irPeakBin >= 0                           &&
             k >= ( irPeakBin - FFT_NOISE_WINDOW )    &&
             k <= ( irPeakBin + FFT_NOISE_WINDOW ) )
        {
            continue; /* skip bins contaminated by IR noise */
        }

        float re  = s_greenFftOut[2 * k];
        float im  = s_greenFftOut[2 * k + 1];
        float mag = re * re + im * im;
        if ( mag > greenMaxMag ) { greenMaxMag = mag; greenPeakBin = k; }
    }

    if ( greenPeakBin < 0 ) { return 0.0f; }

    float freqHz = (float)greenPeakBin * SAMPLE_RATE_HZ / (float)PPG_BUF_SIZE;
    return freqHz * 60.0f;
}

/* ── Public API ─────────────────────────────────────────────────────────────── */

void process_ppg_data( float *green_buf, float *ir_buf,
                       float *out_bpm,   float *out_confidence )
{
    if ( !s_fftReady )
    {
        arm_rfft_fast_init_f32( &s_fftInstance, PPG_BUF_SIZE );
        s_fftReady = true;
    }

    /* ── Step 1: Signal correlation ──────────────────────────────────────── */
    float r = computeNcc( green_buf, ir_buf, PPG_BUF_SIZE );
    LOG_DBG( "NCC (r): %.3f", r );

    /* ── Step 2: Time-domain peak detection ──────────────────────────────── */
    float rrStability = 0.0f;
    float tdBpm       = peakDetectionBpm( green_buf, PPG_BUF_SIZE, &rrStability );
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
        float fdBpm = fftBpm( green_buf, ir_buf );
        LOG_DBG( "Fusion: mid NCC → FFT BPM %.1f", fdBpm );
        finalBpm   = ( fdBpm > 0.0f ) ? fdBpm : tdBpm;
        confidence = r * 0.8f;
    }
    else
    {
        /* Low correlation — signal unreliable.
         * Return last valid BPM; confidence stays below 20 % (r < 0.5 → r*0.4 < 0.2). */
        finalBpm   = s_lastValidBpm;
        confidence = r * 0.4f;
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
