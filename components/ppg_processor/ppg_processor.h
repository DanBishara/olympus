/*
 * File: ppg_processor.h
 * Author: Daniel Bishara
 * Description: Multi-channel PPG signal processing with motion compensation.
 *              Accepts pre-filtered, zero-centered Green and IR buffers and
 *              returns heart rate BPM and a confidence score [0.0, 1.0].
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * @brief Process pre-filtered PPG data to compute heart rate and confidence.
 *
 * @param green_buf      Pre-filtered, zero-centered Green LED buffer
 * @param ir_buf         Pre-filtered, zero-centered IR LED buffer
 * @param buf_size       Number of samples in each buffer (must be a power of 2 for FFT)
 * @param out_bpm        Output heart rate in BPM
 * @param out_confidence Output signal confidence [0.0, 1.0]
 */
void process_ppg_data( float *green_buf, float *ir_buf, uint32_t buf_size,
                       float *out_bpm,   float *out_confidence );

#ifdef __cplusplus
}
#endif
