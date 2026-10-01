#pragma once

#include <stdint.h>

/* On-chip complex FFT for the S3 RF dump. Radix-2, float, Hann windowed.
 * Point counts are modest (256..4096); no external DSP dependency. */

/* Allocate for an n-point transform (n a power of two, 16..4096).
 * Returns 0 on success, -1 on invalid size or allocation failure. */
int dsp_fft_init(int n);
void dsp_fft_free(void);
int dsp_fft_size(void);

/* Transform one snapshot of packed I/Q words (bits 0..9 signed I,
 * bits 10..19 signed Q). Writes n power values in dBFS, natural FFT order
 * (bin 0 = DC ... bin n-1 = just below the sample rate). A full-scale tone
 * reads about 0 dB. */
void dsp_fft_iq(const uint32_t *words, float *out_db);
