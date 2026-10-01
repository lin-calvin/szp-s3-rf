#include "fft.h"

#include <math.h>
#include <stdlib.h>

#include "esp_heap_caps.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static int s_n;
static float *s_win;   /* Hann window, symmetric */
static float *s_twr;   /* cos twiddles, n/2 */
static float *s_twi;   /* sin twiddles, n/2 */
static float *s_re;    /* working real part */
static float *s_im;    /* working imaginary part */
static float *s_src_re;
static float *s_src_im;
static uint16_t *s_brev; /* bit-reversal permutation */

void dsp_fft_free(void)
{
    free(s_win);   s_win = NULL;
    free(s_twr);   s_twr = NULL;
    free(s_twi);   s_twi = NULL;
    free(s_re);    s_re = NULL;
    free(s_im);    s_im = NULL;
    free(s_src_re); s_src_re = NULL;
    free(s_src_im); s_src_im = NULL;
    free(s_brev);  s_brev = NULL;
    s_n = 0;
}

int dsp_fft_size(void) { return s_n; }

int dsp_fft_init(int n)
{
    if (n < 16 || n > 4096 || (n & (n - 1)) != 0) {
        return -1;
    }
    dsp_fft_free();
    s_win = malloc(sizeof(float) * n);
    s_twr = malloc(sizeof(float) * (n / 2));
    s_twi = malloc(sizeof(float) * (n / 2));
    s_re = malloc(sizeof(float) * n);
    s_im = malloc(sizeof(float) * n);
    s_src_re = malloc(sizeof(float) * n);
    s_src_im = malloc(sizeof(float) * n);
    s_brev = malloc(sizeof(uint16_t) * n);
    if (!s_win || !s_twr || !s_twi || !s_re || !s_im || !s_src_re || !s_src_im || !s_brev) {
        dsp_fft_free();
        return -1;
    }
    s_n = n;

    int bits = 0;
    while ((1 << bits) < n) {
        bits++;
    }
    for (int i = 0; i < n; i++) {
        s_win[i] = 0.5f - 0.5f * cosf((float)(2.0 * M_PI * i / (n - 1)));
        int r = 0;
        for (int b = 0; b < bits; b++) {
            if (i & (1 << b)) {
                r |= 1 << (bits - 1 - b);
            }
        }
        s_brev[i] = (uint16_t)r;
    }
    for (int k = 0; k < n / 2; k++) {
        float a = (float)(-2.0 * M_PI * k / n);
        s_twr[k] = cosf(a);
        s_twi[k] = sinf(a);
    }
    return 0;
}

void dsp_fft_iq(const uint32_t *words, float *out_db)
{
    const int n = s_n;
    float mi = 0.0f, mq = 0.0f;
    for (int i = 0; i < n; i++) {
        int iv = (int)(words[i] & 0x3ffu);
        int qv = (int)((words[i] >> 10) & 0x3ffu);
        if (iv >= 512) iv -= 1024;
        if (qv >= 512) qv -= 1024;
        s_src_re[i] = iv * (1.0f / 512.0f);
        s_src_im[i] = qv * (1.0f / 512.0f); /* +Q so +freq is on the right */
        mi += s_src_re[i];
        mq += s_src_im[i];
    }

    /* DC removal: zero-IF receivers have a large DC offset that shows up as a
     * centre spike. Subtracting the per-capture mean removes the DC bin. */
    mi /= (float)n;
    mq /= (float)n;

    for (int i = 0; i < n; i++) {
        int j = s_brev[i];
        s_re[i] = (s_src_re[j] - mi) * s_win[j];
        s_im[i] = (s_src_im[j] - mq) * s_win[j];
    }

    for (int len = 2; len <= n; len <<= 1) {
        int half = len >> 1;
        int step = n / len;
        for (int i = 0; i < n; i += len) {
            for (int k = 0; k < half; k++) {
                float wr = s_twr[k * step];
                float wi = s_twi[k * step];
                int a = i + k;
                int b = a + half;
                float tr = s_re[b] * wr - s_im[b] * wi;
                float ti = s_re[b] * wi + s_im[b] * wr;
                s_re[b] = s_re[a] - tr;
                s_im[b] = s_im[a] - ti;
                s_re[a] += tr;
                s_im[a] += ti;
            }
        }
    }

    /* Hann gain is n/2 on the tone amplitude -> n/4 on a full-scale complex
     * tone in the complex FFT; normalise so that reads ~0 dB. */
    const float ref = 20.0f * log10f((float)n * 0.25f);
    for (int k = 0; k < n; k++) {
        float p = s_re[k] * s_re[k] + s_im[k] * s_im[k];
        out_db[k] = 10.0f * log10f(p + 1e-12f) - ref;
    }
}
