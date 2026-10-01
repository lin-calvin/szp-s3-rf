#include "esp_heap_caps.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <string.h>
#include "lvgl.h"

#include "esp_sdr/receiver.h"
#include "fft.h"
#include "lvgl_port.h"
#include "st7789.h"
#include "touch.h"
#include "ui.h"

#define FFT_MAX       1024
#define FFT_RATE_DIV  1   /* 0 = 80, 1 = 40, 6 = 16 MS/s */
#define FFT_RATE_MSPS 40

/* Capture period tiers in ms (FPS button cycles through these). */
static const int kPeriodTiers[] = {200, 100, 50, 25}; /* 5/10/20/40 fps */
#define PERIOD_TIERS_N (sizeof(kPeriodTiers) / sizeof(kPeriodTiers[0]))

/* 1 = panel bring-up test pattern only (red/green/blue bars), 0 = normal UI. */
#define SZP_DISPLAY_TEST 0
/* 1 = start the RF backend; 0 = LVGL-only bring-up (no WiFi/PHY/RF). */
#define SZP_RF_ENABLE    1

#define CAPTURE_CORE 0
#define UI_CORE      1

/* Gestures only apply inside the chart area (display coords); the title and
 * status strip are handled by LVGL widgets. */
#define CHART_Y0 22
#define CHART_Y1 186

static float s_spec_shared[FFT_MAX];
static volatile int s_spec_len;
static SemaphoreHandle_t s_spec_lock;
static volatile uint32_t s_spec_seq;
static volatile int g_fft_n = 512;
static volatile int g_period_ms = 100;
static volatile bool s_interactive;

/* View: 0 = normal (40 MHz around centre), 1 = ISM sweep, 2 = full sweep. */
static volatile int g_view;

#define OVW_BINS 320
static float s_ovw_shared[OVW_BINS];
static volatile uint32_t s_ovw_seq;

/* ---- button actions (called from the UI task via LVGL events) ---- */
static void act_cycle_n(void)
{
    static const int tiers[] = {128, 256, 512, 1024};
    int i = 0;
    for (i = 0; i < 4; i++) if (tiers[i] == g_fft_n) break;
    g_fft_n = tiers[(i + 1) & 3];
}
static void act_toggle_gain(void) { s3_gain_toggle(); }
static void act_cycle_fps(void)
{
    int i = 0;
    for (i = 0; i < (int)PERIOD_TIERS_N; i++) if (kPeriodTiers[i] == g_period_ms) break;
    g_period_ms = kPeriodTiers[(i + 1) % PERIOD_TIERS_N];
}
static void act_set_freq(unsigned mhz)
{
    s3_set_frequency_mhz(mhz);
    if (g_view != 0) {
        g_view = 0;
        ui_set_view(0);
    }
}
static void act_cycle_view(void)
{
    g_view = (g_view + 1) % 3;
    ui_set_view(g_view);
}

/* Sweep [start,stop] MHz with 40 MHz windows, max-hold into 320 bins. */
static void sweep_window(int c, int n, float bin_mhz, int start, int stop, float *ovw)
{
    s3_set_frequency_mhz((unsigned)c);
    esp_rom_delay_us(200);            /* PLL settle */
    static float spec[FFT_MAX];
    if (!s3_rf_capture(n, FFT_RATE_DIV)) return;
    dsp_fft_iq(s3_iq_words(), spec);
    for (int k = 0; k < n; k++) {
        int off = (k <= n / 2) ? k : k - n;
        float f = (float)c + (float)off * bin_mhz;
        int j = (int)((f - (float)start) * (float)OVW_BINS / (float)(stop - start));
        if (j >= 0 && j < OVW_BINS && spec[k] > ovw[j]) ovw[j] = spec[k];
    }
}

static void sweep_once(int view, float *ovw)
{
    int start = (view == 1) ? 2400 : 100;
    int stop = (view == 1) ? 2484 : 3000;
    const int span = 40;
    /* Small step: every frequency must fall in the flat middle of some window,
     * otherwise the DC-removal notch at each window centre shows as a dark
     * stripe (and the window edges are rolled off by the analog filter). */
    const int step = 10;
    int n = g_fft_n;
    float bin_mhz = (float)FFT_RATE_MSPS / (float)n;
    static float filled[OVW_BINS];
    if (dsp_fft_size() != n) return;

    for (int j = 0; j < OVW_BINS; j++) ovw[j] = -120.0f;

    int c0 = start + span / 2;
    int c1 = stop - span / 2;
    for (int c = c0; ; ) {
        if (g_view != view) return;   /* view changed mid-sweep */
        sweep_window(c, n, bin_mhz, start, stop, ovw);
        if (c >= c1) break;
        c += step;
        if (c > c1) c = c1;           /* always cover the right edge */
    }

    /* Fill narrow notches (window-centre DC dips) from neighbours. */
    filled[0] = ovw[0];
    filled[OVW_BINS - 1] = ovw[OVW_BINS - 1];
    for (int j = 1; j < OVW_BINS - 1; j++) {
        float m = ovw[j - 1];
        if (ovw[j] > m) m = ovw[j];
        if (ovw[j + 1] > m) m = ovw[j + 1];
        filled[j] = m;
    }
    memcpy(ovw, filled, sizeof(filled));
}

/* ---- RF capture / FFT / sweep task (core 0) ---- */
static void capture_task(void *arg)
{
    (void)arg;
    static float spec[FFT_MAX];
    static float ovw[OVW_BINS];
    int applied_n = 0;

    for (;;) {
        s3_cli_poll();
        int n = g_fft_n;
        if (n != applied_n && dsp_fft_init(n) == 0) {
            applied_n = n;
        }
        if (applied_n != n) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (g_view == 0) {
            if (!s3_host_active() && s3_rf_capture(n, FFT_RATE_DIV)) {
                dsp_fft_iq(s3_iq_words(), spec);
                xSemaphoreTake(s_spec_lock, portMAX_DELAY);
                memcpy(s_spec_shared, spec, sizeof(float) * n);
                s_spec_len = n;
                s_spec_seq++;
                xSemaphoreGive(s_spec_lock);
            }
            vTaskDelay(pdMS_TO_TICKS(s_interactive ? 0 : g_period_ms));
        } else {
            if (!s3_host_active()) {
                int view = g_view;
                sweep_once(view, ovw);
                if (g_view == view) {
                    xSemaphoreTake(s_spec_lock, portMAX_DELAY);
                    memcpy(s_ovw_shared, ovw, sizeof(ovw));
                    s_ovw_seq++;
                    xSemaphoreGive(s_spec_lock);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(30));
        }
    }
}

/* ---- UI / touch task (core 1) ---- */
static void ui_task(void *arg)
{
    (void)arg;
    static float spec[FFT_MAX];
#if !SZP_RF_ENABLE
    for (int i = 0; i < FFT_MAX; i++) {
        spec[i] = -60.0f + 30.0f * (float)(i % 7) / 6.0f;
    }
#endif

    bool touching = false, on_ui = false;
    int gsx = 0, gsy = 0, glx = 0, gly = 0, gmode = 0;
    float tune_accum = 0.0f, gain_accum = 0.0f;
    bool lp_fired = false;
    int64_t press_t0 = 0, last_wf = 0;
    uint32_t seen = 0, seen_ovw = 0;
    static float ovw[OVW_BINS];
    int n = 512;

    for (;;) {
        unsigned freq = 2412;
        int gain = -1;
#if SZP_RF_ENABLE
        freq = s3_frequency_mhz();
        gain = s3_gain_current();
#endif
        if (s_spec_seq != seen) {
            xSemaphoreTake(s_spec_lock, portMAX_DELAY);
            n = s_spec_len;
            if (n < 16) n = 16;
            if (n > FFT_MAX) n = FFT_MAX;
            memcpy(spec, s_spec_shared, sizeof(float) * n);
            seen = s_spec_seq;
            xSemaphoreGive(s_spec_lock);
        }

        if (ui_get_view() != 0 && s_ovw_seq != seen_ovw) {
            xSemaphoreTake(s_spec_lock, portMAX_DELAY);
            memcpy(ovw, s_ovw_shared, sizeof(ovw));
            seen_ovw = s_ovw_seq;
            xSemaphoreGive(s_spec_lock);
            ui_overview_update(ovw, OVW_BINS);
            if (ui_get_mode() == 1) ui_waterfall_push(ovw, OVW_BINS, 0);
        }

        int64_t now = esp_timer_get_time();

        /* Chart-area gestures (taps/buttons elsewhere go to LVGL):
         *   long press      -> toggle spectrum/waterfall
         *   horizontal drag -> tune
         *   vertical drag   -> gain */
        int tx, ty;
        bool down = touch_read(&tx, &ty);
        if (down) {
            if (!touching) {
                press_t0 = now;
                lp_fired = false;
                gsx = glx = tx;
                gsy = gly = ty;
                gmode = 0;
                tune_accum = gain_accum = 0.0f;
                int dpy = 239 - ty;   /* display coords */
                on_ui = ui_keypad_active() || dpy < CHART_Y0 || dpy > CHART_Y1;
            } else if (!on_ui) {
                if (!lp_fired && gmode == 0 && (now - press_t0) > 600000 &&
                    abs(tx - gsx) < 12 && abs(ty - gsy) < 12) {
                    ui_set_mode(ui_get_mode() ? 0 : 1);
                    lp_fired = true;
                }
                if (!lp_fired) {
                    if (gmode == 0) {
                        int ax = abs(tx - gsx), ay = abs(ty - gsy);
                        if (ax >= 6 || ay >= 6) gmode = (ax >= ay) ? 1 : 2;
                    }
                    int dx = tx - glx, dy = ty - gly;
                    if (gmode == 1) {
                        tune_accum += (float)dx * (float)FFT_RATE_MSPS / 320.0f;
                        int step = (int)tune_accum;
                        if (step != 0) {
                            long f = (long)s3_frequency_mhz() + step;
                            if (f < 100) f = 100;
                            if (f > 6000) f = 6000;
                            s3_set_frequency_mhz((unsigned)f);
                            freq = (unsigned)f;
                            tune_accum -= (float)step;
                        }
                    } else if (gmode == 2) {
                        int gmax = (int)s3_gain_max();
                        if (gmax <= 0) gmax = 82;
                        gain_accum += (float)(dy) * (float)gmax / 240.0f;
                        int step = (int)gain_accum;
                        if (step != 0) {
                            s3_gain_step(step);
                            gain = s3_gain_current();
                            gain_accum -= (float)step;
                        }
                    }
                }
                glx = tx;
                gly = ty;
            }
        }
        touching = down;
        s_interactive = down && !on_ui;

        ui_update(spec, n, freq, FFT_RATE_MSPS, gain, 1000 / g_period_ms);
        if (ui_get_view() == 0 && ui_get_mode() == 1 &&
            (now - last_wf) >= (int64_t)g_period_ms * 1000) {
            ui_waterfall_push(spec, n, freq);
            last_wf = now;
        }
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

void app_main(void)
{
#if SZP_DISPLAY_TEST
    st7789_init();
    st7789_test_pattern();
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
#else
    st7789_init();
    lvgl_port_init();
    lvgl_port_add_touch();

    static const ui_actions_t actions = {
        .cycle_n = act_cycle_n,
        .toggle_gain = act_toggle_gain,
        .cycle_fps = act_cycle_fps,
        .cycle_view = act_cycle_view,
        .set_freq = act_set_freq,
    };
    ui_set_actions(&actions);
    ui_init();

    s_spec_lock = xSemaphoreCreateMutex();

#if SZP_RF_ENABLE
    s3_rf_init();
    s3_set_bandwidth_mhz(0);   /* widest analog bandwidth */
    xTaskCreatePinnedToCore(capture_task, "capture", 8192, NULL, 4, NULL, CAPTURE_CORE);
#endif
    xTaskCreatePinnedToCore(ui_task, "ui", 8192, NULL, 5, NULL, UI_CORE);
#endif
}
