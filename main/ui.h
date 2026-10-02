#pragma once

#include <stdbool.h>

/* Spectrum analyzer screen (LVGL 8): spectrum chart / waterfall, tappable
 * status bar (N / gain / fps) and a touch frequency keypad. */
void ui_init(void);

/* Actions invoked from the on-screen buttons (implemented in main). */
typedef struct {
    void (*cycle_n)(void);        /* N button: next FFT size */
    void (*toggle_gain)(void);    /* G button: AGC <-> manual */
    void (*cycle_fps)(void);      /* FPS button: next capture tier */
    void (*cycle_view)(void);     /* MODE button: 40mhz -> ism -> overview */
    void (*set_freq)(unsigned mhz); /* keypad OK */
} ui_actions_t;
void ui_set_actions(const ui_actions_t *a);

/* Display view: 0 = normal ("40mhz"), 1 = ISM sweep, 2 = full overview sweep. */
void ui_set_view(int view);
int  ui_get_view(void);

/* Push a sweep result: n == 320 overview bins in dB. */
void ui_overview_update(const float *db, int n);

/* Marker line on the sweep chart at freq_mhz (view 1 or 2); 0 clears it. */
void ui_set_marker(int view, unsigned freq_mhz);

/* PC mode: on-device capture stops, the serial CLI owns the radio. */
void ui_set_pc_mode(bool on);

/* mode: 0 = spectrum line chart, 1 = waterfall. */
void ui_set_mode(int mode);
int  ui_get_mode(void);

/* True while the frequency keypad is open (chart gestures disabled). */
bool ui_keypad_active(void);

/* Push a new spectrum (n bins, natural FFT order) and status line.
 * gain is -1 for hardware AGC, else the manual gain index. */
void ui_update(const float *db, int n, unsigned freq_mhz, int sample_rate_msps,
               int gain, int fps);

/* Push one waterfall line. centered = the data is a DC-centred spectrum
 * (40mhz view); otherwise it is already in absolute frequency order
 * (left = start, right = stop; sweep views). */
void ui_waterfall_push(const float *db, int n, unsigned freq_mhz, bool centered);
