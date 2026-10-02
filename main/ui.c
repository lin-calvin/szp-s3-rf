#include "ui.h"

#include <stdio.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "lvgl.h"
#include "st7789.h"

#define PLOT_W 320
#define PLOT_H 168
#define PLOT_Y 20

#define DB_MIN (-100)
#define DB_MAX (0)

#define WF_DB_LO (-100.0f)
#define WF_DB_HI (-30.0f)

static const char *TAG = "ui";

static lv_obj_t *s_chart;
static lv_chart_series_t *s_ser;
static lv_obj_t *s_canvas;
static lv_obj_t *s_title;
static lv_obj_t *s_lbl_n;
static lv_obj_t *s_lbl_g;
static lv_obj_t *s_lbl_f;
static lv_obj_t *s_lbl_mode;
static lv_obj_t *s_ax_left;
static lv_obj_t *s_ax_mid;
static lv_obj_t *s_ax_right;
static lv_obj_t *s_ovw_chart;
static lv_chart_series_t *s_ovw_ser;
static int s_view;   /* 0=40mhz, 1=ism, 2=overview */
static int s_marker_view;
static unsigned s_marker_mhz;
static lv_obj_t *s_marker_obj;
static lv_obj_t *s_marker_inner;
static bool s_pc_mode;
static lv_obj_t *s_pc_overlay;

static lv_obj_t *s_kp;        /* keypad overlay */
static lv_obj_t *s_kp_val;    /* typed value */
static lv_obj_t *s_pc_overlay;
static char s_kp_buf[10];
static int s_kp_len;

static uint8_t *s_wf_buf;
static int s_mode;
static int s_center_mhz;
static int s_span_mhz;
static int s_prev_y[PLOT_W];
static int s_prev_center = -1;

static ui_actions_t s_act;
static void apply_visibility(void);

void ui_set_actions(const ui_actions_t *a) { s_act = *a; }

/* ---------------- keypad ---------------- */

static void kp_refresh(void)
{
    if (s_kp_len == 0) {
        lv_label_set_text(s_kp_val, "---- MHz");
    } else {
        char b[16];
        snprintf(b, sizeof(b), "%s MHz", s_kp_buf);
        lv_label_set_text(s_kp_val, b);
    }
}

static void kp_close(void)
{
    lv_obj_add_flag(s_kp, LV_OBJ_FLAG_HIDDEN);
    s_prev_center = -1; /* full redraw when the chart reappears */
}

static void kp_open(void)
{
    s_kp_len = 0;
    s_kp_buf[0] = 0;
    kp_refresh();
    lv_obj_clear_flag(s_kp, LV_OBJ_FLAG_HIDDEN);
}

static void kp_key_cb(lv_event_t *e)
{
    char c = (char)(uintptr_t)lv_event_get_user_data(e);
    if (c == 'D') {            /* backspace */
        if (s_kp_len > 0) s_kp_buf[--s_kp_len] = 0;
    } else if (c == 'C') {      /* clear */
        s_kp_len = 0;
        s_kp_buf[0] = 0;
    } else if (s_kp_len < 7 && c >= '0' && c <= '9') {
        s_kp_buf[s_kp_len++] = c;
        s_kp_buf[s_kp_len] = 0;
    }
    kp_refresh();
}

static void kp_ok_cb(lv_event_t *e)
{
    (void)e;
    if (s_kp_len > 0) {
        long v = atol(s_kp_buf);
        if (v < 100) v = 100;
        if (v > 6000) v = 6000;
        if (s_act.set_freq) s_act.set_freq((unsigned)v);
    }
    kp_close();
}

static void kp_cancel_cb(lv_event_t *e) { (void)e; kp_close(); }

static void title_cb(lv_event_t *e) { (void)e; kp_open(); }

static void n_cb(lv_event_t *e) { (void)e; if (s_act.cycle_n) s_act.cycle_n(); }
static void g_cb(lv_event_t *e) { (void)e; if (s_act.toggle_gain) s_act.toggle_gain(); }
static void f_cb(lv_event_t *e) { (void)e; if (s_act.cycle_fps) s_act.cycle_fps(); }
static void mode_cb(lv_event_t *e) { (void)e; if (s_act.cycle_view) s_act.cycle_view(); }

static lv_obj_t *mk_btn(lv_obj_t *parent, const char *txt, int x, int y, int w, int h,
                        lv_event_cb_t cb, void *ud)
{
    lv_obj_t *b = lv_btn_create(parent);
    lv_obj_set_pos(b, x, y);
    lv_obj_set_size(b, w, h);
    lv_obj_set_style_bg_color(b, lv_color_hex(0x2a3b4d), 0);
    lv_obj_set_style_radius(b, 4, 0);
    lv_obj_set_style_pad_all(b, 0, 0);
    lv_obj_t *l = lv_label_create(b);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_14, 0);
    lv_label_set_text(l, txt);
    lv_obj_center(l);
    if (cb) lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, ud);
    return b;
}

static void keypad_init(lv_obj_t *scr)
{
    s_kp = lv_obj_create(scr);
    lv_obj_set_pos(s_kp, 0, PLOT_Y);
    lv_obj_set_size(s_kp, PLOT_W, PLOT_H);
    lv_obj_set_style_bg_color(s_kp, lv_color_hex(0x0a0f16), 0);
    lv_obj_set_style_border_color(s_kp, lv_color_hex(0x2a3b4d), 0);
    lv_obj_set_style_border_width(s_kp, 1, 0);
    lv_obj_set_style_radius(s_kp, 0, 0);
    lv_obj_set_style_pad_all(s_kp, 0, 0);
    lv_obj_clear_flag(s_kp, LV_OBJ_FLAG_SCROLLABLE);

    s_kp_val = lv_label_create(s_kp);
    lv_obj_set_style_text_color(s_kp_val, lv_color_hex(0x00e0ff), 0);
    lv_obj_set_style_text_font(s_kp_val, &lv_font_montserrat_14, 0);
    lv_obj_align(s_kp_val, LV_ALIGN_TOP_MID, 0, 4);

    const char *keys[4][3] = {
        {"1", "2", "3"},
        {"4", "5", "6"},
        {"7", "8", "9"},
        {"DEL", "0", "CLR"},
    };
    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 3; c++) {
            void *ud = (void *)(uintptr_t)(unsigned char)keys[r][c][0];
            if (r == 3 && c == 0) ud = (void *)(uintptr_t)'D';
            if (r == 3 && c == 2) ud = (void *)(uintptr_t)'C';
            mk_btn(s_kp, keys[r][c], 6 + c * 64, 30 + r * 34, 60, 30, kp_key_cb, ud);
        }
    }
    mk_btn(s_kp, "OK", 204, 30, 110, 64, kp_ok_cb, NULL);
    mk_btn(s_kp, "CANCEL", 204, 98, 110, 64, kp_cancel_cb, NULL);

    lv_obj_add_flag(s_kp, LV_OBJ_FLAG_HIDDEN);
}

bool ui_keypad_active(void) { return !lv_obj_has_flag(s_kp, LV_OBJ_FLAG_HIDDEN); }

/* ---------------- colormap / waterfall ---------------- */

static lv_color_t turbo(float t)
{
    if (t < 0) t = 0;
    if (t > 1) t = 1;
    float t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;
    float r = 0.13572138f + 4.61539260f * t - 42.66032258f * t2
            + 132.13108234f * t3 - 152.94239396f * t4 + 59.28637943f * t5;
    float g = 0.09140261f + 2.19418839f * t + 4.84296658f * t2
            - 14.18503333f * t3 + 4.27729857f * t4 + 2.82956604f * t5;
    float b = 0.10667330f + 12.64194608f * t - 60.58204836f * t2
            + 110.36276771f * t3 - 89.90310912f * t4 + 27.34824973f * t5;
    if (r < 0) r = 0;
    if (r > 1) r = 1;
    if (g < 0) g = 0;
    if (g > 1) g = 1;
    if (b < 0) b = 0;
    if (b > 1) b = 1;
    return lv_color_make((uint8_t)(r * 255), (uint8_t)(g * 255), (uint8_t)(b * 255));
}

static void waterfall_init(void)
{
    size_t size = LV_CANVAS_BUF_SIZE_INDEXED_8BIT(PLOT_W, PLOT_H);
    s_wf_buf = heap_caps_malloc(size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!s_wf_buf) {
        ESP_LOGE(TAG, "waterfall buffer alloc failed (%u bytes)", (unsigned)size);
        return;
    }
    s_canvas = lv_canvas_create(lv_scr_act());
    lv_obj_set_size(s_canvas, PLOT_W, PLOT_H);
    lv_obj_align(s_canvas, LV_ALIGN_TOP_MID, 0, PLOT_Y);
    lv_canvas_set_buffer(s_canvas, s_wf_buf, PLOT_W, PLOT_H, LV_IMG_CF_INDEXED_8BIT);
    lv_canvas_set_palette(s_canvas, 0, lv_color_black());
    for (int i = 1; i < 256; i++) {
        lv_canvas_set_palette(s_canvas, i, turbo((float)(i - 1) / 254.0f));
    }
    memset(s_wf_buf + 1024, 0, (size_t)PLOT_W * PLOT_H);
    lv_obj_add_flag(s_canvas, LV_OBJ_FLAG_HIDDEN);
}

/* ---------------- frequency grid ---------------- */

static void chart_grid_draw(lv_event_t *e)
{
    if (s_span_mhz <= 0 || s_center_mhz <= 0) return;
    lv_obj_t *obj = lv_event_get_target(e);
    lv_draw_ctx_t *ctx = lv_event_get_draw_ctx(e);
    lv_area_t c;
    lv_obj_get_coords(obj, &c);
    int x0 = c.x1 + 1, x1 = c.x2 - 1, y0 = c.y1 + 1, y1 = c.y2 - 1;
    int w = x1 - x0;
    int lo = s_center_mhz - s_span_mhz / 2;
    int hi = s_center_mhz + s_span_mhz / 2;
    int step = s_span_mhz / 8;
    if (step < 1) step = 1;

    lv_draw_line_dsc_t dsc;
    lv_draw_line_dsc_init(&dsc);
    dsc.color = lv_color_hex(0x203040);
    dsc.width = 1;
    dsc.opa = LV_OPA_COVER;
    int first = ((lo + step - 1) / step) * step;
    for (int f = first; f <= hi; f += step) {
        int x = x0 + (int)((long)(f - lo) * w / s_span_mhz);
        lv_point_t p1 = {x, y0};
        lv_point_t p2 = {x, y1};
        lv_draw_line(ctx, &dsc, &p1, &p2);
    }
}

static void ovw_range(int view, int *lo, int *hi)
{
    *lo = (view == 1) ? 2200 : 100;
    *hi = (view == 1) ? 2700 : 3000;
}

/* Marker line on the sweep chart at the selected frequency. */
void ui_set_pc_mode(bool on)
{
    s_pc_mode = on;
    if (!s_pc_overlay) return;
    if (on) lv_obj_clear_flag(s_pc_overlay, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_add_flag(s_pc_overlay, LV_OBJ_FLAG_HIDDEN);
    apply_visibility();
}

void ui_set_marker(int view, unsigned freq_mhz)
{
    s_marker_view = view;
    s_marker_mhz = freq_mhz;
    if (!s_marker_obj) return;
    int lo, hi;
    ovw_range(view, &lo, &hi);
    if (!freq_mhz) {
        lv_obj_add_flag(s_marker_obj, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    int x = (int)((long)((int)freq_mhz - lo) * PLOT_W / (hi - lo));
    if (x < 0) x = 0;
    if (x > PLOT_W - 3) x = PLOT_W - 3;
    lv_obj_set_pos(s_marker_obj, x, PLOT_Y);
    lv_obj_clear_flag(s_marker_obj, LV_OBJ_FLAG_HIDDEN);
}

/* ---------------- init ---------------- */
void ui_init(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x101820), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    s_title = lv_label_create(scr);
    lv_obj_set_style_text_color(s_title, lv_color_hex(0x00e0ff), 0);
    lv_obj_set_style_text_font(s_title, &lv_font_montserrat_14, 0);
    lv_label_set_text(s_title, "2412 MHz");
    lv_obj_align(s_title, LV_ALIGN_TOP_MID, 0, 1);
    lv_obj_add_flag(s_title, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_title, title_cb, LV_EVENT_CLICKED, NULL);

    s_chart = lv_chart_create(scr);
    lv_obj_set_size(s_chart, PLOT_W, PLOT_H);
    lv_obj_align(s_chart, LV_ALIGN_TOP_MID, 0, PLOT_Y);
    lv_obj_set_style_bg_color(s_chart, lv_color_hex(0x081018), 0);
    lv_obj_set_style_border_color(s_chart, lv_color_hex(0x2a3b4d), 0);
    lv_obj_set_style_border_width(s_chart, 1, 0);
    lv_obj_set_style_pad_all(s_chart, 0, 0);
    lv_obj_set_style_line_width(s_chart, 1, LV_PART_ITEMS);
    lv_obj_set_style_size(s_chart, 0, LV_PART_INDICATOR);
    lv_chart_set_type(s_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(s_chart, PLOT_W);
    lv_chart_set_range(s_chart, LV_CHART_AXIS_PRIMARY_Y, 0, 1000);
    lv_chart_set_div_line_count(s_chart, 8, 0);
    s_ser = lv_chart_add_series(s_chart, lv_color_hex(0x00ff66), LV_CHART_AXIS_PRIMARY_Y);
    lv_obj_add_event_cb(s_chart, chart_grid_draw, LV_EVENT_DRAW_POST, NULL);

    waterfall_init();

    /* Tappable status buttons. */
    lv_obj_t *b;
    b = mk_btn(scr, "N 512", 2, 190, 76, 26, n_cb, NULL);
    s_lbl_n = lv_obj_get_child(b, 0);
    b = mk_btn(scr, "AGC", 82, 190, 76, 26, g_cb, NULL);
    s_lbl_g = lv_obj_get_child(b, 0);
    b = mk_btn(scr, "10fps", 162, 190, 76, 26, f_cb, NULL);
    s_lbl_f = lv_obj_get_child(b, 0);
    b = mk_btn(scr, "40mhz", 242, 190, 76, 26, mode_cb, NULL);
    s_lbl_mode = lv_obj_get_child(b, 0);
    lv_obj_set_style_text_font(s_lbl_n, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_font(s_lbl_g, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_font(s_lbl_f, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_font(s_lbl_mode, &lv_font_montserrat_10, 0);

    /* Sweep overview chart (fixed absolute-frequency axis). */
    s_ovw_chart = lv_chart_create(scr);
    lv_obj_set_size(s_ovw_chart, PLOT_W, PLOT_H);
    lv_obj_align(s_ovw_chart, LV_ALIGN_TOP_MID, 0, PLOT_Y);
    lv_obj_set_style_bg_color(s_ovw_chart, lv_color_hex(0x081018), 0);
    lv_obj_set_style_border_color(s_ovw_chart, lv_color_hex(0x2a3b4d), 0);
    lv_obj_set_style_border_width(s_ovw_chart, 1, 0);
    lv_obj_set_style_pad_all(s_ovw_chart, 0, 0);
    lv_obj_set_style_line_width(s_ovw_chart, 1, LV_PART_ITEMS);
    lv_obj_set_style_size(s_ovw_chart, 0, LV_PART_INDICATOR);
    lv_chart_set_type(s_ovw_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(s_ovw_chart, PLOT_W);
    lv_chart_set_range(s_ovw_chart, LV_CHART_AXIS_PRIMARY_Y, 0, 1000);
    lv_chart_set_div_line_count(s_ovw_chart, 8, 6);
    s_ovw_ser = lv_chart_add_series(s_ovw_chart, lv_color_hex(0xff9040), LV_CHART_AXIS_PRIMARY_Y);
    lv_obj_add_flag(s_ovw_chart, LV_OBJ_FLAG_HIDDEN);

    /* Marker line for the tapped frequency: black bar with a white core so it
     * is visible on the turbo waterfall as well as the line chart. */
    s_marker_obj = lv_obj_create(scr);
    lv_obj_set_size(s_marker_obj, 3, PLOT_H);
    lv_obj_set_pos(s_marker_obj, 0, PLOT_Y);
    lv_obj_set_style_bg_color(s_marker_obj, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(s_marker_obj, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(s_marker_obj, 0, 0);
    lv_obj_set_style_radius(s_marker_obj, 0, 0);
    lv_obj_set_style_pad_all(s_marker_obj, 0, 0);
    lv_obj_clear_flag(s_marker_obj, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_marker_obj, LV_OBJ_FLAG_HIDDEN);

    s_marker_inner = lv_obj_create(s_marker_obj);
    lv_obj_set_size(s_marker_inner, 1, PLOT_H);
    lv_obj_set_pos(s_marker_inner, 1, 0);
    lv_obj_set_style_bg_color(s_marker_inner, lv_color_hex(0xffffff), 0);
    lv_obj_set_style_bg_opa(s_marker_inner, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(s_marker_inner, 0, 0);
    lv_obj_set_style_radius(s_marker_inner, 0, 0);
    lv_obj_set_style_pad_all(s_marker_inner, 0, 0);
    lv_obj_clear_flag(s_marker_inner, LV_OBJ_FLAG_SCROLLABLE);

    s_ax_left = lv_label_create(scr);
    lv_obj_set_style_text_color(s_ax_left, lv_color_hex(0x8090a0), 0);
    lv_obj_set_style_text_font(s_ax_left, &lv_font_montserrat_10, 0);
    lv_obj_align(s_ax_left, LV_ALIGN_TOP_LEFT, 2, 220);

    s_ax_mid = lv_label_create(scr);
    lv_obj_set_style_text_color(s_ax_mid, lv_color_hex(0x8090a0), 0);
    lv_obj_set_style_text_font(s_ax_mid, &lv_font_montserrat_10, 0);
    lv_label_set_text(s_ax_mid, "DC");
    lv_obj_align(s_ax_mid, LV_ALIGN_TOP_MID, 0, 220);

    s_ax_right = lv_label_create(scr);
    lv_obj_set_style_text_color(s_ax_right, lv_color_hex(0x8090a0), 0);
    lv_obj_set_style_text_font(s_ax_right, &lv_font_montserrat_10, 0);
    lv_obj_align(s_ax_right, LV_ALIGN_TOP_RIGHT, -2, 220);

    keypad_init(scr);

    /* Opaque, centered PC-mode notice; normal UI is hidden while the host owns
     * the radio so LVGL/SPI cannot slow down the serial client. */
    s_pc_overlay = lv_obj_create(scr);
    lv_obj_set_size(s_pc_overlay, ST7789_W, ST7789_H);
    lv_obj_set_pos(s_pc_overlay, 0, 0);
    lv_obj_set_style_bg_color(s_pc_overlay, lv_color_hex(0x101820), 0);
    lv_obj_set_style_bg_opa(s_pc_overlay, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(s_pc_overlay, 0, 0);
    lv_obj_set_style_radius(s_pc_overlay, 0, 0);
    lv_obj_clear_flag(s_pc_overlay, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t *pc_label = lv_label_create(s_pc_overlay);
    lv_obj_set_style_text_color(pc_label, lv_color_hex(0x00e0ff), 0);
    lv_obj_set_style_text_font(pc_label, &lv_font_montserrat_14, 0);
    lv_label_set_text(pc_label, "PC MODE\nUSB Serial-JTAG\nPress BOOT to exit");
    lv_obj_center(pc_label);
    lv_obj_add_flag(s_pc_overlay, LV_OBJ_FLAG_HIDDEN);

    ui_set_mode(0);
    ui_set_view(0);
}

int ui_get_mode(void) { return s_mode; }
int ui_get_view(void) { return s_view; }

static void apply_visibility(void)
{
    if (s_pc_mode) {
        lv_obj_add_flag(s_title, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_lbl_n, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_lbl_g, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_lbl_f, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_lbl_mode, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_ax_left, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_ax_mid, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_ax_right, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_kp, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_chart, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_canvas, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_ovw_chart, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_marker_obj, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    lv_obj_clear_flag(s_title, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(s_lbl_n, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(s_lbl_g, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(s_lbl_f, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(s_lbl_mode, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(s_ax_left, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(s_ax_mid, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(s_ax_right, LV_OBJ_FLAG_HIDDEN);

    bool wf = (s_mode == 1);                       /* waterfall in any view */
    bool show_chart = (!wf && s_view == 0);        /* spectrum line chart  */
    bool show_ovw = (!wf && s_view != 0);          /* sweep line chart     */
    if (s_chart) {
        if (show_chart) lv_obj_clear_flag(s_chart, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(s_chart, LV_OBJ_FLAG_HIDDEN);
    }
    if (s_ovw_chart) {
        if (show_ovw) lv_obj_clear_flag(s_ovw_chart, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(s_ovw_chart, LV_OBJ_FLAG_HIDDEN);
    }
    if (s_canvas) {
        if (wf) lv_obj_clear_flag(s_canvas, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(s_canvas, LV_OBJ_FLAG_HIDDEN);
    }
    s_prev_center = -1;
}

void ui_set_mode(int mode)
{
    s_mode = mode;
    if (mode == 1 && s_wf_buf) {
        memset(s_wf_buf + 1024, 0, (size_t)PLOT_W * PLOT_H);
        lv_obj_invalidate(s_canvas);
    }
    apply_visibility();
}

void ui_set_view(int view)
{
    s_view = view;
    const char *nm = view == 0 ? "40mhz" : view == 1 ? "ism" : "overview";
    lv_label_set_text(s_lbl_mode, nm);
    apply_visibility();
    if (view != 0) {
        int lo = view == 1 ? 2200 : 100;
        int hi = view == 1 ? 2700 : 3000;
        char b[20];
        snprintf(b, sizeof(b), "%d", lo);
        lv_label_set_text(s_ax_left, b);
        snprintf(b, sizeof(b), "%d", (lo + hi) / 2);
        lv_label_set_text(s_ax_mid, b);
        snprintf(b, sizeof(b), "%d", hi);
        lv_label_set_text(s_ax_right, b);
    }
}

void ui_overview_update(const float *db, int n)
{
    if (s_view == 0 || !s_ovw_chart) return;
    for (int j = 0; j < PLOT_W && j < n; j++) {
        float v = db[j];
        if (v < DB_MIN) v = DB_MIN;
        if (v > DB_MAX) v = DB_MAX;
        int y = (int)((v - DB_MIN) * (1000.0f / (DB_MAX - DB_MIN)));
        s_ovw_ser->y_points[j] = (lv_coord_t)y;
    }
    lv_chart_refresh(s_ovw_chart);
}

static int bin_for_column(int x, int n)
{
    int off = (x - PLOT_W / 2) * n / PLOT_W;
    return off >= 0 ? off : n + off;
}

void ui_update(const float *db, int n, unsigned freq_mhz, int sample_rate_msps,
               int gain, int fps)
{
    s_center_mhz = (int)freq_mhz;
    s_span_mhz = sample_rate_msps;

    char buf[24];
    if (s_pc_mode) {
        lv_label_set_text(s_title, "PC MODE");
    } else {
        snprintf(buf, sizeof(buf), "%u MHz", freq_mhz);
        lv_label_set_text(s_title, buf);
    }

    snprintf(buf, sizeof(buf), "N:%d", n);
    lv_label_set_text(s_lbl_n, buf);
    if (gain < 0) {
        lv_label_set_text(s_lbl_g, "G:AGC");
    } else {
        snprintf(buf, sizeof(buf), "G:%d", gain);
        lv_label_set_text(s_lbl_g, buf);
    }
    snprintf(buf, sizeof(buf), "FPS:%d", fps);
    lv_label_set_text(s_lbl_f, buf);

    if (s_view == 0) {
        int span = sample_rate_msps / 2;
        int lo = (int)freq_mhz - span;
        int hi = (int)freq_mhz + span;
        if (lo < 0) lo = 0;
        char l[24];
        snprintf(l, sizeof(l), "%d", lo);
        lv_label_set_text(s_ax_left, l);
        snprintf(l, sizeof(l), "%u", freq_mhz);
        lv_label_set_text(s_ax_mid, l);
        snprintf(l, sizeof(l), "%d", hi);
        lv_label_set_text(s_ax_right, l);
    }

    if (s_view == 0 && s_mode == 0 && s_chart) {
        int ymin = 1 << 20, ymax = -(1 << 20), xmin = PLOT_W, xmax = -1;
        for (int x = 0; x < PLOT_W; x++) {
            float v = db[bin_for_column(x, n)];
            int y = (int)((v - DB_MIN) * (1000.0f / (DB_MAX - DB_MIN)));
            if (y < 0) y = 0;
            if (y > 1000) y = 1000;
            s_ser->y_points[x] = (lv_coord_t)y;
            if (y != s_prev_y[x]) {
                if (x < xmin) xmin = x;
                if (x > xmax) xmax = x;
                int lo2 = y < s_prev_y[x] ? y : s_prev_y[x];
                int hi2 = y > s_prev_y[x] ? y : s_prev_y[x];
                if (lo2 < ymin) ymin = lo2;
                if (hi2 > ymax) ymax = hi2;
            }
            s_prev_y[x] = y;
        }
        if ((int)freq_mhz != s_prev_center) {
            lv_chart_refresh(s_chart);
            s_prev_center = (int)freq_mhz;
        } else if (xmax >= 0) {
            lv_area_t c;
            lv_obj_get_coords(s_chart, &c);
            int top = c.y1 + 1, bot = c.y2 - 1, H = bot - top;
            if (H < 1) H = 1;
            int py0 = bot - (long)ymax * H / 1000;
            int py1 = bot - (long)ymin * H / 1000;
            lv_area_t area;
            area.x1 = c.x1 + xmin - 1;
            area.x2 = c.x1 + xmax + 1;
            area.y1 = py0 - 3;
            area.y2 = py1 + 3;
            if (area.x1 < c.x1) area.x1 = c.x1;
            if (area.x2 > c.x2) area.x2 = c.x2;
            if (area.y1 < c.y1) area.y1 = c.y1;
            if (area.y2 > c.y2) area.y2 = c.y2;
            lv_obj_invalidate_area(s_chart, &area);
        }
    }
}

void ui_waterfall_push(const float *db, int n, unsigned freq_mhz, bool centered)
{
    (void)freq_mhz;
    if (s_mode != 1 || !s_canvas || !s_wf_buf) return;
    uint8_t *idx = s_wf_buf + 1024;
    memmove(idx + PLOT_W, idx, (size_t)PLOT_W * (PLOT_H - 1));
    for (int x = 0; x < PLOT_W; x++) {
        int b = centered ? bin_for_column(x, n) : (x < n ? x : n - 1);
        float v = db[b];
        int p = (int)((v - WF_DB_LO) * (255.0f / (WF_DB_HI - WF_DB_LO)));
        if (p < 0) p = 0;
        if (p > 255) p = 255;
        idx[x] = (uint8_t)p;
    }
    lv_obj_invalidate(s_canvas);
}
