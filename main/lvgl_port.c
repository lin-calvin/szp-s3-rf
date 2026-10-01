#include "lvgl_port.h"

#include "esp_heap_caps.h"
#include "lvgl.h"
#include "st7789.h"
#include "touch.h"

/* Partial draw buffer: N screen rows. The flush is synchronous (esp_lcd SPI
 * uses polling transmit when no done-callback is registered). */
#define DRAW_BUF_LINES 60

static lv_disp_draw_buf_t s_draw_buf;
static lv_color_t *s_buf;

static void disp_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *px)
{
    st7789_flush(area->x1, area->y1, area->x2, area->y2, px);
    lv_disp_flush_ready(drv);
}

void lvgl_port_init(void)
{
    lv_init();
    s_buf = heap_caps_malloc(ST7789_W * DRAW_BUF_LINES * sizeof(lv_color_t),
                             MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    lv_disp_draw_buf_init(&s_draw_buf, s_buf, NULL, ST7789_W * DRAW_BUF_LINES);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = ST7789_W;
    disp_drv.ver_res = ST7789_H;
    disp_drv.flush_cb = disp_flush;
    disp_drv.draw_buf = &s_draw_buf;
    lv_disp_drv_register(&disp_drv);
}

/* The FT6336U axes are 180 deg from the display; flip for LVGL coordinates. */
static void touch_read_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    (void)drv;
    int x, y;
    if (touch_read(&x, &y)) {
        data->point.x = 319 - x;
        data->point.y = 239 - y;
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

void lvgl_port_add_touch(void)
{
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touch_read_cb;
    lv_indev_drv_register(&indev_drv);
}
