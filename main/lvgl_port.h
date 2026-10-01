#pragma once

/* LVGL 8 display port over the ST7789 esp_lcd panel IO. */
void lvgl_port_init(void);

/* Register the FT6336U as an LVGL pointer input device (call after
 * lvgl_port_init() and touch init). */
void lvgl_port_add_touch(void);
