#pragma once
#include <stdint.h>

/* ST7789 240x320 panel on the 实战派 ESP32-S3 board.
 * SPI: MOSI=40 SCLK=41 DC=39, no CS (tied low on board), backlight GPIO42. */

void st7789_init(void);
void st7789_backlight(int percent);          /* 0..100, LEDC PWM */
void st7789_fill(uint16_t color);
/* Row-major RGB565 blit into the panel window at (x,y), w*h pixels. */
void st7789_blit(int x, int y, int w, int h, const uint16_t *px);

/* LVGL flush: inclusive area (x1,y1)-(x2,y2), row-major RGB565. */
void st7789_flush(int x1, int y1, int x2, int y2, const void *px);

/* Solid colour rectangle and a bring-up test pattern. */
void st7789_fill_rect(int x, int y, int w, int h, uint16_t color);
void st7789_test_pattern(void);

/* Bring-up diagnostics: log PCA9557 registers and backlight level. */
void st7789_debug_log(void);

/* Landscape logical size (panel GRAM is 240x320, rotated 90 degrees). */
#define ST7789_W 320
#define ST7789_H 240
