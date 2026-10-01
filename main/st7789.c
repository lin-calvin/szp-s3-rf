#include "st7789.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

/* Matches the vendor BSP (maker-community/lcsc-shizhanpi-esp32s3-examples). */
#define LCD_HOST      SPI3_HOST
#define PIN_LCD_MOSI  40
#define PIN_LCD_SCLK  41
#define PIN_LCD_DC    39
#define PIN_LCD_BL    42

/* Backlight is active LOW on this board (LEDC inverted in the vendor BSP). */
#define BL_ON_LEVEL 0
#define BL_OFF_LEVEL 1

/* LCD chip-select is on PCA9557 IO0. */
#define PCA9557_ADDR   0x19
#define PCA_REG_INPUT  0x00
#define PCA_REG_OUTPUT 0x01
#define PCA_REG_CONFIG 0x03

static const char *TAG = "st7789";
static esp_lcd_panel_io_handle_t s_io;
static esp_lcd_panel_handle_t s_panel;
static SemaphoreHandle_t s_flush_sem;

/* esp_lcd SPI color transfers are queued asynchronously; without a done
 * callback it returns before DMA has read the buffer. Signal completion so
 * the caller can wait and LVGL never reuses a buffer still being read. */
static bool IRAM_ATTR flush_done_cb(esp_lcd_panel_io_handle_t io,
                                    esp_lcd_panel_io_event_data_t *edata,
                                    void *user_ctx)
{
    BaseType_t hpw = pdFALSE;
    xSemaphoreGiveFromISR(s_flush_sem, &hpw);
    return hpw == pdTRUE;
}

static void draw_bitmap_sync(int x1, int y1, int x2, int y2, const void *data)
{
    esp_lcd_panel_draw_bitmap(s_panel, x1, y1, x2, y2, data);
    xSemaphoreTake(s_flush_sem, pdMS_TO_TICKS(1000));
}

/* ---- PCA9557 (LCD CS / camera PWDN / audio PA) ---- */

static esp_err_t pca_write_reg(uint8_t reg, uint8_t val)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCA9557_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, val, true);
    i2c_master_stop(cmd);
    esp_err_t r = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return r;
}

static esp_err_t pca_read_reg(uint8_t reg, uint8_t *val)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCA9557_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCA9557_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, val, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t r = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return r;
}

static void lcd_cs(int level)
{
    uint8_t out = 0;
    if (pca_read_reg(PCA_REG_OUTPUT, &out) != ESP_OK) return;
    if (level) out |= 0x01; else out &= (uint8_t)~0x01;
    pca_write_reg(PCA_REG_OUTPUT, out);
}

static void pca9557_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 1,
        .scl_io_num = 2,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

    uint8_t cfg = 0, out = 0;
    pca_read_reg(PCA_REG_CONFIG, &cfg);
    pca_read_reg(PCA_REG_OUTPUT, &out);
    ESP_LOGI(TAG, "pca9557 before cfg=%02x out=%02x", cfg, out);

    /* IO0..IO2 outputs (0xf8), IO0=CS deasserted (1), IO1=PA off, IO2=cam off. */
    pca_write_reg(PCA_REG_CONFIG, 0xF8);
    pca_write_reg(PCA_REG_OUTPUT, 0x05);

    uint8_t cfg2 = 0, out2 = 0;
    pca_read_reg(PCA_REG_CONFIG, &cfg2);
    pca_read_reg(PCA_REG_OUTPUT, &out2);
    ESP_LOGI(TAG, "pca9557 after cfg=%02x out=%02x", cfg2, out2);
}

/* ---- Panel ---- */

void st7789_init(void)
{
    gpio_config_t bl = {
        .pin_bit_mask = 1ULL << PIN_LCD_BL,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&bl);
    gpio_set_level(PIN_LCD_BL, BL_OFF_LEVEL);

    pca9557_init();

    s_flush_sem = xSemaphoreCreateBinary();

    spi_bus_config_t bus = {
        .mosi_io_num = PIN_LCD_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = PIN_LCD_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = ST7789_W * ST7789_H * 2,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &bus, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_spi_config_t io_cfg = {
        .cs_gpio_num = -1,
        .dc_gpio_num = PIN_LCD_DC,
        .spi_mode = 2,
        .pclk_hz = 80 * 1000 * 1000,
        .trans_queue_depth = 10,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .on_color_trans_done = flush_done_cb,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_cfg, &s_io));

    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = -1,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(s_io, &panel_cfg, &s_panel));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel));   /* software reset */
    lcd_cs(0);                                       /* assert LCD CS */
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(s_panel, true));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(s_panel, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(s_panel, true, false));

    st7789_fill(0x0000);
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel, true));
    gpio_set_level(PIN_LCD_BL, BL_ON_LEVEL);
    ESP_LOGI(TAG, "panel ready (%dx%d)", ST7789_W, ST7789_H);
}

void st7789_backlight(int percent)
{
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    gpio_set_level(PIN_LCD_BL, percent > 0 ? BL_ON_LEVEL : BL_OFF_LEVEL);
}

void st7789_flush(int x1, int y1, int x2, int y2, const void *px)
{
    draw_bitmap_sync(x1, y1, x2 + 1, y2 + 1, px);
}

static void fill_rect_cb(int x, int y, int w, int h, uint16_t color)
{
    static uint16_t line[ST7789_W];
    if (w > ST7789_W) w = ST7789_W;
    for (int i = 0; i < w; i++) line[i] = color;
    for (int r = 0; r < h; r++) {
        draw_bitmap_sync(x, y + r, x + w, y + r + 1, line);
    }
}

void st7789_fill_rect(int x, int y, int w, int h, uint16_t color)
{
    fill_rect_cb(x, y, w, h, color);
}

void st7789_fill(uint16_t color)
{
    fill_rect_cb(0, 0, ST7789_W, ST7789_H, color);
}

void st7789_test_pattern(void)
{
    st7789_fill_rect(0, 0, ST7789_W, 106, 0xF800);
    st7789_fill_rect(0, 106, ST7789_W, 107, 0x07E0);
    st7789_fill_rect(0, 213, ST7789_W, 107, 0x001F);
    st7789_fill_rect(0, 0, ST7789_W, 6, 0xFFFF);
}

void st7789_debug_log(void)
{
    uint8_t cfg = 0, out = 0, in = 0;
    pca_read_reg(PCA_REG_CONFIG, &cfg);
    pca_read_reg(PCA_REG_OUTPUT, &out);
    pca_read_reg(PCA_REG_INPUT, &in);
    ESP_LOGI(TAG, "pca cfg=%02x out=%02x in=%02x bl=%d", cfg, out, in,
             gpio_get_level(PIN_LCD_BL));
}
