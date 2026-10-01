#include "touch.h"

#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#define FT6336U_ADDR       0x38
#define FT6336U_TD_STATUS  0x02
#define FT6336U_TOUCH1     0x03

static const char *TAG = "touch";

/* Raw panel is 240x320. The display uses swap_xy + mirror(true,false), so the
 * touch is transformed the same way (mirrors the vendor BSP). */
#define TOUCH_SWAP_XY  1
#define TOUCH_MIRROR_X 1
#define TOUCH_MIRROR_Y 0

static bool read_regs(uint8_t reg, uint8_t *buf, size_t n)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (FT6336U_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (FT6336U_ADDR << 1) | I2C_MASTER_READ, true);
    if (n > 1) {
        i2c_master_read(cmd, buf, n - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, buf + n - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t r = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return r == ESP_OK;
}

void touch_init(void)
{
    uint8_t id = 0;
    if (read_regs(0xA8, &id, 1)) {
        ESP_LOGI(TAG, "FT6336U focaltech id = 0x%02x", id);
    } else {
        ESP_LOGW(TAG, "FT6336U not responding");
    }
}

bool touch_read(int *x, int *y)
{
    uint8_t st = 0;
    if (!read_regs(FT6336U_TD_STATUS, &st, 1)) return false;
    int points = st & 0x0f;
    if (points == 0) return false;

    uint8_t d[4];
    if (!read_regs(FT6336U_TOUCH1, d, 4)) return false;
    int rx = ((d[0] & 0x0f) << 8) | d[1];
    int ry = ((d[2] & 0x0f) << 8) | d[3];

    int a = rx, b = ry;
#if TOUCH_SWAP_XY
    int t = a; a = b; b = t;
#endif
#if TOUCH_MIRROR_X
    a = 319 - a;
#endif
#if TOUCH_MIRROR_Y
    b = 239 - b;
#endif
    if (a < 0) a = 0;
    if (a > 319) a = 319;
    if (b < 0) b = 0;
    if (b > 239) b = 239;
    *x = a;
    *y = b;
    return true;
}
