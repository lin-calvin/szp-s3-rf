#pragma once
#include <stdbool.h>

/* FT6336U capacitive touch on the shared I2C bus (addr 0x38).
 * Call after st7789_init() (which installs the I2C driver). */
void touch_init(void);

/* Read one point. Returns true while a finger is down, with screen
 * coordinates in the 320x240 landscape frame. */
bool touch_read(int *x, int *y);
