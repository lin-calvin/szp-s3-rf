#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Bring up the ESP32-S3 RF dump backend and the burst serial transport. */
void s3_rf_init(void);

/* Non-blocking burst-protocol client poll; keeps gr-esp32 / web clients working. */
void s3_cli_poll(void);

/* True while a host holds an active burst-protocol lease. */
bool s3_host_active(void);

/* One raw RF snapshot: n <= 16380 packed I/Q words at rate index 0/1/6
 * (80/40/16 MS/s). Returns false on timeout/garbage. */
bool s3_rf_capture(unsigned n, unsigned divider);

/* Words captured by the last successful s3_rf_capture: bits 0..9 signed I,
 * bits 10..19 signed Q. */
const uint32_t *s3_iq_words(void);

unsigned s3_frequency_mhz(void);
void s3_set_frequency_mhz(unsigned mhz);

/* Light retune for sweeps: only moves the PLL/channel, keeping the RX path
 * set up by the last full prepare_rx(). Much faster than s3_set_frequency_mhz. */
void s3_retune_mhz(unsigned mhz);

/* Approximate analog receive bandwidth in MHz; 0 selects the widest setting. */
void s3_set_bandwidth_mhz(unsigned mhz);

/* PHY gain-table control. Current gain is -1 for hardware AGC, else index. */
unsigned s3_gain_max(void);
int s3_gain_current(void);
void s3_gain_step(int delta);   /* switch to manual gain and adjust by delta */
void s3_gain_hardware(void);    /* restore AGC */
void s3_gain_toggle(void);      /* toggle AGC <-> manual */
