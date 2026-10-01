idf_component_register(
    SRCS "main.c" "fft.c" "st7789.c" "lvgl_port.c" "ui.c" "touch.c"
         "esp_sdr/receiver.c" "esp_sdr/burst_serial.c"
    INCLUDE_DIRS "."
    PRIV_INCLUDE_DIRS "esp_sdr"
    REQUIRES esp_phy esp_wifi nvs_flash esp_timer driver lvgl heap esp_lcd
)
