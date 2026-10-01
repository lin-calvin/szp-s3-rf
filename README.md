# szp-s3-rf — 实战派 ESP32-S3 片上 FFT 频谱仪

把 ESP32-S3 内置 Wi-Fi 射频的原始 I/Q 采样抓下来，**在片上做 FFT**，并把实时频谱画到板载 320×240 LCD 上；触摸手势可直接调谐。基于
[ESPARGOS/esp-sdr](https://github.com/ESPARGOS/esp-sdr) 的 ESP32-S3 接收后端。

> 不需要任何外置 SDR 硬件。esp-sdr 发现了一条绕过固定 Wi-Fi modem 的调试通路，可以直接把 ADC 的 I/Q 采样 dump 到 CPU。

## 硬件（立创·实战派 ESP32-S3 / ESP32S3R8N8）

| 部件 | 说明 |
| --- | --- |
| MCU | ESP32-S3，双核 LX7 @240MHz（本固件单核），8MB PSRAM（未启用），16MB flash |
| LCD | ST7789 240×320 SPI，`SPI3_HOST`，MOSI=GPIO40 SCLK=GPIO41 DC=GPIO39，无 RST（软件复位） |
| LCD CS | 不在 GPIO 上，接 **PCA9557 IO0**（I2C `0x19`） |
| 背光 | GPIO42，**低电平点亮**（官方 BSP 用 LEDC 反相 PWM） |
| 触摸 | FT6336U 电容屏，I2C `0x38`，SDA=GPIO1 SCL=GPIO2，无 RST/INT |
| PCA9557 | IO0=LCD CS，IO1=音频 PA 使能，IO2=摄像头 PWDN |

## 构建与烧录

用 PlatformIO 的 `espidf` 框架（自动下载 **ESP-IDF 5.2.1**）。LVGL 8.3.9 以 **git submodule** 形式放在
`components/lvgl`，记得带 submodule 克隆（或事后 `git submodule update --init`）。

因为系统 python 升到 3.14 后 `~/.platformio/penv` 已损坏，这里用项目内的 uv 环境：

```sh
git clone --recurse-submodules https://github.com/lin-calvin/szp-s3-rf.git
cd szp-s3-rf
uv venv --python /usr/bin/python3.13 .venv
uv pip install --python .venv/bin/python platformio
.venv/bin/pio run -t upload        # 串口默认 /dev/ttyACM0
```

## 功能

- **片上 FFT**：512 点复数 FFT（radix-2 + Hann 窗 + dBFS），`main/fft.c`，无外部 DSP 依赖。
  每次采集先**减均值做 DC removal**，消除零中频的中央 DC 尖峰。
  （320 px 宽的显示用 512 点即可平滑；256 点会出现台阶。）
- **显示模式**（底部模式按钮循环 `40mhz → ism → overview`）：
  - **40mhz**：普通频谱/瀑布（40 MHz 跨度，中心为 DC；长按切频谱↔瀑布）；
    底部显示绝对频率刻度，纵向频率网格线随调谐水平滚动。
  - **ism**：2400–2484 MHz 扫描概览（40 MHz 窗口、20 MHz 步长、max-hold）。
  - **overview**：100–3000 MHz 全段扫描概览（40 MHz 窗口、30 MHz 步长、max-hold）。
  - 瀑布模式：全幅 320×168 时间-频谱图，turbo 配色，新帧顶部插入；用 LVGL 8 位索引 canvas（≈54 KB）。
- **触摸交互**（FT6336U 注册为 LVGL 指针 indev；注意触摸轴与显示相差 180°，在 `read_cb` 里翻转）：
  - 点**标题**（当前频率）→ 弹出**数字键盘**，输入中心频率后 OK
  - 底部状态条是真正的 `lv_btn`：点 `N` 循环 FFT 点数 128/256/512/1024；点 `AGC` 切 AGC/手动；点 `10fps` 循环采集档位 5/10/20/40；点 `40mhz/ism/overview` 切换显示模式
  - **图表区**手势（自定义，仅在图表区生效）：长按切频谱/瀑布、左右拖动调谐、上下拖动增益
- **最宽模拟带宽**：启动时 `s3_set_bandwidth_mhz(0)`。
- **保持 esp-sdr 协议**：USB Serial/JTAG 上的 burst CLI（`INFO`/`CAPS`/`CAP20`/`RXRUN`/`FREQ`/`GAIN`/`BANDWIDTH` …），gnuradio 客户端
  [`gr-esp32`](/home/calvin/exps/gr-esp32) 仍可直接使用；有主机串流时 FFT 循环自动暂停。

### 可调参数（`main/main.c`）

- `FFT_N`（默认 512）、`FFT_RATE_DIV`（0/1/6 = 80/40/16 MS/s）、`FFT_RATE_MSPS`
- `SZP_DISPLAY_TEST`（1 = 只跑红绿蓝彩条自检）、`SZP_RF_ENABLE`（0 = 只跑 LVGL，不启 RF）

## 关键实现点 / 踩过的坑

1. **IDF 版本差异**：esp-sdr 固定的是 IDF 5.5-dev，这里用 PIO 的 5.2.1。差异点：
   - `usb_serial_jtag_wait_tx_done()` 在 5.2 不存在 → 用 `vTaskDelay(100ms)` 代替；
   - 组件名：5.2 里 UART/USB-serial-JTAG 都在 `driver` 组件（5.3+ 才拆出 `esp_driver_uart` 等）；
   - `force_rx_gain` 由 `esp_phy` 的 `librftest.a` 提供（需 `CONFIG_ESP_PHY_ENABLE_CERT_TEST=y`）。
2. **显示驱动必须用官方 `esp_lcd_new_panel_st7789`**（不是手写寄存器序列）：
   `spi_mode = 2`、`reset_gpio_num = -1`、`rgb_ele_order = RGB`，然后
   `esp_lcd_panel_reset()` → `lcd_cs(0)` → `esp_lcd_panel_init()` →
   `invert_color(true)` → `swap_xy(true)` → `mirror(true, false)`。
   手写序列热启动能亮、**冷启动黑屏**；换成官方驱动后冷启动正常。
   参考官方 BSP：`maker-community/lcsc-shizhanpi-esp32s3-examples`（`esp32_s3_szp.c`）。
3. **esp_lcd SPI 的 `tx_color` 是异步排队**的（`spi_device_queue_trans` 后不等待）。若不注册回调，
   它会先于 DMA 读完就返回，LVGL 复用同一 draw buffer → **屏幕中间出现黑带/撕裂**。
   修法：注册 `on_color_trans_done` 回调，用二值信号量让 `st7789_flush` 真正同步（见 `main/st7789.c`）。
4. **RF dump SRAM 预留**：`soc_reserve_region`（`SOC_RESERVE_MEMORY_REGION`）把 `0x3fcd0000..0x3fce0000`
   这段 64KB 从堆里排除，避免 dump 覆盖堆数据。`receiver.c` 顶部已保留。
5. **背光低电平点亮**；LCD CS 在 PCA9557 IO0（`lcd_cs()`）。
6. **自定义网格**：关掉 `lv_chart` 的纵向 div 线，改用 `LV_EVENT_DRAW_POST` 回调按
   绝对频率手绘竖线（`chart_grid_draw`），随中心频率平移，实现"网格滚动"。
   （用 `DRAW_MAIN_BEGIN` 会被图表背景盖住。）
7. **采集与 UI 异步 + 双核**：RF 采集/FFT 在 core 0（与 WiFi 同核），UI 渲染在 core 1。
   否则采集/FFT 的内存访问会把 core 上 LVGL 的 32KB cache 冲掉，渲染从 ~12ms 掉到 ~25ms。
   拆开后 UI 明显更顺；采集空闲时 10fps（`CAPTURE_PERIOD_MS`），**触摸/拖动时拉满**，
   松手回到 10fps。频谱经互斥锁保护的
   共享缓冲交给 UI 任务；RF 操作用 `rf_lock` 串行化（采集 vs 触摸调谐/增益）。
8. **局部刷新要绕过 `lv_chart_set_value_by_id`**：它对每个点都调一次 `invalidate_point`，
   320 次等于把整块标脏，局部刷新失效。改为直写 `ser->y_points[x]` 再单次局部失效，
   `ui_update` 从 5.4ms → 0.96ms，纯 UI 帧率 23 → 66fps。

## 目录

```
main/
  main.c            双任务：capture_task(core0, RF+FFT) / ui_task(core1, 触摸+渲染)
  fft.c/.h          片上复数 FFT（Hann 窗 + dBFS）
  st7789.c/.h       ST7789 显示驱动（esp_lcd + 同步 flush）与 PCA9557/背光
  lvgl_port.c/.h    LVGL 8 显示移植（局部 draw buffer + flush）与触摸 indev
  ui.c/.h           频谱/瀑布界面、可点状态条（lv_btn）、频率数字键盘
  touch.c/.h        FT6336U 触摸读取与坐标变换
  esp_sdr/          vendored esp-sdr ESP32-S3 RF 后端（Kconfig 无关）
components/
  lvgl/             LVGL 8.3.9（IDF component）
  lv_conf.h         板子原版 LVGL 配置（tick 改为 esp_timer，SPI 反相等）
```

## 已知限制

- esp-sdr 是**突发快照**：单次最多 16380 个复数采样，采样之间有空隙；不是连续采样。
- 采样率名义 80/40/16 MS/s（本固件用 40），超过串口吞吐时靠快照拼接。
- 增益/功率未标定；模拟前端就是 Wi-Fi PHY；中心频率只能落在 ISM/扩展范围内才有意义。
- 8 位/10 位 I/Q 上限、SRAM 预留等约束沿用上游。

## 来源与许可

- RF 后端：ESPARGOS/esp-sdr（部分代码由 LLM 生成）。
- 显示初始化：立创官方例程 `maker-community/lcsc-shizhanpi-esp32s3-examples`。
- LVGL 8.3.9（MIT）。
