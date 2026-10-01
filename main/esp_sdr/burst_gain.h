/* Shared snapshot RX gain control. Codes are PHY gain indices, not dB.
 * Hardware AGC controls gain unless a manual index is selected.
 * AGCPWR_CTRL7 bits 8..14 hold the maximum calibrated low-table index.
 * The pinned PHY implementations populate this field when tuning. */
extern void force_rx_gain(unsigned,unsigned,unsigned);
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C61 || CONFIG_IDF_TARGET_ESP32C6
#define BURST_GAIN_REG 0x600a702cu
#else
#define BURST_GAIN_REG 0x6001c02cu
#endif
typedef enum { GAIN_MANUAL, GAIN_HARDWARE } burst_gain_mode_t;
static burst_gain_mode_t gain_mode=GAIN_HARDWARE;
static unsigned gain_code=40;
static unsigned gain_max(void) {
    unsigned maximum=(REG_READ(BURST_GAIN_REG)>>8)&127u;
    #if CONFIG_IDF_TARGET_ESP32C5
    return maximum<90u ? maximum : 0u;
#elif CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32S2
    return maximum<=82u ? maximum : 0u;
#else
    return maximum<80u ? maximum : 0u;
#endif /* Never expose uncalibrated slots. */
}
#if CONFIG_IDF_TARGET_ESP32C61
extern void burst_gain_mirror(int index);
extern void phy_rfrx_sat_rst(unsigned);
static unsigned gain_init_saved, gain_threshold_saved;
static bool gain_defaults_saved;
#endif
static void gain_apply(void) {
    bool manual=gain_mode==GAIN_MANUAL;
    if(gain_code>gain_max())gain_code=gain_max();
#if CONFIG_IDF_TARGET_ESP32C61
    if(!gain_defaults_saved) {
        gain_init_saved=REG_READ(0x600a7094u);
        gain_threshold_saved=REG_READ(0x600a713cu);
        gain_defaults_saved=true;
    }
    burst_gain_mirror(manual?(int)gain_code:-1);
    phy_rfrx_sat_rst(manual?0u:1u);
    REG_WRITE(0x600a7094u,manual ? (gain_init_saved&~0x1fcu)|(gain_code<<2) : gain_init_saved);
    REG_WRITE(0x600a713cu,manual ? (gain_threshold_saved&~0x1fc0000u)|(gain_code<<18) : gain_threshold_saved);
#endif
    force_rx_gain(manual,gain_code,0);
}
static bool gain_command(const char *line) {
    unsigned code;char extra;
    if(!strcmp(line,"GAIN?")) {
        char h[64];snprintf(h,sizeof(h),"GAIN %s %d 0 %u %u\n",gain_mode==GAIN_HARDWARE?"HARDWARE":"MANUAL",gain_mode==GAIN_HARDWARE?-1:(int)gain_code,gain_max(),(unsigned)((REG_READ(BURST_GAIN_REG)>>23)&1));reply(h);return true;
    }
    if(!strcmp(line,"GAIN HARDWARE")) {gain_mode=GAIN_HARDWARE;}
    else if(sscanf(line,"GAIN MANUAL %u %c",&code,&extra)==1 && code<=gain_max()) {
        gain_mode=GAIN_MANUAL;gain_code=code;
    } else return false;
    gain_apply();reply("OK\n");return true;
}
