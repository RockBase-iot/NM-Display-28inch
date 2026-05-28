#if defined(NM_DISPLAY_28)

#include "config.h"
#include "../../drivers/devices/hal.h"
#include "../../drivers/display/spiport.h"
#include "../../drivers/touch/iicport.h"
#include <Wire.h>
#include <SPI.h>
#include "../../utils/logger.h"

// ─── AXP2101 power management ────────────────────────────────────────────────

#define AXP2101_I2C_ADDR            0x34

/* Output rail enable registers */
#define AXP2101_REG_DCDC_EN         0x80
#define AXP2101_REG_LDO_EN          0x90
#define AXP2101_REG_DLDO2_EN        0x91

/* Power-path / charger registers */
#define AXP2101_REG_VSYS_MIN        0x14   /* VSYS minimum DPM threshold (bits[6:4]: 000=4.1V … step 100mV) */
#define AXP2101_REG_VINDPM          0x15   /* VBUS input voltage DPM (bits[3:0]: 0x00=3.88V … step 80mV) */
#define AXP2101_REG_VBUSLIM         0x16   /* VBUS input current limit (bits[2:0]: 000=100mA … 100=1500mA) */
#define AXP2101_REG_DCDC_OVP_UVP    0x23   /* DC-DC OVP/UVP triggered PMIC shutdown enable */
#define AXP2101_REG_FAST_PWRON0     0x28   /* Fast power-on sequence config 0 (DC4/DC3/DC2/DC1) */
#define AXP2101_REG_FAST_PWRON1     0x29   /* Fast power-on sequence config 1 (ALDO3/ALDO2/ALDO1/DC5) */
#define AXP2101_REG_ITERM_CHG       0x63   /* Charger termination current (bits[3:0]: 0x8=200mA) */

/* Voltage output registers */
#define AXP2101_REG_DC2_VOL         0x83
#define AXP2101_REG_DC3_VOL         0x84
#define AXP2101_REG_DC4_VOL         0x85
#define AXP2101_REG_DC5_VOL         0x86
#define AXP2101_REG_ALDO1_VOL       0x92
#define AXP2101_REG_ALDO2_VOL       0x93
#define AXP2101_REG_ALDO3_VOL       0x94
#define AXP2101_REG_ALDO4_VOL       0x95
#define AXP2101_REG_BLDO1_VOL       0x96
#define AXP2101_REG_BLDO2_VOL       0x97
#define AXP2101_REG_CPUSLDO_VOL     0x98
#define AXP2101_REG_DLDO1_VOL       0x99
#define AXP2101_REG_DLDO2_VOL       0x9A

/* Voltage register values */
#define AXP2101_DC2_1000MV          0x32
#define AXP2101_DC3_3300MV          0x69
#define AXP2101_DC4_1000MV          0x32
#define AXP2101_DC5_3300MV          0x13
#define AXP2101_LDO_3300MV          0x1C
#define AXP2101_BLDO1_1500MV        0x0A
#define AXP2101_BLDO2_2800MV        0x17
#define AXP2101_CPUSLDO_1000MV      0x0A

/* Named register field values */
#define AXP2101_VSYS_MIN_4V1        (0b000 << 4)   /* 4.1 V */
#define AXP2101_VINDPM_3V88         0x00            /* 3.88 V — lowest setting */
#define AXP2101_VBUS_1500MA         0x04            /* 1500 mA */
#define AXP2101_DCDC_OVP_UVP_OFF    0x00            /* disable all DC-DC OVP/UVP shutdown triggers */
#define AXP2101_FAST_PWRON_ALL_DIS  0xFF            /* disable fast power-on for all rails */
#define AXP2101_ITERM_200MA         0x08            /* 200 mA termination (~C/10 for 2000 mAh) */

struct axp2101_reg_update_t {
    uint8_t     reg;
    uint8_t     mask;
    uint8_t     value;
    const char *name;
};

static bool _axp2101_read(uint8_t reg, uint8_t *val) {
    Wire.beginTransmission(AXP2101_I2C_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)AXP2101_I2C_ADDR, (uint8_t)1) != 1) return false;
    *val = Wire.read();
    return true;
}

static bool _axp2101_write(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(AXP2101_I2C_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

static bool _axp2101_update(uint8_t reg, uint8_t mask, uint8_t value) {
    uint8_t cur = 0;
    if (!_axp2101_read(reg, &cur)) return false;
    cur = (cur & ~mask) | (value & mask);
    return _axp2101_write(reg, cur);
}

static bool _axp2101_apply_updates(const axp2101_reg_update_t *updates, size_t count) {
    for (size_t i = 0; i < count; i++) {
        if (!_axp2101_update(updates[i].reg, updates[i].mask, updates[i].value)) {
            LOG_E("AXP2101 %s failed", updates[i].name);
            return false;
        }
    }
    return true;
}

// ─── ST7789 initialisation data ─────────────────────────────────────────────
// Data arrays must be in DRAM (DRAM_ATTR) when the SPI driver uses DMA.
// ST7789 init sequence based on the NMMiner esp32_2432s028r_7789 reference BSP.

DRAM_ATTR static uint8_t _st7789_colmod[]  = { 0x55 };               // 16-bit RGB565
DRAM_ATTR static uint8_t _st7789_dfunc[]   = { 0x0A, 0x82 };         // Display function control
DRAM_ATTR static uint8_t _st7789_ramctl[]  = { 0x00, 0xE0 };         // RAM control
DRAM_ATTR static uint8_t _st7789_porctrl[] = { 0x0C, 0x0C, 0x00, 0x33, 0x33 }; // Porch
DRAM_ATTR static uint8_t _st7789_gctrl[]   = { 0x35 };               // Gate control
DRAM_ATTR static uint8_t _st7789_vcoms[]   = { 0x28 };               // VCOMS
DRAM_ATTR static uint8_t _st7789_lcm[]     = { 0x0C };               // LCM control
DRAM_ATTR static uint8_t _st7789_vdvvr[]   = { 0x01, 0xFF };         // VDV/VRH enable
DRAM_ATTR static uint8_t _st7789_vrh[]     = { 0x10 };               // VRH set
DRAM_ATTR static uint8_t _st7789_vdv[]     = { 0x20 };               // VDV set
DRAM_ATTR static uint8_t _st7789_frctr2[]  = { 0x0F };               // 60 fps
DRAM_ATTR static uint8_t _st7789_pwrctl[]  = { 0xA4, 0xA1 };         // Power control
DRAM_ATTR static uint8_t _st7789_pvgam[]   = {
    0xD0, 0x00, 0x02, 0x07, 0x0A, 0x28, 0x32,
    0x44, 0x42, 0x06, 0x0E, 0x12, 0x14, 0x17 }; // Positive gamma
DRAM_ATTR static uint8_t _st7789_nvgam[]   = {
    0xD0, 0x00, 0x02, 0x07, 0x0A, 0x28, 0x31,
    0x54, 0x47, 0x0E, 0x1C, 0x17, 0x1B, 0x1E }; // Negative gamma
// CASET / RASET cover the full 320×240 logical window (MV=1 swaps axes)
DRAM_ATTR static uint8_t _st7789_caset[]   = {
    0x00, 0x00,
    (uint8_t)(((SCREEN_WIDTH  - 1) >> 8) & 0xFF),
    (uint8_t)( (SCREEN_WIDTH  - 1)       & 0xFF) };
DRAM_ATTR static uint8_t _st7789_raset[]   = {
    0x00, 0x00,
    (uint8_t)(((SCREEN_HEIGHT - 1) >> 8) & 0xFF),
    (uint8_t)( (SCREEN_HEIGHT - 1)       & 0xFF) };

static const lcd_init_cmd_t _st7789_init_cmds[] = {
    { 0x11, nullptr,        0,  120 },  // SLPOUT  — sleep out (120 ms)
    { 0x13, nullptr,        0,    0 },  // NORON   — normal display mode on
    { 0x21, nullptr,        0,    0 },  // INVON   — display inversion on (required by this panel)
    { 0x36, nullptr,        0,    0 },  // MADCTL  — placeholder; rotate() sends the real value
    { 0xB6, _st7789_dfunc,  2,    0 },  // DFUNCTR
    { 0xB0, _st7789_ramctl, 2,    0 },  // RAMCTRL
    { 0x3A, _st7789_colmod, 1,   10 },  // COLMOD  — 16-bit colour
    { 0xB2, _st7789_porctrl,5,    0 },  // PORCTRL
    { 0xB7, _st7789_gctrl,  1,    0 },  // GCTRL
    { 0xBB, _st7789_vcoms,  1,    0 },  // VCOMS
    { 0xC0, _st7789_lcm,    1,    0 },  // LCMCTRL
    { 0xC2, _st7789_vdvvr,  2,    0 },  // VDVVRHEN
    { 0xC3, _st7789_vrh,    1,    0 },  // VRHS
    { 0xC4, _st7789_vdv,    1,    0 },  // VDVSET
    { 0xC6, _st7789_frctr2, 1,    0 },  // FRCTR2
    { 0xD0, _st7789_pwrctl, 2,    0 },  // PWCTRL1
    { 0xE0, _st7789_pvgam, 14,    0 },  // PVGAMCTRL
    { 0xE1, _st7789_nvgam, 14,    0 },  // NVGAMCTRL
    { 0x2A, _st7789_caset,  4,    0 },  // CASET
    { 0x2B, _st7789_raset,  4,    0 },  // RASET
    { 0x29, nullptr,        0,  120 },  // DISPON  — display on (120 ms)
};

// ST7789 MADCTL values:
//   MY=0x80  MX=0x40  MV=0x20  BGR=0x08
//   0°  portrait:     BGR only  = 0x08
//   90° landscape:    MX+MV+BGR = 0x68
//   180° portrait:    MY+MX+BGR = 0xC8
//   270° landscape:   MY+MV+BGR = 0xA8   ← operating mode for this board
static const lcd_vendor_config_t _st7789_vendor_cfg = {
    .init_cmds      = _st7789_init_cmds,
    .init_cmds_size = sizeof(_st7789_init_cmds) / sizeof(lcd_init_cmd_t),
    .color_inverted = false,   // No software inversion
    .order_rgb      = true,    // No R/B channel swap (handled by BGR bit in MADCTL)
    .swap_bytes     = true,    // Swap bytes for big-endian SPI (LV_COLOR_16_SWAP=0)
    .reset_active_low = false, // RST handled via TCA9554, not a direct GPIO
    .reset_pulse_ms   = 10,
    .bl_active_high   = true,  // BL pin: higher duty = brighter
    .spi_mode         = SPI_MODE3, // ST7789 requires SPI MODE3 at high speed
    .rotation = {
        .madctl_cmd      = 0x36,
        .rotation_values = { 0x08, 0x68, 0xC8, 0xA8 }, // 0°, 90°, 180°, 270°
        .swap_dimensions = { true, false, true, false }, // portrait / landscape
        .colstart        = { 0, 0, 0, 0 },
        .rowstart        = { 0, 0, 0, 0 },
    },
};

// ─── Board implementation ────────────────────────────────────────────────────

class NMDisplay28Board : public Board {
private:
    BoardProfile _profile;
    SPIClass     _lcdspi { FSPI };   // SPI2 (FSPI) — MOSI=1, SCLK=5
    Display     *_display = nullptr;
    Touch       *_touch   = nullptr;

public:
    NMDisplay28Board() {
        _profile.screen_width  = SCREEN_WIDTH;
        _profile.screen_height = SCREEN_HEIGHT;
        _profile.has_touch     = true;
        _profile.has_button    = true;
        _profile.has_led       = false;
        _profile.has_gauge     = false;
        _profile.has_camera    = true;
        _profile.has_audio     = true;
        _profile.has_imu       = true;
        _profile.has_pmu       = true;
        _profile.has_rtc       = true;
        _profile.has_sdcard    = true;
    }

    void init() override {
        Serial.begin(115200);

        // Shared I2C bus: Touch (FT6336), PMU, IMU, RTC, Audio, IO-Expander (TCA9554)
        Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, (uint32_t)I2C_FREQ_HZ);

        // Reset LCD via TCA9554 IO1 before bringing up the SPI bus
        _tca9554_lcd_reset();

        // Start SPI2 for the ST7789 display
        _lcdspi.begin(LCD_SCLK_PIN, LCD_MISO_PIN, LCD_MOSI_PIN, -1);
    }

    void deinit()  override {}
    void restart() override { ESP.restart(); }

    // Lazy-init the ST7789 display.
    // On first call: creates SPIScreen, loads vendor config, sends init sequence,
    // rotates to 270° landscape, and leaves backlight off until the app is ready.
    Display* get_display() override {
        if (_display) return _display;

        auto *scr = new SPIScreen(
            &_lcdspi,
            LCD_DC_PIN, LCD_RST_PIN, LCD_CS_PIN, LCD_BL_PIN,
            SCREEN_WIDTH, SCREEN_HEIGHT,
            LCD_SPI_FREQ_HZ,
            LCD_BL_LEDC_CH);   // LEDC channel 1 (channel 0 reserved for camera XCLK)

        scr->load_vendor_config(_st7789_vendor_cfg);
        scr->reset();           // Hardware reset pulse (no-op here — RST is via TCA9554)
        scr->init();            // Send ST7789 init command table
        scr->rotate(DISPLAY_ROTATION); // Set 270° landscape via MADCTL
        scr->blctrl(0.0f);      // Backlight off; app turns it on after UI is ready

        _display = scr;
        return _display;
    }

    // Lazy-init the FT6336 capacitive touch controller.
    // Shares the Wire I2C bus already started in init().
    Touch* get_touch() override {
        if (_touch) return _touch;

        // FT6336 is on the shared Wire bus (already initialised in init()).
        // Pass sdaPin = -1 to skip redundant Wire.begin() inside IICTouch::init().
        auto *tp = new IICTouch(
            &Wire,
            FT6336_I2C_ADDR,
            -1, -1,             // SDA/SCL: -1 = bus already up
            -1, -1,             // IRQ/RST: not connected on this board
            // Pass the sensor's PORTRAIT native resolution (240×320) so that
            // the 1:1 scaling inside IICTouch gives raw portrait coords.
            // The rotation_map swap_xy below then converts to landscape.
            240, 320,
            I2C_FREQ_HZ);

        tp->load_config({
            .init_cmds      = nullptr,
            .init_cmds_size = 0,
            // FT6336 raw range in portrait orientation.
            .x_max = 240,
            .y_max = 320,
            // 90° landscape: the sensor's portrait X axis aligns with the
            // display's landscape Y axis, and portrait Y aligns with landscape X.
            // A simple swap_xy (no mirroring) produces correct LVGL coordinates.
            //   After swap: pt.x = raw_y (0-319 = LVGL x range ✓)
            //               pt.y = raw_x (0-239 = LVGL y range ✓)
            .rotation_map = {
                [0] = { false, false, false },  // 0°
                [1] = { true,  false, true  },  // 90°  swap_xy + mirror_y
                [2] = { false, false, false },  // 180°
                [3] = { false, false, false },  // 270°
            },
        });
        tp->reset();
        tp->init();
        tp->set_rotation(DISPLAY_ROTATION);  // apply transform for operating rotation

        _touch = tp;
        return _touch;
    }

    Button* get_boot_button() override { return nullptr; }
    Stream& get_uart()        override { return Serial; }
    String  get_board_model() override { return BSP_BOARD_MODEL; }
    float   get_mcu_temp()    override { return temperatureRead(); }

    const BoardProfile& get_profile() const override { return _profile; }

private:
    bool _axp2101_init() {
        /* Step 0: raise VBUS current limit BEFORE enabling any output rail.
         * Default is 500 mA; at full system load (WiFi + camera + display + audio)
         * demand exceeds this, causing VSYS to droop below UVLO and resetting the
         * board.  1500 mA removes this failure mode when running from USB. */
        static const axp2101_reg_update_t power_path_updates[] = {
            {AXP2101_REG_VBUSLIM,      0x07, AXP2101_VBUS_1500MA,         "VBUS limit 1500 mA"},
        };
        if (!_axp2101_apply_updates(power_path_updates, 1)) {
            LOG_E("AXP2101 power-path config failed");
            return false;
        }

        static const axp2101_reg_update_t voltage_updates[] = {
            /* Power-path / charger tuning */
            {AXP2101_REG_VSYS_MIN,     0x70, AXP2101_VSYS_MIN_4V1,        "vsys_min 4.1V"},
            {AXP2101_REG_FAST_PWRON0,  0xFF, AXP2101_FAST_PWRON_ALL_DIS,  "fast_pwron DC4/3/2/1 off"},
            {AXP2101_REG_FAST_PWRON1,  0xFF, AXP2101_FAST_PWRON_ALL_DIS,  "fast_pwron ALDO3/2/1/DC5 off"},
            {AXP2101_REG_ITERM_CHG,    0x0F, AXP2101_ITERM_200MA,         "iterm 200mA"},
            {AXP2101_REG_DCDC_OVP_UVP, 0xFF, AXP2101_DCDC_OVP_UVP_OFF,   "dcdc ovp/uvp off"},
            {AXP2101_REG_VINDPM,       0x0F, AXP2101_VINDPM_3V88,         "vindpm 3.88V"},
            /* Rail voltages */
            {AXP2101_REG_BLDO1_VOL,    0x1F, AXP2101_BLDO1_1500MV,        "BLDO1 1500 mV"},
            {AXP2101_REG_DC2_VOL,      0x7F, AXP2101_DC2_1000MV,          "DC2 1000 mV"},
            {AXP2101_REG_DC3_VOL,      0x7F, AXP2101_DC3_3300MV,          "DC3 3300 mV"},
            {AXP2101_REG_DC4_VOL,      0x7F, AXP2101_DC4_1000MV,          "DC4 1000 mV"},
            {AXP2101_REG_DC5_VOL,      0x1F, AXP2101_DC5_3300MV,          "DC5 3300 mV"},
            {AXP2101_REG_ALDO1_VOL,    0x1F, AXP2101_LDO_3300MV,          "ALDO1 3300 mV"},
            {AXP2101_REG_ALDO2_VOL,    0x1F, AXP2101_LDO_3300MV,          "ALDO2 3300 mV"},
            {AXP2101_REG_ALDO3_VOL,    0x1F, AXP2101_LDO_3300MV,          "ALDO3 3300 mV"},
            {AXP2101_REG_ALDO4_VOL,    0x1F, AXP2101_LDO_3300MV,          "ALDO4 3300 mV"},
            {AXP2101_REG_BLDO2_VOL,    0x1F, AXP2101_BLDO2_2800MV,        "BLDO2 2800 mV"},
            {AXP2101_REG_CPUSLDO_VOL,  0x1F, AXP2101_CPUSLDO_1000MV,      "CPUSLDO 1000 mV"},
            {AXP2101_REG_DLDO1_VOL,    0x1F, AXP2101_LDO_3300MV,          "DLDO1 3300 mV"},
            {AXP2101_REG_DLDO2_VOL,    0x1F, AXP2101_LDO_3300MV,          "DLDO2 3300 mV"},
        };
        static const axp2101_reg_update_t enable_updates[] = {
            {AXP2101_REG_DCDC_EN,  0x1E, 0x1E, "enable DC2-DC5"},
            {AXP2101_REG_LDO_EN,   0xFF, 0xFF, "enable ALDO/BLDO/CPUSLDO/DLDO1"},
            {AXP2101_REG_DLDO2_EN, 0x01, 0x01, "enable DLDO2"},
        };

        if (!_axp2101_apply_updates(voltage_updates, sizeof(voltage_updates) / sizeof(voltage_updates[0]))) {
            return false;
        }
        if (!_axp2101_apply_updates(enable_updates, sizeof(enable_updates) / sizeof(enable_updates[0]))) {
            return false;
        }
        LOG_I("AXP2101 power rails enabled");
        return true;
    }

    // Assert LCD reset via TCA9554 IO1: configure IO1 as output then pulse low.
    void _tca9554_lcd_reset() {
        // Config register 0x03: IO1 = output (bit1 = 0), all others = input.
        Wire.beginTransmission(TCA9554_I2C_ADDR);
        Wire.write(0x03);
        Wire.write(0xFD);   // 0b1111_1101
        Wire.endTransmission();

        _tca9554_set_io1(0);  delay(100);   // assert reset (LOW)
        _tca9554_set_io1(1);  delay(100);   // deassert reset (HIGH)
    }

    void _tca9554_set_io1(uint8_t level) {
        Wire.beginTransmission(TCA9554_I2C_ADDR);
        Wire.write(0x01);                       // Output port register
        Wire.write(level ? 0x02 : 0x00);        // bit1 drives IO1
        Wire.endTransmission();
    }
};

DECLARE_BOARD(NMDisplay28Board);

#endif // NM_DISPLAY_28
