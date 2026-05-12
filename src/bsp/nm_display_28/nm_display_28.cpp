#if defined(NM_DISPLAY_28)

#include "config.h"
#include "../../drivers/devices/hal.h"
#include <Wire.h>

// NMDisplay28Board — concrete Board implementation for NMTech 2.8" display module.
// Phase 1: Serial + I2C bus init only.
// Phase 2+: SPI/LCD/Touch and further peripherals will be added incrementally.
class NMDisplay28Board : public Board {
private:
    BoardProfile _profile;

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

        // Shared I2C bus: Touch, PMU, IMU, RTC, Audio, IO-Expander
        Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, (uint32_t)I2C_FREQ_HZ);

        // Reset LCD via TCA9554 IO expander
        _tca9554_lcd_reset();

        // TODO Phase 2: SPI2 + ST7789 LCD (spiport.h)
        // TODO Phase 2: FT6336 touch driver (ft6336_port.h)
        // TODO Phase 2: BOOT button (onebutton_port.h)
    }

    void deinit()  override {}
    void restart() override { ESP.restart(); }

    // Stubs until Phase 2 — callers must check profile.has_xxx before use.
    Display* get_display()     override { return nullptr; }
    Touch*   get_touch()       override { return nullptr; }
    Button*  get_boot_button() override { return nullptr; }

    Stream&  get_uart()        override { return Serial; }
    String   get_board_model() override { return BSP_BOARD_MODEL; }
    float    get_mcu_temp()    override { return temperatureRead(); }

    const BoardProfile& get_profile() const override {
        return _profile;
    }

private:
    // Assert LCD reset via TCA9554 IO1: pull low then high.
    void _tca9554_lcd_reset() {
        // Configuration register 0x03: set IO1 as output (bit1 = 0), rest input.
        Wire.beginTransmission(TCA9554_I2C_ADDR);
        Wire.write(0x03);
        Wire.write(0xFD);   // 0b11111101
        Wire.endTransmission();

        _tca9554_set_io1(0);  delay(100);   // assert reset
        _tca9554_set_io1(1);  delay(100);   // deassert reset
    }

    void _tca9554_set_io1(uint8_t level) {
        Wire.beginTransmission(TCA9554_I2C_ADDR);
        Wire.write(0x01);                    // Output port register
        Wire.write(level ? 0x02 : 0x00);     // bit1 drives IO1
        Wire.endTransmission();
    }
};

DECLARE_BOARD(NMDisplay28Board);

#endif // NM_DISPLAY_28
