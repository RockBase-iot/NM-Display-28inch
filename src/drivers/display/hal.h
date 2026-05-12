#pragma once

// Display Hardware Abstraction Layer
// Defines the abstract Display interface and vendor configuration structures
// used by concrete display drivers (SPIScreen, etc.).

// LCD initialization command descriptor
typedef struct {
    int         cmd;          // LCD command byte
    const void *data;         // Pointer to command parameter bytes (may be NULL)
    size_t      data_bytes;   // Length of data in bytes
    uint16_t    delay_ms;     // Post-command delay in milliseconds
} lcd_init_cmd_t;

// Per-rotation configuration for MADCTL-driven displays
typedef struct {
    uint8_t madctl_cmd;          // MADCTL register address (e.g. 0x36)
    uint8_t rotation_values[4];  // MADCTL byte for 0°, 90°, 180°, 270°
    bool    swap_dimensions[4];  // Whether to swap width/height for each angle
    uint8_t colstart[4];         // GRAM column offset per rotation
    uint8_t rowstart[4];         // GRAM row offset per rotation
} lcd_rotation_config_t;

// LCD vendor / panel configuration
typedef struct {
    const lcd_init_cmd_t *init_cmds;      // Initialization command table (static const)
    uint16_t              init_cmds_size;  // Number of entries in init_cmds
    bool color_inverted;    // Software-invert every pixel in pushColors
    bool order_rgb;         // true = RGB (no channel swap); false = BGR swap R/B
    bool swap_bytes;        // Swap the two bytes of each RGB565 word (big-endian SPI)
    bool reset_active_low;  // Reset pin polarity: true = active LOW
    uint16_t reset_pulse_ms; // Duration of reset pulse in ms
    bool bl_active_high;    // Backlight PWM polarity
    uint8_t spi_mode;       // SPI mode: 0 = MODE0, 3 = MODE3 (ST7789 typically needs MODE3)
    lcd_rotation_config_t rotation; // Optional rotation config (set madctl_cmd=0 to disable)
} lcd_vendor_config_t;


// Abstract display interface
class Display {
protected:
    int _width  = 0;
    int _height = 0;
    lcd_vendor_config_t _vendor_config;

public:
    Display(uint16_t width, uint16_t height)
        : _width(width), _height(height) {}

    virtual ~Display() {}

    virtual bool init()    = 0;
    virtual void deinit()  = 0;
    virtual void load_vendor_config(const lcd_vendor_config_t &config) = 0;
    virtual void reset()   = 0;
    virtual void refresh() = 0;
    virtual void rotate(uint16_t deg) = 0;      // 0, 90, 180, 270
    virtual void blctrl(float brightness) = 0;  // 0.0 (off) .. 1.0 (full)

    // Pixel transfer — called by LVGL flush callback
    virtual void setAddrWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h) = 0;
    virtual void pushColors(uint16_t *data, uint32_t len, bool last_chunk = true) = 0;
    virtual void startWrite() = 0;
    virtual void endWrite()   = 0;

    inline int width()  const { return _width;  }
    inline int height() const { return _height; }
};
