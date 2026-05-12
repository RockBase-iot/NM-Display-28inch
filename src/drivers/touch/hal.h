#pragma once

// Touch Hardware Abstraction Layer.
// Defines the Touch abstract interface and supporting types used by
// concrete touch drivers (IICTouch, SPITouch, etc.).

#include <Arduino.h>

// Single touch point
typedef struct {
    uint16_t x;        // X coordinate (logical display pixels)
    uint16_t y;        // Y coordinate (logical display pixels)
    bool     pressed;  // true = finger down
} touch_point_t;

// Coordinate transform flags applied after the raw-to-pixel mapping.
// One entry per rotation angle (0°=idx0, 90°=idx1, 180°=idx2, 270°=idx3).
typedef struct {
    bool swap_xy;   // Swap X and Y after mapping
    bool mirror_x;  // Mirror X  (x = width  - x)
    bool mirror_y;  // Mirror Y  (y = height - y)
} touch_rotation_t;

// Touch panel hardware configuration
typedef struct {
    const uint8_t  *init_cmds;       // Register-value pairs to write during init (may be nullptr)
    uint16_t        init_cmds_size;  // Byte count of init_cmds
    uint16_t        x_max;           // Raw sensor X upper bound (0 disables auto-scaling)
    uint16_t        y_max;           // Raw sensor Y upper bound (0 disables auto-scaling)
    touch_rotation_t rotation_map[4]; // Transform per rotation index
} touch_config_t;


// Abstract touch controller
class Touch {
protected:
    uint16_t _width  = 0;
    uint16_t _height = 0;
    touch_config_t _config{};

public:
    Touch(uint16_t width, uint16_t height)
        : _width(width), _height(height) {}

    virtual ~Touch() {}

    virtual bool init()   = 0;
    virtual void deinit() = 0;
    virtual void load_config(const touch_config_t &cfg) = 0;
    virtual void reset()  = 0;
    virtual bool read(touch_point_t *point)   = 0;
    virtual void set_rotation(uint16_t deg)   = 0;  // 0, 90, 180, 270

    virtual void sleep()  {}
    virtual void wakeup() {}

    inline uint16_t width()  const { return _width;  }
    inline uint16_t height() const { return _height; }
};
