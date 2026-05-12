#pragma once

// I2C touch controller driver.
// Compatible with FT6x06-family controllers: FT6206, FT6336, FT5316, etc.
// Pass sdaPin / sclPin as -1 if the Wire bus is already initialised.

#include <Arduino.h>
#include <Wire.h>
#include "hal.h"

class IICTouch : public Touch {
private:
    TwoWire *_wire        = nullptr;
    uint8_t  _addr        = 0x00;
    int8_t   _sdaPin      = -1;
    int8_t   _sclPin      = -1;
    int8_t   _irqPin      = -1;
    int8_t   _rstPin      = -1;
    uint32_t _i2cFreq     = 400000;
    uint16_t _rotation    = 0;
    touch_rotation_t _curRot{false, false, false};
    bool _ready = false;

    // ── I2C helpers ─────────────────────────────────────────────────────────

    uint8_t _readReg(uint8_t reg) {
        _wire->beginTransmission(_addr);
        _wire->write(reg);
        _wire->endTransmission(false);
        _wire->requestFrom(_addr, (uint8_t)1);
        return _wire->available() ? _wire->read() : 0;
    }

    void _writeReg(uint8_t reg, uint8_t val) {
        _wire->beginTransmission(_addr);
        _wire->write(reg);
        _wire->write(val);
        _wire->endTransmission();
    }

    bool _readBytes(uint8_t reg, uint8_t *buf, size_t len) {
        _wire->beginTransmission(_addr);
        _wire->write(reg);
        _wire->endTransmission(false);
        _wire->requestFrom(_addr, (uint8_t)len);
        size_t n = 0;
        while (_wire->available() && n < len) buf[n++] = _wire->read();
        return n == len;
    }

    // ── Coordinate transform ─────────────────────────────────────────────────

    void _applyTransform(touch_point_t *pt) {
        if (_curRot.swap_xy) {
            uint16_t t = pt->x; pt->x = pt->y; pt->y = t;
        }
        if (_curRot.mirror_x) pt->x = _width  - 1 - pt->x;
        if (_curRot.mirror_y) pt->y = _height - 1 - pt->y;
    }

public:
    IICTouch(TwoWire *wire, uint8_t i2cAddr,
             int8_t sdaPin, int8_t sclPin,
             int8_t irqPin, int8_t rstPin,
             uint16_t width, uint16_t height,
             uint32_t i2cFreq = 400000)
        : Touch(width, height),
          _wire(wire), _addr(i2cAddr),
          _sdaPin(sdaPin), _sclPin(sclPin),
          _irqPin(irqPin), _rstPin(rstPin),
          _i2cFreq(i2cFreq)
    {
        if (_irqPin != -1) pinMode(_irqPin, INPUT_PULLUP);
        if (_rstPin != -1) { pinMode(_rstPin, OUTPUT); digitalWrite(_rstPin, HIGH); }
    }

    ~IICTouch() { deinit(); }

    bool init() override {
        if (!_wire) return false;

        // Only (re-)init I2C bus when pins are explicitly provided.
        // If sdaPin == -1 the host already called Wire.begin() in the BSP.
        if (_sdaPin != -1 && _sclPin != -1) {
            _wire->begin(_sdaPin, _sclPin);
            _wire->setClock(_i2cFreq);
        }

        reset();
        delay(50);

        // Confirm device presence
        _wire->beginTransmission(_addr);
        if (_wire->endTransmission() != 0) return false;

        // Send optional init register-value pairs
        if (_config.init_cmds && _config.init_cmds_size > 0) {
            for (uint16_t i = 0; i + 1 < _config.init_cmds_size; i += 2) {
                _writeReg(_config.init_cmds[i], _config.init_cmds[i + 1]);
                delay(5);
            }
        }

        _ready = true;
        return true;
    }

    void deinit() override { _ready = false; }

    void load_config(const touch_config_t &cfg) override { _config = cfg; }

    void reset() override {
        if (_rstPin != -1) {
            digitalWrite(_rstPin, LOW);
            delay(10);
            digitalWrite(_rstPin, HIGH);
            delay(50);
        }
    }

    // Read one touch point using the FT6x06 register protocol.
    //
    // FT6x06 register map (starting at 0x00):
    //   0x02  TD_STATUS — number of touch points (bits[3:0])
    //   0x03  P1_XH    — event flag (bits[7:6]) + X high nibble (bits[3:0])
    //   0x04  P1_XL    — X low byte
    //   0x05  P1_YH    — touch ID (bits[7:4]) + Y high nibble (bits[3:0])
    //   0x06  P1_YL    — Y low byte
    bool read(touch_point_t *pt) override {
        if (!_ready || !_wire || !pt) {
            if (pt) pt->pressed = false;
            return false;
        }

        // IRQ pin fast-exit (active low; skip if pin not used)
        if (_irqPin != -1 && digitalRead(_irqPin) == HIGH) {
            pt->pressed = false;
            return true;
        }

        uint8_t data[7];
        if (_readBytes(0x00, data, 7)) {
            uint8_t points = data[0x02] & 0x0F;
            if (points > 0) {
                uint16_t rx = ((uint16_t)(data[0x03] & 0x0F) << 8) | data[0x04];
                uint16_t ry = ((uint16_t)(data[0x05] & 0x0F) << 8) | data[0x06];

                // Scale raw values to pixel coordinates
                pt->x = (_config.x_max > 0)
                    ? (uint16_t)map((long)rx, 0, _config.x_max - 1, 0, _width  - 1)
                    : (uint16_t)rx;
                pt->y = (_config.y_max > 0)
                    ? (uint16_t)map((long)ry, 0, _config.y_max - 1, 0, _height - 1)
                    : (uint16_t)ry;
                pt->pressed = true;
                _applyTransform(pt);
            } else {
                pt->pressed = false;
            }
        } else {
            pt->pressed = false;
        }
        return true;
    }

    void set_rotation(uint16_t rotation) override {
        uint8_t idx = (rotation / 90) % 4;
        _curRot  = _config.rotation_map[idx];
        _rotation = rotation;
    }

    void sleep() override {
        // FT6x06 hibernate: write 0x03 to PMODE register (0xA5)
        _writeReg(0xA5, 0x03);
    }

    void wakeup() override {
        reset();
        delay(50);
        if (_config.init_cmds && _config.init_cmds_size > 0) {
            for (uint16_t i = 0; i + 1 < _config.init_cmds_size; i += 2) {
                _writeReg(_config.init_cmds[i], _config.init_cmds[i + 1]);
                delay(5);
            }
        }
    }
};
