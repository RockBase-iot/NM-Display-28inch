#pragma once

// SPI display driver — wraps the Arduino SPI bus and implements the Display interface.
// Supports any SPI-connected LCD panel (ST7789, ILI9341, etc.) via a vendor config table.

#include <Arduino.h>
#include <SPI.h>
#include "hal.h"

class SPIScreen : public Display {
private:
    SPIClass *_spi         = nullptr;
    int8_t    _dcPin       = -1;
    int8_t    _rstPin      = -1;
    int8_t    _csPin       = -1;
    int8_t    _blPin       = -1;
    int8_t    _blChannel   = 0;    // LEDC channel for backlight PWM
    int8_t    _blRes       = 8;    // LEDC resolution bits (default 8-bit, 0-255)
    uint32_t  _spiFreq     = 40000000;
    uint16_t  _colorBuf[1024];     // Scratch buffer for colour-transform chunks
    int       _initWidth   = 0;
    int       _initHeight  = 0;
    uint16_t  _colOffset   = 0;
    uint16_t  _rowOffset   = 0;

public:
    // spiFreq   : SPI clock in Hz (e.g. 80000000 for ST7789 at 80 MHz)
    // blChannel : LEDC channel for backlight (pick one not used by other peripherals)
    SPIScreen(SPIClass *spi, int8_t dcPin, int8_t rstPin, int8_t csPin, int8_t blPin,
              uint16_t width, uint16_t height,
              uint32_t spiFreq = 40000000,
              int8_t blChannel = 0)
        : Display(width, height),
          _spi(spi), _dcPin(dcPin), _rstPin(rstPin), _csPin(csPin), _blPin(blPin),
          _blChannel(blChannel), _spiFreq(spiFreq),
          _initWidth(width), _initHeight(height)
    {
        if (_dcPin  != -1) pinMode(_dcPin,  OUTPUT);
        if (_rstPin != -1) pinMode(_rstPin, OUTPUT);
        if (_csPin  != -1) pinMode(_csPin,  OUTPUT);
        if (_blPin  != -1) pinMode(_blPin,  OUTPUT);
    }

    ~SPIScreen() {}

    void load_vendor_config(const lcd_vendor_config_t &cfg) override {
        _vendor_config = cfg;
    }

    bool init() override {
        if (!_spi || !_vendor_config.init_cmds) return false;

        _spi->beginTransaction(SPISettings(_spiFreq, MSBFIRST, _vendor_config.spi_mode));

        const lcd_init_cmd_t *cmd = _vendor_config.init_cmds;
        for (int i = 0; i < _vendor_config.init_cmds_size; i++, cmd++) {
            if (_csPin != -1) digitalWrite(_csPin, LOW);
            if (_dcPin != -1) digitalWrite(_dcPin, LOW);   // command phase
            _spi->transfer((uint8_t)cmd->cmd);

            if (cmd->data && cmd->data_bytes > 0) {
                if (_dcPin != -1) digitalWrite(_dcPin, HIGH); // data phase
                _spi->writeBytes((const uint8_t *)cmd->data, cmd->data_bytes);
            }
            if (_csPin != -1) digitalWrite(_csPin, HIGH);

            if (cmd->delay_ms > 0) {
                _spi->endTransaction();
                delay(cmd->delay_ms);
                _spi->beginTransaction(SPISettings(_spiFreq, MSBFIRST, _vendor_config.spi_mode));
            }
        }
        _spi->endTransaction();
        return true;
    }

    void deinit() override {}

    void reset() override {
        if (_rstPin == -1) return;
        if (_vendor_config.reset_active_low) {
            digitalWrite(_rstPin, LOW);
            delay(_vendor_config.reset_pulse_ms);
            digitalWrite(_rstPin, HIGH);
        } else {
            digitalWrite(_rstPin, HIGH);
            delay(_vendor_config.reset_pulse_ms);
            digitalWrite(_rstPin, LOW);
        }
        delay(_vendor_config.reset_pulse_ms);
    }

    void refresh() override {}

    // Set the active pixel window for the next pushColors call.
    // Sends CASET (0x2A) + PASET (0x2B) + RAMWR (0x2C) start.
    // CS is left LOW after RAMWR so pushColors can stream data immediately.
    void setAddrWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h) override {
        if (!_spi) return;

        uint16_t xa = x + _colOffset;
        uint16_t ya = y + _rowOffset;

        uint8_t caset[4] = {
            (uint8_t)(xa >> 8), (uint8_t)(xa & 0xFF),
            (uint8_t)((xa + w - 1) >> 8), (uint8_t)((xa + w - 1) & 0xFF)
        };
        uint8_t paset[4] = {
            (uint8_t)(ya >> 8), (uint8_t)(ya & 0xFF),
            (uint8_t)((ya + h - 1) >> 8), (uint8_t)((ya + h - 1) & 0xFF)
        };

        // CASET
        if (_csPin != -1) digitalWrite(_csPin, LOW);
        if (_dcPin != -1) digitalWrite(_dcPin, LOW);
        _spi->transfer(0x2A);
        if (_dcPin != -1) digitalWrite(_dcPin, HIGH);
        _spi->writeBytes(caset, 4);
        if (_csPin != -1) digitalWrite(_csPin, HIGH);

        // PASET
        if (_csPin != -1) digitalWrite(_csPin, LOW);
        if (_dcPin != -1) digitalWrite(_dcPin, LOW);
        _spi->transfer(0x2B);
        if (_dcPin != -1) digitalWrite(_dcPin, HIGH);
        _spi->writeBytes(paset, 4);
        if (_csPin != -1) digitalWrite(_csPin, HIGH);

        // RAMWR — CS stays LOW; pushColors streams pixel data
        if (_csPin != -1) digitalWrite(_csPin, LOW);
        if (_dcPin != -1) digitalWrite(_dcPin, LOW);
        _spi->transfer(0x2C);
        if (_dcPin != -1) digitalWrite(_dcPin, HIGH);
    }

    // Stream pixel data.  Applies byte-swap / BGR / inversion transforms as needed.
    // Closes CS when last_chunk == true.
    void pushColors(uint16_t *data, uint32_t len, bool last_chunk = true) override {
        if (!_spi || !data || len == 0) return;

        bool need_xform = (!_vendor_config.order_rgb ||
                           _vendor_config.color_inverted ||
                           _vendor_config.swap_bytes);

        if (!need_xform) {
            _spi->writeBytes((uint8_t *)data, len * 2);
        } else {
            uint32_t offset = 0;
            while (offset < len) {
                uint32_t chunk = (len - offset) > 1024 ? 1024 : (len - offset);
                for (uint32_t i = 0; i < chunk; i++) {
                    uint16_t c = data[offset + i];
                    if (!_vendor_config.order_rgb) {
                        uint16_t r = (c >> 11) & 0x1F;
                        uint16_t g = (c >>  5) & 0x3F;
                        uint16_t b =  c        & 0x1F;
                        c = (b << 11) | (g << 5) | r;
                    }
                    if (_vendor_config.swap_bytes)     c = (c >> 8) | (c << 8);
                    if (_vendor_config.color_inverted) c = ~c;
                    _colorBuf[i] = c;
                }
                _spi->writeBytes((uint8_t *)_colorBuf, chunk * 2);
                offset += chunk;
            }
        }

        if (last_chunk && _csPin != -1) {
            digitalWrite(_csPin, HIGH);
        }
    }

    // Must be called before setAddrWindow to start the SPI transaction.
    // Re-asserts the display SPI frequency in case a slower device (e.g. touch)
    // changed the bus settings since the last write.
    void startWrite() override {
        if (_spi) _spi->beginTransaction(SPISettings(_spiFreq, MSBFIRST, _vendor_config.spi_mode));
    }

    void endWrite() override {
        if (!_spi) return;
        if (_csPin != -1) digitalWrite(_csPin, HIGH);
        _spi->endTransaction();
    }

    // Send MADCTL for the given rotation and update _width/_height accordingly.
    void rotate(uint16_t degree) override {
        if (!_spi || _vendor_config.rotation.madctl_cmd == 0) return;

        uint8_t idx  = (degree / 90) % 4;
        uint8_t mval = _vendor_config.rotation.rotation_values[idx];

        _spi->beginTransaction(SPISettings(_spiFreq, MSBFIRST, _vendor_config.spi_mode));
        if (_csPin != -1) digitalWrite(_csPin, LOW);
        if (_dcPin != -1) digitalWrite(_dcPin, LOW);
        _spi->transfer(_vendor_config.rotation.madctl_cmd);
        if (_dcPin != -1) digitalWrite(_dcPin, HIGH);
        _spi->writeBytes(&mval, 1);
        if (_csPin != -1) digitalWrite(_csPin, HIGH);
        _spi->endTransaction();

        _colOffset = _vendor_config.rotation.colstart[idx];
        _rowOffset = _vendor_config.rotation.rowstart[idx];

        if (_vendor_config.rotation.swap_dimensions[idx]) {
            _width  = _initHeight;
            _height = _initWidth;
        } else {
            _width  = _initWidth;
            _height = _initHeight;
        }
    }

    // Set backlight brightness via LEDC PWM.
    // Lazily sets up the LEDC channel on first call.
    void blctrl(float brightness) override {
        if (_blPin == -1) return;

        static bool pwm_ready = false;
        if (!pwm_ready) {
            ledcSetup(_blChannel, 5000, _blRes);
            ledcAttachPin(_blPin, _blChannel);
            pwm_ready = true;
        }

        float b = (brightness <= 0.0f) ? 0.0f : (brightness >= 1.0f ? 0.99f : brightness);
        int   maxVal = (1 << _blRes) - 1;
        int   duty   = _vendor_config.bl_active_high ? (int)(b * maxVal) : (int)((1.0f - b) * maxVal);
        ledcWrite(_blChannel, duty);
    }
};
