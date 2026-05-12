#include "factory_test.h"
#include "../../app/task_config.h"
#include "../../utils/logger.h"
#include "../../drivers/display/lvgl_port.h"
#include "../../bsp/nm_display_28/config.h"

#include <lvgl.h>
#include <Wire.h>
#include <WiFi.h>
#include <SD_MMC.h>
#include <SPIFFS.h>
#include <esp_camera.h>

// ─── Colour palette used across test screens ─────────────────────────────────
#define COLOR_BG        0x1A1A2E
#define COLOR_TITLE_BG  0x16213E
#define COLOR_PASS      0x2ECC71
#define COLOR_FAIL      0xE74C3C
#define COLOR_SKIP      0x7F8C8D
#define COLOR_TEXT      0xECF0F1
#define COLOR_SUBTEXT   0xBDC3C7

// ─── Constructor ─────────────────────────────────────────────────────────────

FactoryTest::FactoryTest(Board &board) : _board(board) {}

// ─── Public API ──────────────────────────────────────────────────────────────

void FactoryTest::start()
{
    if (_task_handle) return;
    xTaskCreatePinnedToCore(
        _task_entry,
        "factory_test",
        TASK_STACK_FACTORY_TEST,
        this,
        TASK_PRIORITY_SENSOR,
        &_task_handle,
        FactoryTestTaskCore);
}

// ─── FreeRTOS task ───────────────────────────────────────────────────────────

void FactoryTest::_task_entry(void *arg)
{
    FactoryTest *self = static_cast<FactoryTest *>(arg);
    self->_run_all();
    self->_task_handle = nullptr;
    vTaskDelete(nullptr);
}

// ─── Test orchestration ──────────────────────────────────────────────────────

void FactoryTest::_run_all()
{
    struct TestCase {
        const char *name;
        Result (FactoryTest::*fn)();
    };

    constexpr TestCase cases[] = {
        { "Display",  &FactoryTest::_test_display  },
        { "Touch",    &FactoryTest::_test_touch    },
        { "SD Card",  &FactoryTest::_test_sdcard   },
        { "WiFi",     &FactoryTest::_test_wifi     },
        { "IMU",      &FactoryTest::_test_imu      },
        { "PMU",      &FactoryTest::_test_pmu      },
        { "RTC",      &FactoryTest::_test_rtc      },
        { "Camera",   &FactoryTest::_test_camera   },
        { "Audio",    &FactoryTest::_test_audio    },
    };
    constexpr int TOTAL = sizeof(cases) / sizeof(cases[0]);

    int pass = 0, fail = 0, skip = 0;
    char fail_names[TOTAL][32];
    int  fail_count = 0;

    for (int i = 0; i < TOTAL; i++) {
        LOG_I("FactoryTest [%d/%d]: %s", i + 1, TOTAL, cases[i].name);
        Result r = (this->*(cases[i].fn))();

        switch (r) {
            case Result::PASS: pass++; break;
            case Result::FAIL:
                fail++;
                strncpy(fail_names[fail_count++], cases[i].name, 31);
                break;
            case Result::SKIP: skip++; break;
        }
        vTaskDelay(pdMS_TO_TICKS(800));   // brief pause between tests
    }

    _show_summary(pass, fail, skip, fail_names, fail_count);
}

// ─── LVGL screen helpers ─────────────────────────────────────────────────────

static const char *_result_str(FactoryTest::Result r) {
    switch (r) {
        case FactoryTest::Result::PASS: return "  PASS  ";
        case FactoryTest::Result::FAIL: return "  FAIL  ";
        default:                         return "  ----  ";
    }
}

static uint32_t _result_color(FactoryTest::Result r) {
    switch (r) {
        case FactoryTest::Result::PASS: return COLOR_PASS;
        case FactoryTest::Result::FAIL: return COLOR_FAIL;
        default:                         return COLOR_SKIP;
    }
}

void FactoryTest::_show_screen(uint8_t index, const char *name,
                                const char *body, Result result)
{
    if (!LVGL_LOCK(500)) return;

    lv_obj_t *scr = lv_obj_create(nullptr);
    lv_obj_set_style_bg_color(scr, lv_color_hex(COLOR_BG), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(scr, 0, 0);

    // Title bar
    lv_obj_t *title_bar = lv_obj_create(scr);
    lv_obj_set_size(title_bar, SCREEN_WIDTH, 36);
    lv_obj_align(title_bar, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(title_bar, lv_color_hex(COLOR_TITLE_BG), 0);
    lv_obj_set_style_border_width(title_bar, 0, 0);
    lv_obj_set_style_radius(title_bar, 0, 0);
    lv_obj_set_style_pad_all(title_bar, 4, 0);

    char title_buf[48];
    snprintf(title_buf, sizeof(title_buf), "Test %d/9: %s", index, name);
    lv_obj_t *title_lbl = lv_label_create(title_bar);
    lv_label_set_text(title_lbl, title_buf);
    lv_obj_set_style_text_color(title_lbl, lv_color_hex(COLOR_TEXT), 0);
    lv_obj_center(title_lbl);

    // Body text
    lv_obj_t *body_lbl = lv_label_create(scr);
    lv_label_set_text(body_lbl, body);
    lv_obj_set_style_text_color(body_lbl, lv_color_hex(COLOR_SUBTEXT), 0);
    lv_obj_set_width(body_lbl, SCREEN_WIDTH - 20);
    lv_label_set_long_mode(body_lbl, LV_LABEL_LONG_WRAP);
    lv_obj_align(body_lbl, LV_ALIGN_TOP_LEFT, 10, 46);

    // Result badge
    lv_obj_t *badge = lv_label_create(scr);
    lv_label_set_text(badge, _result_str(result));
    lv_obj_set_style_text_color(badge, lv_color_hex(_result_color(result)), 0);
    lv_obj_align(badge, LV_ALIGN_BOTTOM_MID, 0, -12);

    // Cache widget pointers for live updates
    _lv_body_label  = body_lbl;
    _lv_badge_label = badge;

    lv_scr_load(scr);
    LVGL_UNLOCK();
}

void FactoryTest::_update_screen(const char *body, Result result)
{
    if (!_lv_body_label || !_lv_badge_label) return;
    if (!LVGL_LOCK(100)) return;
    lv_label_set_text(static_cast<lv_obj_t *>(_lv_body_label),  body);
    lv_label_set_text(static_cast<lv_obj_t *>(_lv_badge_label), _result_str(result));
    lv_obj_set_style_text_color(static_cast<lv_obj_t *>(_lv_badge_label),
                                 lv_color_hex(_result_color(result)), 0);
    LVGL_UNLOCK();
}

bool FactoryTest::_wait_touch(uint32_t wait_ms)
{
    Touch *touch = _board.get_touch();
    if (!touch) return false;

    const uint32_t poll_ms  = 50;
    uint32_t       elapsed  = 0;
    while (elapsed < wait_ms) {
        touch_point_t tp;
        touch->read(&tp);
        if (tp.pressed) return true;
        vTaskDelay(pdMS_TO_TICKS(poll_ms));
        elapsed += poll_ms;
    }
    return false;
}

void FactoryTest::_show_summary(int pass, int fail, int skip,
                                 const char fail_names[][32], int fail_count)
{
    bool overall_pass = (fail == 0);

    char body[256];
    int  n = 0;
    n += snprintf(body + n, (int)sizeof(body) - n,
                  "PASS  %d\nFAIL  %d\nSKIP  %d\n", pass, fail, skip);
    if (fail_count > 0) {
        n += snprintf(body + n, (int)sizeof(body) - n, "\nFailed:");
        for (int i = 0; i < fail_count && n < (int)sizeof(body) - 20; i++) {
            n += snprintf(body + n, (int)sizeof(body) - n, " %s", fail_names[i]);
        }
    }

    if (!LVGL_LOCK(500)) return;

    lv_obj_t *scr = lv_obj_create(nullptr);
    lv_obj_set_style_bg_color(scr, lv_color_hex(COLOR_BG), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(scr, 0, 0);

    lv_obj_t *title_lbl = lv_label_create(scr);
    lv_label_set_text(title_lbl, "Factory Test Complete");
    lv_obj_set_style_text_color(title_lbl, lv_color_hex(COLOR_TEXT), 0);
    lv_obj_align(title_lbl, LV_ALIGN_TOP_MID, 0, 10);

    lv_obj_t *body_lbl = lv_label_create(scr);
    lv_label_set_text(body_lbl, body);
    lv_obj_set_style_text_color(body_lbl, lv_color_hex(COLOR_SUBTEXT), 0);
    lv_obj_set_width(body_lbl, SCREEN_WIDTH - 20);
    lv_label_set_long_mode(body_lbl, LV_LABEL_LONG_WRAP);
    lv_obj_align(body_lbl, LV_ALIGN_TOP_LEFT, 10, 40);

    lv_obj_t *verdict = lv_label_create(scr);
    lv_label_set_text(verdict, overall_pass ? "  OVERALL: PASS  " : "  OVERALL: FAIL  ");
    lv_obj_set_style_text_color(verdict,
        lv_color_hex(overall_pass ? COLOR_PASS : COLOR_FAIL), 0);
    lv_obj_align(verdict, LV_ALIGN_BOTTOM_MID, 0, -30);

    lv_obj_t *hint = lv_label_create(scr);
    lv_label_set_text(hint, "< tap to restart >");
    lv_obj_set_style_text_color(hint, lv_color_hex(COLOR_SKIP), 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -10);

    lv_scr_load(scr);
    LVGL_UNLOCK();

    LOG_I("FactoryTest summary: PASS=%d FAIL=%d SKIP=%d", pass, fail, skip);

    // Wait for user to tap, then restart the device.
    _wait_touch(60000);
    ESP.restart();
}

// ─── Test cases ──────────────────────────────────────────────────────────────

FactoryTest::Result FactoryTest::_test_display()
{
    _show_screen(1, "Display", "Filling colors...", Result::SKIP);

    struct { uint32_t color; const char *name; } steps[] = {
        { 0xFF0000, "Red"   },
        { 0x00FF00, "Green" },
        { 0x0000FF, "Blue"  },
        { 0xFFFFFF, "White" },
        { 0x000000, "Black" },
    };

    Display *disp = _board.get_display();
    if (!disp) {
        _update_screen("No display driver", Result::FAIL);
        vTaskDelay(pdMS_TO_TICKS(2000));
        return Result::FAIL;
    }

    for (auto &s : steps) {
        char msg[48];
        snprintf(msg, sizeof(msg), "Filling: %s", s.name);
        _update_screen(msg, Result::SKIP);

        // Fill via LVGL (let LVGL draw a solid-color rectangle)
        if (LVGL_LOCK(200)) {
            lv_obj_t *scr = lv_scr_act();
            lv_obj_set_style_bg_color(scr, lv_color_hex(s.color), 0);
            lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
            lv_obj_invalidate(scr);
            LVGL_UNLOCK();
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    _show_screen(1, "Display", "All colors rendered", Result::PASS);
    vTaskDelay(pdMS_TO_TICKS(1000));
    return Result::PASS;
}

FactoryTest::Result FactoryTest::_test_touch()
{
    _show_screen(2, "Touch", "Tap 3 different spots\nwithin 15 seconds.", Result::SKIP);

    Touch *touch = _board.get_touch();
    if (!touch) {
        _update_screen("No touch driver", Result::SKIP);
        vTaskDelay(pdMS_TO_TICKS(2000));
        return Result::SKIP;
    }

    int taps = 0;
    char msg[64];
    const uint32_t timeout_ms = 15000;
    const uint32_t poll_ms    = 50;
    uint32_t elapsed = 0;
    bool     prev_pressed = false;

    while (taps < 3 && elapsed < timeout_ms) {
        touch_point_t tp;
        touch->read(&tp);

        if (tp.pressed && !prev_pressed) {
            taps++;
            snprintf(msg, sizeof(msg), "Tap %d/3 at (%d, %d)", taps, tp.x, tp.y);
            _update_screen(msg, Result::SKIP);
            LOG_I("Touch tap %d: x=%d y=%d", taps, tp.x, tp.y);
        }
        prev_pressed = tp.pressed;

        vTaskDelay(pdMS_TO_TICKS(poll_ms));
        elapsed += poll_ms;
    }

    if (taps >= 3) {
        _update_screen("3 taps registered", Result::PASS);
        vTaskDelay(pdMS_TO_TICKS(1000));
        return Result::PASS;
    } else {
        snprintf(msg, sizeof(msg), "Only %d/3 taps (timeout)", taps);
        _update_screen(msg, Result::FAIL);
        vTaskDelay(pdMS_TO_TICKS(1500));
        return Result::FAIL;
    }
}

FactoryTest::Result FactoryTest::_test_sdcard()
{
    _show_screen(3, "SD Card", "Mounting...", Result::SKIP);

    SD_MMC.setPins(SD_CLK_PIN, SD_CMD_PIN, SD_D0_PIN);
    if (!SD_MMC.begin("/sdcard", true)) {   // true = 1-bit mode
        _update_screen("Mount failed\n(SD inserted?)", Result::FAIL);
        vTaskDelay(pdMS_TO_TICKS(2000));
        return Result::FAIL;
    }

    const char *test_path = "/sdcard/factory_test.tmp";
    const char *test_data = "NMDisplay28-FactoryTest-OK";

    // Write
    _update_screen("Mounted\nWriting...", Result::SKIP);
    File f = SD_MMC.open(test_path, FILE_WRITE);
    if (!f) {
        _update_screen("Write open failed", Result::FAIL);
        SD_MMC.end();
        vTaskDelay(pdMS_TO_TICKS(2000));
        return Result::FAIL;
    }
    f.print(test_data);
    f.close();

    // Read back
    _update_screen("Written\nReading...", Result::SKIP);
    f = SD_MMC.open(test_path, FILE_READ);
    if (!f) {
        _update_screen("Read open failed", Result::FAIL);
        SD_MMC.end();
        vTaskDelay(pdMS_TO_TICKS(2000));
        return Result::FAIL;
    }
    char read_buf[64] = {};
    f.readBytes(read_buf, sizeof(read_buf) - 1);
    f.close();
    SD_MMC.remove(test_path);   // clean up
    SD_MMC.end();

    if (strcmp(read_buf, test_data) == 0) {
        _update_screen("Write OK\nRead OK\nContent matches", Result::PASS);
        vTaskDelay(pdMS_TO_TICKS(1500));
        return Result::PASS;
    } else {
        _update_screen("Content mismatch!", Result::FAIL);
        vTaskDelay(pdMS_TO_TICKS(2000));
        return Result::FAIL;
    }
}

FactoryTest::Result FactoryTest::_test_wifi()
{
    _show_screen(4, "WiFi", "Scanning...", Result::SKIP);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    vTaskDelay(pdMS_TO_TICKS(100));

    int n = WiFi.scanNetworks(false, true);   // sync scan, include hidden
    WiFi.mode(WIFI_OFF);

    if (n <= 0) {
        _update_screen("No networks found", Result::FAIL);
        vTaskDelay(pdMS_TO_TICKS(2000));
        return Result::FAIL;
    }

    char msg[128];
    int  written = snprintf(msg, sizeof(msg), "Found %d APs:\n", n);
    for (int i = 0; i < n && i < 4 && written < (int)sizeof(msg) - 30; i++) {
        written += snprintf(msg + written, sizeof(msg) - written,
                            "%s (%d dBm)\n", WiFi.SSID(i).c_str(), WiFi.RSSI(i));
    }
    WiFi.scanDelete();

    _update_screen(msg, Result::PASS);
    vTaskDelay(pdMS_TO_TICKS(2500));
    return Result::PASS;
}

// Helper: probe I2C address, return true if ACK received.
static bool i2c_probe(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
}

FactoryTest::Result FactoryTest::_test_imu()
{
    // QMI8658 default address: 0x6B (SA0 tied high) or 0x6A.
    constexpr uint8_t QMI8658_ADDR = 0x6B;
    _show_screen(5, "IMU (QMI8658)", "Probing I2C...", Result::SKIP);

    if (!i2c_probe(QMI8658_ADDR)) {
        _update_screen("No ACK at 0x6B\nor 0x6A", Result::FAIL);
        vTaskDelay(pdMS_TO_TICKS(2000));
        return Result::FAIL;
    }

    // Read WHO_AM_I (register 0x00) — QMI8658A returns 0x05.
    Wire.beginTransmission(QMI8658_ADDR);
    Wire.write(0x00);
    Wire.endTransmission(false);
    Wire.requestFrom(QMI8658_ADDR, (uint8_t)1);
    uint8_t who = Wire.available() ? Wire.read() : 0xFF;

    char msg[64];
    snprintf(msg, sizeof(msg), "ACK at 0x%02X\nWHO_AM_I = 0x%02X%s",
             QMI8658_ADDR, who, (who == 0x05) ? " (OK)" : " (unexpected)");
    _update_screen(msg, (who == 0x05) ? Result::PASS : Result::FAIL);
    vTaskDelay(pdMS_TO_TICKS(2000));
    return (who == 0x05) ? Result::PASS : Result::FAIL;
}

FactoryTest::Result FactoryTest::_test_pmu()
{
    _show_screen(6, "PMU (AXP2101)", "Probing I2C...", Result::SKIP);

    if (!i2c_probe(AXP2101_I2C_ADDR)) {
        _update_screen("No ACK at 0x34", Result::FAIL);
        vTaskDelay(pdMS_TO_TICKS(2000));
        return Result::FAIL;
    }

    // Read chip ID register (0x03) — AXP2101 returns 0x4A.
    Wire.beginTransmission(AXP2101_I2C_ADDR);
    Wire.write(0x03);
    Wire.endTransmission(false);
    Wire.requestFrom(AXP2101_I2C_ADDR, (uint8_t)1);
    uint8_t chip_id = Wire.available() ? Wire.read() : 0xFF;

    char msg[64];
    snprintf(msg, sizeof(msg), "ACK at 0x%02X\nChip ID = 0x%02X%s",
             AXP2101_I2C_ADDR, chip_id, (chip_id == 0x4A) ? " (OK)" : " (unexpected)");
    _update_screen(msg, (chip_id == 0x4A) ? Result::PASS : Result::FAIL);
    vTaskDelay(pdMS_TO_TICKS(2000));
    return (chip_id == 0x4A) ? Result::PASS : Result::FAIL;
}

FactoryTest::Result FactoryTest::_test_rtc()
{
    // PCF85063 I2C address: 0x51.
    constexpr uint8_t PCF85063_ADDR = 0x51;
    _show_screen(7, "RTC (PCF85063)", "Probing I2C...", Result::SKIP);

    if (!i2c_probe(PCF85063_ADDR)) {
        _update_screen("No ACK at 0x51", Result::FAIL);
        vTaskDelay(pdMS_TO_TICKS(2000));
        return Result::FAIL;
    }

    // Read seconds register (0x04) twice 1s apart; it should increment.
    auto read_seconds = [&]() -> uint8_t {
        Wire.beginTransmission(PCF85063_ADDR);
        Wire.write(0x04);
        Wire.endTransmission(false);
        Wire.requestFrom(PCF85063_ADDR, (uint8_t)1);
        uint8_t v = Wire.available() ? Wire.read() : 0xFF;
        return v & 0x7F;   // strip OS bit
    };

    uint8_t s1 = read_seconds();
    _update_screen("Reading seconds...", Result::SKIP);
    vTaskDelay(pdMS_TO_TICKS(1100));
    uint8_t s2 = read_seconds();

    bool ticking = (s2 != s1);
    char msg[64];
    snprintf(msg, sizeof(msg), "sec[0]=%u  sec[1]=%u\n%s",
             s1, s2, ticking ? "Counter incremented" : "Counter NOT ticking!");
    _update_screen(msg, ticking ? Result::PASS : Result::FAIL);
    vTaskDelay(pdMS_TO_TICKS(1500));
    return ticking ? Result::PASS : Result::FAIL;
}

FactoryTest::Result FactoryTest::_test_camera()
{
    _show_screen(8, "Camera", "Initializing...", Result::SKIP);

    // Use esp_camera directly (available in Arduino ESP32 framework).
    camera_config_t cfg = {};
    cfg.ledc_channel = (ledc_channel_t)CAM_LEDC_CH;
    cfg.ledc_timer   = (ledc_timer_t)CAM_LEDC_TIMER;
    cfg.pin_d0  = CAM_D0_PIN;  cfg.pin_d1 = CAM_D1_PIN;
    cfg.pin_d2  = CAM_D2_PIN;  cfg.pin_d3 = CAM_D3_PIN;
    cfg.pin_d4  = CAM_D4_PIN;  cfg.pin_d5 = CAM_D5_PIN;
    cfg.pin_d6  = CAM_D6_PIN;  cfg.pin_d7 = CAM_D7_PIN;
    cfg.pin_xclk  = CAM_XCLK_PIN;
    cfg.pin_pclk  = CAM_PCLK_PIN;
    cfg.pin_vsync = CAM_VSYNC_PIN;
    cfg.pin_href  = CAM_HREF_PIN;
    cfg.pin_pwdn  = CAM_PWDN_PIN;
    cfg.pin_reset = CAM_RESET_PIN;
    cfg.xclk_freq_hz = 20000000;
    cfg.pixel_format = PIXFORMAT_JPEG;
    cfg.frame_size   = FRAMESIZE_QVGA;   // 320×240
    cfg.jpeg_quality = 12;
    cfg.fb_count     = 1;
    cfg.fb_location  = CAMERA_FB_IN_PSRAM;
    cfg.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;

    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        char msg[64];
        snprintf(msg, sizeof(msg), "Init failed: 0x%X", err);
        _update_screen(msg, Result::FAIL);
        vTaskDelay(pdMS_TO_TICKS(2000));
        return Result::FAIL;
    }

    _update_screen("Capturing frame...", Result::SKIP);
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb || fb->len == 0) {
        _update_screen("Frame capture failed", Result::FAIL);
        esp_camera_deinit();
        vTaskDelay(pdMS_TO_TICKS(2000));
        return Result::FAIL;
    }

    char msg[80];
    snprintf(msg, sizeof(msg), "Frame OK\n%d x %d  %u bytes",
             fb->width, fb->height, fb->len);
    esp_camera_fb_return(fb);
    esp_camera_deinit();

    _update_screen(msg, Result::PASS);
    vTaskDelay(pdMS_TO_TICKS(2000));
    return Result::PASS;
}

FactoryTest::Result FactoryTest::_test_audio()
{
    _show_screen(9, "Audio (ES8311)", "Probing I2C...", Result::SKIP);

    if (!i2c_probe(ES8311_I2C_ADDR)) {
        _update_screen("No ACK at 0x18\n(ES8311 not found)", Result::FAIL);
        vTaskDelay(pdMS_TO_TICKS(2000));
        return Result::FAIL;
    }

    // Read chip ID registers (0xFD / 0xFE) — ES8311 returns 0x83 / 0x11.
    Wire.beginTransmission(ES8311_I2C_ADDR);
    Wire.write(0xFD);
    Wire.endTransmission(false);
    Wire.requestFrom(ES8311_I2C_ADDR, (uint8_t)2);
    uint8_t id0 = Wire.available() ? Wire.read() : 0xFF;
    uint8_t id1 = Wire.available() ? Wire.read() : 0xFF;

    bool ok = (id0 == 0x83 && id1 == 0x11);
    char msg[64];
    snprintf(msg, sizeof(msg), "ACK at 0x%02X\nChipID = 0x%02X%02X%s",
             ES8311_I2C_ADDR, id0, id1, ok ? " (OK)" : " (unexpected)");
    _update_screen(msg, ok ? Result::PASS : Result::FAIL);
    vTaskDelay(pdMS_TO_TICKS(2000));
    return ok ? Result::PASS : Result::FAIL;
}
