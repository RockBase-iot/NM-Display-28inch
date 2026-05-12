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

// ─── Verdict button callbacks ─────────────────────────────────────────────────

void FactoryTest::_on_ok_btn(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    static_cast<FactoryTest *>(lv_event_get_user_data(e))->_verdict = 1;
}

void FactoryTest::_on_fail_btn(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    static_cast<FactoryTest *>(lv_event_get_user_data(e))->_verdict = 0;
}

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

void FactoryTest::_show_screen(uint8_t index, const char *name,
                                const char *body, Result /*result*/)
{
    if (!LVGL_LOCK(500)) return;

    lv_obj_t *scr = lv_obj_create(nullptr);
    lv_obj_set_style_bg_color(scr, lv_color_hex(COLOR_BG), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(scr, 0, 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    // ── Title bar ─────────────────────────────────────────────────────────────
    lv_obj_t *title_bar = lv_obj_create(scr);
    lv_obj_set_size(title_bar, SCREEN_WIDTH, 40);
    lv_obj_align(title_bar, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(title_bar, lv_color_hex(COLOR_TITLE_BG), 0);
    lv_obj_set_style_border_width(title_bar, 0, 0);
    lv_obj_set_style_radius(title_bar, 0, 0);
    lv_obj_set_style_pad_all(title_bar, 4, 0);
    lv_obj_clear_flag(title_bar, LV_OBJ_FLAG_SCROLLABLE);

    char title_buf[48];
    snprintf(title_buf, sizeof(title_buf), "Test %d/9: %s", index, name);
    lv_obj_t *title_lbl = lv_label_create(title_bar);
    lv_label_set_text(title_lbl, title_buf);
    lv_obj_set_style_text_color(title_lbl, lv_color_hex(COLOR_TEXT), 0);
    // Bold-weight title: use Montserrat 16 (larger = visually bold on small screen)
    lv_obj_set_style_text_font(title_lbl, &lv_font_montserrat_16, 0);
    lv_obj_center(title_lbl);

    // ── Body text (monospace, aligned output) ──────────────────────────────────
    lv_obj_t *body_lbl = lv_label_create(scr);
    lv_label_set_text(body_lbl, body);
    lv_obj_set_style_text_color(body_lbl, lv_color_hex(COLOR_SUBTEXT), 0);
    // Monospace font for aligned technical output
    lv_obj_set_style_text_font(body_lbl, &lv_font_unscii_8, 0);
    lv_obj_set_width(body_lbl, SCREEN_WIDTH - 16);
    lv_label_set_long_mode(body_lbl, LV_LABEL_LONG_WRAP);
    // Body area: title=40px, bottom buttons=54px  ->  y_start=44, max_h=142px
    lv_obj_align(body_lbl, LV_ALIGN_TOP_LEFT, 8, 44);

    // Cache body widget pointer for live updates (_lv_badge_label unused)
    _lv_body_label  = body_lbl;
    _lv_badge_label = nullptr;

    lv_scr_load(scr);
    LVGL_UNLOCK();
}

void FactoryTest::_update_screen(const char *body, Result /*result*/)
{
    if (!_lv_body_label) return;
    if (!LVGL_LOCK(100)) return;
    lv_label_set_text(static_cast<lv_obj_t *>(_lv_body_label), body);
    LVGL_UNLOCK();
}

bool FactoryTest::_wait_verdict()
{
    _verdict = -1;

    if (!LVGL_LOCK(500)) return false;

    lv_obj_t *scr = lv_scr_act();

    // ── Ok button (green, left) ────────────────────────────────────────────────
    lv_obj_t *ok_btn = lv_btn_create(scr);
    lv_obj_set_size(ok_btn, 142, 44);
    lv_obj_align(ok_btn, LV_ALIGN_BOTTOM_LEFT, 8, -6);
    lv_obj_set_style_bg_color(ok_btn, lv_color_hex(COLOR_PASS), 0);
    lv_obj_set_style_bg_color(ok_btn, lv_color_hex(0x27AE60), LV_STATE_PRESSED);
    lv_obj_set_style_border_width(ok_btn, 0, 0);
    lv_obj_set_style_radius(ok_btn, 6, 0);
    lv_obj_add_event_cb(ok_btn, _on_ok_btn, LV_EVENT_CLICKED, this);
    lv_obj_t *ok_lbl = lv_label_create(ok_btn);
    lv_label_set_text(ok_lbl, LV_SYMBOL_OK "  Ok");
    lv_obj_set_style_text_color(ok_lbl, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(ok_lbl);

    // ── Failed button (red, right) ─────────────────────────────────────────────
    lv_obj_t *fail_btn = lv_btn_create(scr);
    lv_obj_set_size(fail_btn, 142, 44);
    lv_obj_align(fail_btn, LV_ALIGN_BOTTOM_RIGHT, -8, -6);
    lv_obj_set_style_bg_color(fail_btn, lv_color_hex(COLOR_FAIL), 0);
    lv_obj_set_style_bg_color(fail_btn, lv_color_hex(0xC0392B), LV_STATE_PRESSED);
    lv_obj_set_style_border_width(fail_btn, 0, 0);
    lv_obj_set_style_radius(fail_btn, 6, 0);
    lv_obj_add_event_cb(fail_btn, _on_fail_btn, LV_EVENT_CLICKED, this);
    lv_obj_t *fail_lbl = lv_label_create(fail_btn);
    lv_label_set_text(fail_lbl, LV_SYMBOL_CLOSE "  Failed");
    lv_obj_set_style_text_color(fail_lbl, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(fail_lbl);

    LVGL_UNLOCK();

    // Block FreeRTOS task until user presses a button
    while (_verdict < 0) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return (_verdict == 1);
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
                  "PASS  %-3d\nFAIL  %-3d\nSKIP  %-3d\n", pass, fail, skip);
    if (fail_count > 0) {
        n += snprintf(body + n, (int)sizeof(body) - n, "\nFailed items:");
        for (int i = 0; i < fail_count && n < (int)sizeof(body) - 20; i++) {
            n += snprintf(body + n, (int)sizeof(body) - n, "\n  - %s", fail_names[i]);
        }
    }

    if (!LVGL_LOCK(500)) return;

    lv_obj_t *scr = lv_obj_create(nullptr);
    lv_obj_set_style_bg_color(scr, lv_color_hex(COLOR_BG), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(scr, 0, 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    // Bold title (Montserrat 16)
    lv_obj_t *title_lbl = lv_label_create(scr);
    lv_label_set_text(title_lbl, "Factory Test Complete");
    lv_obj_set_style_text_color(title_lbl, lv_color_hex(COLOR_TEXT), 0);
    lv_obj_set_style_text_font(title_lbl, &lv_font_montserrat_16, 0);
    lv_obj_align(title_lbl, LV_ALIGN_TOP_MID, 0, 8);

    // Monospace body (UNSCII 8)
    lv_obj_t *body_lbl = lv_label_create(scr);
    lv_label_set_text(body_lbl, body);
    lv_obj_set_style_text_color(body_lbl, lv_color_hex(COLOR_SUBTEXT), 0);
    lv_obj_set_style_text_font(body_lbl, &lv_font_unscii_8, 0);
    lv_obj_set_width(body_lbl, SCREEN_WIDTH - 16);
    lv_label_set_long_mode(body_lbl, LV_LABEL_LONG_WRAP);
    lv_obj_align(body_lbl, LV_ALIGN_TOP_LEFT, 8, 36);

    // Overall verdict badge
    lv_obj_t *verdict = lv_label_create(scr);
    lv_label_set_text(verdict,
        overall_pass ? LV_SYMBOL_OK "  OVERALL: PASS" : LV_SYMBOL_CLOSE "  OVERALL: FAIL");
    lv_obj_set_style_text_color(verdict,
        lv_color_hex(overall_pass ? COLOR_PASS : COLOR_FAIL), 0);
    lv_obj_set_style_text_font(verdict, &lv_font_montserrat_16, 0);
    lv_obj_align(verdict, LV_ALIGN_BOTTOM_MID, 0, -34);

    lv_obj_t *hint = lv_label_create(scr);
    lv_label_set_text(hint, "tap anywhere to restart");
    lv_obj_set_style_text_color(hint, lv_color_hex(COLOR_SKIP), 0);
    lv_obj_set_style_text_font(hint, &lv_font_unscii_8, 0);
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
        _update_screen("No display driver!", Result::FAIL);
        return _wait_verdict() ? Result::PASS : Result::FAIL;
    }

    for (auto &s : steps) {
        // Full-screen solid-colour overlay for visual check
        lv_obj_t *overlay = nullptr;
        if (LVGL_LOCK(200)) {
            overlay = lv_obj_create(lv_scr_act());
            lv_obj_set_size(overlay, SCREEN_WIDTH, SCREEN_HEIGHT);
            lv_obj_set_pos(overlay, 0, 0);
            lv_obj_set_style_bg_color(overlay, lv_color_hex(s.color), 0);
            lv_obj_set_style_bg_opa(overlay, LV_OPA_COVER, 0);
            lv_obj_set_style_border_width(overlay, 0, 0);
            lv_obj_set_style_radius(overlay, 0, 0);
            lv_obj_set_style_pad_all(overlay, 0, 0);
            // Small colour-name label in bottom-right corner
            uint32_t txt_col = (s.color <= 0x0000FF || s.color == 0x000000) ? 0xFFFFFF : 0x000000;
            lv_obj_t *name_lbl = lv_label_create(overlay);
            lv_label_set_text(name_lbl, s.name);
            lv_obj_set_style_text_color(name_lbl, lv_color_hex(txt_col), 0);
            lv_obj_set_style_text_font(name_lbl, &lv_font_montserrat_16, 0);
            lv_obj_align(name_lbl, LV_ALIGN_BOTTOM_RIGHT, -8, -6);
            LVGL_UNLOCK();
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (overlay && LVGL_LOCK(200)) {
            lv_obj_del(overlay);
            LVGL_UNLOCK();
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    _update_screen("Red / Green / Blue\nWhite / Black shown.\n\n"
                   "Did all 5 colors look\ncorrect?", Result::SKIP);
    return _wait_verdict() ? Result::PASS : Result::FAIL;
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
        _update_screen("Taps detected  : 3/3\nResult         : OK", Result::PASS);
    } else {
        snprintf(msg, sizeof(msg),
                 "Taps detected  : %d/3\nResult         : TIMEOUT", taps);
        _update_screen(msg, Result::FAIL);
    }
    return _wait_verdict() ? Result::PASS : Result::FAIL;
}

FactoryTest::Result FactoryTest::_test_sdcard()
{
    _show_screen(3, "SD Card", "Mounting...", Result::SKIP);

    SD_MMC.setPins(SD_CLK_PIN, SD_CMD_PIN, SD_D0_PIN);
    if (!SD_MMC.begin("/sdcard", true)) {   // true = 1-bit mode
        _update_screen("Mount           FAIL\n(SD inserted?)", Result::FAIL);
        return _wait_verdict() ? Result::PASS : Result::FAIL;
    }

    const char *test_path = "/sdcard/factory_test.tmp";
    const char *test_data = "NMDisplay28-FactoryTest-OK";

    _update_screen("Mount           OK\nWrite           ...", Result::SKIP);
    File f = SD_MMC.open(test_path, FILE_WRITE);
    if (!f) {
        _update_screen("Mount           OK\nWrite open      FAIL", Result::FAIL);
        SD_MMC.end();
        return _wait_verdict() ? Result::PASS : Result::FAIL;
    }
    f.print(test_data);
    f.close();

    _update_screen("Mount           OK\nWrite           OK\nRead            ...", Result::SKIP);
    f = SD_MMC.open(test_path, FILE_READ);
    if (!f) {
        _update_screen("Mount           OK\nWrite           OK\nRead open       FAIL", Result::FAIL);
        SD_MMC.end();
        return _wait_verdict() ? Result::PASS : Result::FAIL;
    }
    char read_buf[64] = {};
    f.readBytes(read_buf, sizeof(read_buf) - 1);
    f.close();
    SD_MMC.remove(test_path);
    SD_MMC.end();

    bool match = (strcmp(read_buf, test_data) == 0);
    _update_screen(
        match ? "Mount           OK\nWrite           OK\nRead            OK\nContent match   OK"
              : "Mount           OK\nWrite           OK\nRead            OK\nContent match   FAIL",
        match ? Result::PASS : Result::FAIL);
    return _wait_verdict() ? Result::PASS : Result::FAIL;
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
        _update_screen("Scan result     FAIL\nNo APs found", Result::FAIL);
        return _wait_verdict() ? Result::PASS : Result::FAIL;
    }

    char msg[192];
    int  written = snprintf(msg, sizeof(msg), "Scan result     OK\nFound %-3d APs:\n", n);
    for (int i = 0; i < n && i < 5 && written < (int)sizeof(msg) - 36; i++) {
        written += snprintf(msg + written, sizeof(msg) - written,
                            "  %-16.16s %ddBm\n",
                            WiFi.SSID(i).c_str(), WiFi.RSSI(i));
    }
    WiFi.scanDelete();

    _update_screen(msg, Result::PASS);
    return _wait_verdict() ? Result::PASS : Result::FAIL;
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
        _update_screen("I2C probe 0x6B  FAIL\nNo ACK", Result::FAIL);
        return _wait_verdict() ? Result::PASS : Result::FAIL;
    }

    // Read WHO_AM_I (register 0x00) — QMI8658A returns 0x05.
    Wire.beginTransmission(QMI8658_ADDR);
    Wire.write(0x00);
    Wire.endTransmission(false);
    Wire.requestFrom(QMI8658_ADDR, (uint8_t)1);
    uint8_t who = Wire.available() ? Wire.read() : 0xFF;
    bool ok = (who == 0x05);

    char msg[96];
    snprintf(msg, sizeof(msg),
             "I2C probe 0x6B  OK\n"
             "WHO_AM_I  0x%02X  %s\n"
             "Expected  0x05  %s",
             who, ok ? "OK" : "!!", ok ? "match" : "mismatch");
    _update_screen(msg, ok ? Result::PASS : Result::FAIL);
    return _wait_verdict() ? Result::PASS : Result::FAIL;
}

FactoryTest::Result FactoryTest::_test_pmu()
{
    _show_screen(6, "PMU (AXP2101)", "Probing I2C...", Result::SKIP);

    if (!i2c_probe(AXP2101_I2C_ADDR)) {
        _update_screen("I2C probe 0x34  FAIL\nNo ACK", Result::FAIL);
        return _wait_verdict() ? Result::PASS : Result::FAIL;
    }

    // Read chip ID register (0x03) — AXP2101 returns 0x4A.
    Wire.beginTransmission(AXP2101_I2C_ADDR);
    Wire.write(0x03);
    Wire.endTransmission(false);
    Wire.requestFrom(AXP2101_I2C_ADDR, (uint8_t)1);
    uint8_t chip_id = Wire.available() ? Wire.read() : 0xFF;
    bool ok = (chip_id == 0x4A);

    char msg[96];
    snprintf(msg, sizeof(msg),
             "I2C probe 0x34  OK\n"
             "Chip ID   0x%02X  %s\n"
             "Expected  0x4A  %s",
             chip_id, ok ? "OK" : "!!", ok ? "match" : "mismatch");
    _update_screen(msg, ok ? Result::PASS : Result::FAIL);
    return _wait_verdict() ? Result::PASS : Result::FAIL;
}

FactoryTest::Result FactoryTest::_test_rtc()
{
    // PCF85063 I2C address: 0x51.
    constexpr uint8_t PCF85063_ADDR = 0x51;
    _show_screen(7, "RTC (PCF85063)", "Probing I2C...", Result::SKIP);

    if (!i2c_probe(PCF85063_ADDR)) {
        _update_screen("I2C probe 0x51  FAIL\nNo ACK", Result::FAIL);
        return _wait_verdict() ? Result::PASS : Result::FAIL;
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
    _update_screen("I2C probe 0x51  OK\nReading seconds...", Result::SKIP);
    vTaskDelay(pdMS_TO_TICKS(1100));
    uint8_t s2 = read_seconds();

    bool ticking = (s2 != s1);
    char msg[96];
    snprintf(msg, sizeof(msg),
             "I2C probe 0x51  OK\n"
             "sec[0]    %-3u\n"
             "sec[1]    %-3u\n"
             "Ticking   %s",
             s1, s2, ticking ? "YES  OK" : "NO   FAIL");
    _update_screen(msg, ticking ? Result::PASS : Result::FAIL);
    return _wait_verdict() ? Result::PASS : Result::FAIL;
}

FactoryTest::Result FactoryTest::_test_camera()
{
    // TODO: Camera hardware test not yet implemented.
    // Requires a physical OV2640 (or compatible) module connected to CAM_* pins.
    // TODO: Verify CAM_LEDC_TIMER / CAM_LEDC_CH don't conflict with backlight
    //       LEDC before enabling esp_camera_init() here.
    // TODO: Add frame-quality check: verify captured JPEG size > threshold
    //       and that the frame is not an all-black "dark frame".
    _show_screen(8, "Camera",
        "TODO: Camera test\n"
        "not implemented yet.\n"
        "\n"
        "Requires OV2640 module\n"
        "on CAM_* pins.\n"
        "\n"
        "Press Ok to SKIP.",
        Result::SKIP);
    _wait_verdict();
    return Result::SKIP;   // always SKIP until TODO resolved
}

FactoryTest::Result FactoryTest::_test_audio()
{
    _show_screen(9, "Audio (ES8311)", "Probing I2C 0x18...", Result::SKIP);

    if (!i2c_probe(ES8311_I2C_ADDR)) {
        _update_screen("I2C probe 0x18  FAIL\nES8311 not found", Result::FAIL);
        return _wait_verdict() ? Result::PASS : Result::FAIL;
    }

    // Read chip ID registers (0xFD / 0xFE) — ES8311 returns 0x83 / 0x11.
    Wire.beginTransmission(ES8311_I2C_ADDR);
    Wire.write(0xFD);
    Wire.endTransmission(false);
    Wire.requestFrom(ES8311_I2C_ADDR, (uint8_t)2);
    uint8_t id0 = Wire.available() ? Wire.read() : 0xFF;
    uint8_t id1 = Wire.available() ? Wire.read() : 0xFF;
    bool i2c_ok = (id0 == 0x83 && id1 == 0x11);

    // TODO: Actual audio I/O test not yet implemented.
    // TODO: Add I2S init + tone playback via ES8311 DAC and verify
    //       output level with an analog measurement or loopback.
    // TODO: Add DMIC / line-in recording test via ES8311 ADC.
    char msg[160];
    snprintf(msg, sizeof(msg),
             "I2C probe 0x18  OK\n"
             "ChipID[FD] 0x%02X  %s\n"
             "ChipID[FE] 0x%02X  %s\n"
             "\n"
             "TODO: I2S playback test\n"
             "(I2C probe only)",
             id0, (id0 == 0x83) ? "OK" : "!!",
             id1, (id1 == 0x11) ? "OK" : "!!");
    _update_screen(msg, i2c_ok ? Result::PASS : Result::FAIL);
    return _wait_verdict() ? Result::PASS : Result::FAIL;
}
