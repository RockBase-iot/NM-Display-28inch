#pragma once

// Factory test sequencer.
// Runs a series of hardware test cases sequentially using LVGL screens.
// Each test case is a separate function that returns a Result (PASS / FAIL / SKIP).
// After all tests complete, a summary screen is shown.
//
// Usage:
//   FactoryTest *ft = new FactoryTest(Board::GetInstance());
//   ft->start();   // spawns a FreeRTOS task and returns immediately

#include "../../drivers/devices/hal.h"
#include <lvgl.h>

class FactoryTest {
public:
    explicit FactoryTest(Board &board);

    // Spawn the factory test FreeRTOS task.  Returns immediately.
    void start();

    // Individual test result.
    enum class Result : uint8_t { PASS, FAIL, SKIP };

private:
    // FreeRTOS task entry point.
    static void _task_entry(void *arg);

    // Runs all test cases in sequence, then shows the summary.
    void _run_all();

    // ── Test cases ────────────────────────────────────────────────────────
    // Each function draws a test screen, runs the test, updates the screen
    // with the result, waits briefly, and returns the result.
    Result _test_display();   // Color fill: R / G / B / white / black
    Result _test_touch();     // Tap 3 points anywhere on screen
    Result _test_sdcard();    // Mount SD → write / read test file
    Result _test_wifi();      // Scan for ≥ 1 access point
    Result _test_imu();       // QMI8658: I2C ack + non-zero accelerometer
    Result _test_pmu();       // AXP2101: I2C ack + valid voltage reading
    Result _test_rtc();       // PCF85063: I2C ack + second register increments
    Result _test_camera();    // Camera init + capture one non-black frame
    Result _test_audio();     // ES8311: I2C ack + register write without NAK

    // ── LVGL screen helpers ───────────────────────────────────────────────

    // Show a full-screen test status page.
    //   index   : 1-based test index (shown in title bar, e.g. "Test 3/9")
    //   name    : short test name,  e.g. "SD Card"
    //   body    : multiline status text updated as the test progresses
    //   result  : PASS = green badge, FAIL = red, SKIP = grey; use SKIP while running
    void _show_screen(uint8_t index, const char *name, const char *body, Result result);

    // Update only the body text on the current screen without re-creating it.
    // Call this while a test is in progress.
    void _update_screen(const char *body, Result result);

    // Block until the user taps the screen (max wait_ms milliseconds).
    // Returns true if a tap was detected, false if timed out.
    bool _wait_touch(uint32_t wait_ms);

    // Add Ok / Failed LVGL buttons to the current screen and block until
    // the user presses one.  Returns true = Ok (PASS), false = Failed (FAIL).
    bool _wait_verdict();

    // Show the final summary and block until the user taps.
    void _show_summary(int pass, int fail, int skip,
                       const char fail_names[][32], int fail_count);

    // ── LVGL button event callbacks (static so they can access private _verdict) ─
    static void _on_ok_btn(lv_event_t *e);
    static void _on_fail_btn(lv_event_t *e);

    // ── State ─────────────────────────────────────────────────────────────
    Board       &_board;
    TaskHandle_t _task_handle    = nullptr;

    // Verdict set by button callbacks; -1 = waiting, 0 = Failed, 1 = Ok.
    volatile int8_t _verdict = -1;

    // Cached LVGL widgets from the current test screen (updated by _update_screen).
    void        *_lv_body_label  = nullptr;   // lv_obj_t* cast to void*
    void        *_lv_badge_label = nullptr;   // unused; kept for ABI compat
};
