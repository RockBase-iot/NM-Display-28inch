#pragma once

// LVGL port for NM-Display-28inch.
// Initializes LVGL 8.x with PSRAM frame buffers, registers the SPI display
// driver and (optionally) the I2C touch driver, then starts the LVGL timer-
// handler task.

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "hal.h"
#include "../touch/hal.h"

// Global mutex — must be held while calling any lv_* API from application code.
extern SemaphoreHandle_t g_lvgl_mutex;

// Take the LVGL mutex.  Returns true if acquired within timeout_ms.
#define LVGL_LOCK(timeout_ms)  (xSemaphoreTake(g_lvgl_mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE)

// Release the LVGL mutex.
#define LVGL_UNLOCK()          xSemaphoreGive(g_lvgl_mutex)


class LvglPort {
public:
    // Initialise LVGL, allocate PSRAM frame buffers, register display and
    // touch input drivers, and start the LVGL timer-handler FreeRTOS task.
    //
    //   disp  : fully initialised Display pointer (not null).
    //   touch : IICTouch / SPITouch pointer, or nullptr if no touch.
    //   width, height : logical display dimensions in pixels.
    //
    // Returns false if PSRAM allocation fails or LVGL init fails.
    static bool init(Display *disp, Touch *touch, uint16_t width, uint16_t height);

    static void deinit();

private:
    LvglPort() = delete;
};
