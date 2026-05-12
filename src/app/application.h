#pragma once
#include <Arduino.h>
#include "task_config.h"

// NMDisplay28App — application singleton.
//   init()  : initialise all hardware; called from setup().
//   begin() : start FreeRTOS tasks; called after init() succeeds.
class NMDisplay28App {
public:
    static NMDisplay28App& instance();

    bool init();    // Phase 1: BSP init. Phase 2+: LVGL and peripheral drivers.
    void begin();   // Launch FreeRTOS tasks. (stub in Phase 1)

private:
    NMDisplay28App()  = default;
    NMDisplay28App(const NMDisplay28App&) = delete;
    NMDisplay28App& operator=(const NMDisplay28App&) = delete;
};
