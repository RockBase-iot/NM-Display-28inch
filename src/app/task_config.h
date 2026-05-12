#pragma once
#include <freertos/FreeRTOS.h>

// Core assignment
#define CORE_0  0
#define CORE_1  1

#define LvglTaskCore        CORE_0   // LVGL tick — highest UI priority
#define CameraTaskCore      CORE_1   // Camera frame fetch — isolated to Core 1
#define AudioTaskCore       CORE_0
#define SensorTaskCore      CORE_0
#define WifiTaskCore        CORE_0
#define BootAnimTaskCore    CORE_1   // Boot animation — isolated from LVGL on Core 0
#define FactoryTestTaskCore CORE_0

// Task priorities (higher number = higher priority)
enum TaskPriority {
    TASK_PRIORITY_IDLE = 0,
    TASK_PRIORITY_SENSOR,       // IMU / RTC polling
    TASK_PRIORITY_AUDIO,        // ES8311 audio
    TASK_PRIORITY_WIFI,         // WiFi scan / connect
    TASK_PRIORITY_BTN,          // Button debounce
    TASK_PRIORITY_CAMERA,       // Camera frame decode
    TASK_PRIORITY_BOOT_ANIM,    // Boot animation — higher than camera, below LVGL tick
    TASK_PRIORITY_LVGL_DRV,     // LVGL tick — must maintain 5 ms period
};

// Task stack sizes (bytes)
#define TASK_STACK_LVGL         8192
#define TASK_STACK_CAMERA       16384   // Frame buffer + LVGL image update
#define TASK_STACK_AUDIO        8192
#define TASK_STACK_SENSOR       4096
#define TASK_STACK_WIFI         8192
#define TASK_STACK_BTN          2048
#define TASK_STACK_BOOT_ANIM    32768   // AnimatedGIF LZW decoder needs >16 KB stack
#define TASK_STACK_FACTORY_TEST 8192
