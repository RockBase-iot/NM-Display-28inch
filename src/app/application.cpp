#include "application.h"
#include "../drivers/devices/hal.h"
#include "../drivers/display/lvgl_port.h"
#include "../utils/logger.h"
#include <lvgl.h>

NMDisplay28App& NMDisplay28App::instance() {
    static NMDisplay28App app;
    return app;
}

bool NMDisplay28App::init() {
    Board& board = Board::GetInstance();
    board.init();   // Serial.begin(), Wire.begin(), TCA9554 LCD reset, SPI2 start

    LOG_I("Board: %s", board.get_board_model().c_str());
    LOG_I("Chip:  ESP32-S3 @ %u MHz  flash=%uMB  psram=%uMB",
          getCpuFrequencyMhz(),
          ESP.getFlashChipSize() / (1024 * 1024),
          ESP.getPsramSize()     / (1024 * 1024));

    // Phase 2: display + touch + LVGL
    Display *disp  = board.get_display();  // ST7789 init + 270° rotate
    Touch   *touch = board.get_touch();    // FT6336 init (may be nullptr on failure)

    if (!disp) {
        LOG_E("Display init failed — halting");
        while (true) delay(1000);
    }

    if (!LvglPort::init(disp, touch, disp->width(), disp->height())) {
        LOG_E("LvglPort init failed — halting");
        while (true) delay(1000);
    }

    // Turn on backlight once LVGL is ready
    disp->blctrl(0.85f);

    // TODO Phase 3: axp2101 / qmi8658 / pcf85063 port init
    // TODO Phase 4: sdcard / wifi / button port init
    // TODO Phase 5: camera_port_init()
    // TODO Phase 7: es8311_port_init()

    return true;
}

void NMDisplay28App::begin() {
    // Create the placeholder Hello World screen under LVGL mutex.
    if (LVGL_LOCK(500)) {
        lv_obj_t *scr = lv_scr_act();
        lv_obj_set_style_bg_color(scr, lv_color_hex(0x1A1A2E), LV_PART_MAIN);

        lv_obj_t *label = lv_label_create(scr);
        lv_label_set_text(label, "Hello, World!");
        lv_obj_set_style_text_color(label, lv_color_hex(0xE0E0FF), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_center(label);

        LVGL_UNLOCK();
    }

    // Heartbeat logger task (runs until real tasks replace it)
    xTaskCreatePinnedToCore(
        [](void*) {
            uint32_t tick = 0;
            while (true) {
                LOG_I("[heartbeat] tick=%u  heap=%u  psram=%u",
                      tick++,
                      ESP.getFreeHeap(),
                      ESP.getFreePsram());
                vTaskDelay(pdMS_TO_TICKS(5000));
            }
        },
        "heartbeat", TASK_STACK_SENSOR, nullptr, TASK_PRIORITY_IDLE + 1, nullptr, CORE_0
    );
}
