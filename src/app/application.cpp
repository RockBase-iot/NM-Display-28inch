#include "application.h"
#include "../drivers/devices/hal.h"
#include "../utils/logger.h"

NMDisplay28App& NMDisplay28App::instance() {
    static NMDisplay28App app;
    return app;
}

bool NMDisplay28App::init() {
    Board& board = Board::GetInstance();
    board.init();   // Serial.begin() is called inside

    LOG_I("Board: %s", board.get_board_model().c_str());
    LOG_I("Chip:  ESP32-S3 @ %u MHz  flash=%uMB  psram=%uMB",
          getCpuFrequencyMhz(),
          ESP.getFlashChipSize() / (1024 * 1024),
          ESP.getPsramSize()     / (1024 * 1024));

    // TODO Phase 2: LvglPort::init(board.get_display(), board.get_touch(), ...)
    // TODO Phase 3: axp2101 / qmi8658 / pcf85063 port init
    // TODO Phase 4: sdcard / wifi / button port init
    // TODO Phase 5: camera_port_init()
    // TODO Phase 7: es8311_port_init()

    return true;
}

void NMDisplay28App::begin() {
    // TODO Phase 2+: conditionally launch tasks based on profile.has_xxx.
    // Example (uncomment in Phase 5):
    // const BoardProfile& prof = Board::GetInstance().get_profile();
    // if (prof.has_camera) {
    //     xTaskCreatePinnedToCore(camera_task, "cam_task",
    //                             TASK_STACK_CAMERA, nullptr,
    //                             TASK_PRIORITY_CAMERA, nullptr, CameraTaskCore);
    // }

    // Placeholder heartbeat task — remove once real tasks are running.
    xTaskCreatePinnedToCore(
        [](void*) {
            uint32_t tick = 0;
            while (true) {
                LOG_I("[heartbeat] tick=%u  heap=%u  psram=%u",
                      tick++,
                      ESP.getFreeHeap(),
                      ESP.getFreePsram());
                vTaskDelay(pdMS_TO_TICKS(2000));
            }
        },
        "heartbeat", TASK_STACK_SENSOR, nullptr, TASK_PRIORITY_IDLE + 1, nullptr, CORE_0
    );
}
