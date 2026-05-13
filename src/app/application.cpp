#include "application.h"
#include "../drivers/devices/hal.h"
#include "../drivers/display/lvgl_port.h"
#include "../utils/logger.h"
#include "../ui/boot_anim/boot_anim.h"
#include "../ui/factory_test/factory_test.h"
#include <SPIFFS.h>
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

    // Backlight stays OFF (0%) — boot_anim will fade it in after the first GIF
    // frame is on screen, giving a clean "gradual power-on" effect with no
    // visible blank time.  Meanwhile load a solid-black LVGL screen so the
    // display controller shows a defined state even before the GIF starts.
    if (LVGL_LOCK(200)) {
        lv_obj_t *black_scr = lv_obj_create(nullptr);
        lv_obj_set_style_bg_color(black_scr, lv_color_hex(0x000000), 0);
        lv_obj_set_style_bg_opa(black_scr, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(black_scr, 0, 0);
        lv_scr_load(black_scr);
        LVGL_UNLOCK();
    }

    // Mount SPIFFS so boot.gif is accessible before begin() is called.
    if (!SPIFFS.begin(true)) {
        LOG_W("SPIFFS mount failed — boot animation will be skipped");
    } else {
        LOG_I("SPIFFS: total=%u  used=%u bytes",
              SPIFFS.totalBytes(), SPIFFS.usedBytes());
    }

    // TODO Phase 3: axp2101 / qmi8658 / pcf85063 port init
    // TODO Phase 4: sdcard / wifi / button port init
    // TODO Phase 5: camera_port_init()
    // TODO Phase 7: es8311_port_init()

    return true;
}

void NMDisplay28App::begin() {
    Board   &board = Board::GetInstance();
    Display *disp  = board.get_display();
    Touch   *touch = board.get_touch();

    // BootAnim and FactoryTest are heap-allocated; they live until device restart.
    BootAnim    *boot_anim    = new BootAnim(disp, touch, "/boot.gif");
    FactoryTest *factory_test = new FactoryTest(board);

    // When the user taps the screen the animation exits and factory test starts.
    boot_anim->start([factory_test]() {
        LOG_I("Boot animation exited — starting factory test");
        factory_test->start();
    });
}
