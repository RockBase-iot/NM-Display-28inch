#include "lvgl_port.h"
#include "../../app/task_config.h"
#include "../../utils/logger.h"

#include <lvgl.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_heap_caps.h>

// ─── Module-private state ────────────────────────────────────────────────────

SemaphoreHandle_t g_lvgl_mutex = nullptr;

static lv_disp_draw_buf_t _draw_buf;
static lv_disp_drv_t      _disp_drv;
static lv_indev_drv_t     _indev_drv;

// ─── Flush callback ─────────────────────────────────────────────────────────

static void _disp_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    Display *disp = static_cast<Display *>(drv->user_data);
    uint32_t w = area->x2 - area->x1 + 1;
    uint32_t h = area->y2 - area->y1 + 1;

    disp->startWrite();
    disp->setAddrWindow(area->x1, area->y1, w, h);
    disp->pushColors(reinterpret_cast<uint16_t *>(color_map), w * h, true);
    disp->endWrite();

    lv_disp_flush_ready(drv);
}

// ─── Touch read callback ─────────────────────────────────────────────────────

static void _touch_read(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    Touch *touch = static_cast<Touch *>(drv->user_data);
    touch_point_t pt;
    touch->read(&pt);

    if (pt.pressed) {
        data->point.x = pt.x;
        data->point.y = pt.y;
        data->state   = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

// ─── LVGL timer-handler task ─────────────────────────────────────────────────

static void _lvgl_task(void * /*arg*/)
{
    while (true) {
        if (xSemaphoreTake(g_lvgl_mutex, portMAX_DELAY) == pdTRUE) {
            lv_timer_handler();
            xSemaphoreGive(g_lvgl_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// ─── Public API ──────────────────────────────────────────────────────────────

bool LvglPort::init(Display *disp, Touch *touch, uint16_t width, uint16_t height)
{
    if (!disp) {
        LOG_E("LvglPort::init — display pointer is null");
        return false;
    }

    // Allocate two PSRAM frame buffers (1/4 screen each → double-buffer)
    const uint32_t buf_pixels = (uint32_t)width * height / 4;
    const uint32_t buf_bytes  = buf_pixels * sizeof(lv_color_t);

    lv_color_t *buf1 = static_cast<lv_color_t *>(
        heap_caps_malloc(buf_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    lv_color_t *buf2 = static_cast<lv_color_t *>(
        heap_caps_malloc(buf_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));

    if (!buf1 || !buf2) {
        LOG_E("LvglPort::init — PSRAM frame buffer allocation failed");
        heap_caps_free(buf1);
        heap_caps_free(buf2);
        return false;
    }

    LOG_I("LvglPort: buffers %u bytes × 2 in PSRAM", buf_bytes);

    // Initialise LVGL core
    lv_init();

    // Create mutex before starting the task
    g_lvgl_mutex = xSemaphoreCreateMutex();
    if (!g_lvgl_mutex) {
        LOG_E("LvglPort::init — mutex creation failed");
        return false;
    }

    // Register display driver
    lv_disp_draw_buf_init(&_draw_buf, buf1, buf2, buf_pixels);

    lv_disp_drv_init(&_disp_drv);
    _disp_drv.hor_res   = (lv_coord_t)width;
    _disp_drv.ver_res   = (lv_coord_t)height;
    _disp_drv.flush_cb  = _disp_flush;
    _disp_drv.draw_buf  = &_draw_buf;
    _disp_drv.user_data = disp;
    lv_disp_drv_register(&_disp_drv);

    // Register touch input driver (optional)
    if (touch) {
        lv_indev_drv_init(&_indev_drv);
        _indev_drv.type      = LV_INDEV_TYPE_POINTER;
        _indev_drv.read_cb   = _touch_read;
        _indev_drv.user_data = touch;
        lv_indev_drv_register(&_indev_drv);
        LOG_I("LvglPort: touch input registered");
    }

    // Start LVGL timer-handler task
    // LV_TICK_CUSTOM = 1 uses millis() automatically — no tick task needed.
    xTaskCreatePinnedToCore(
        _lvgl_task,
        "lvgl",
        TASK_STACK_LVGL,
        nullptr,
        TASK_PRIORITY_LVGL_DRV,
        nullptr,
        LvglTaskCore);

    LOG_I("LvglPort: initialised  %u×%u", width, height);
    return true;
}

void LvglPort::deinit()
{
    // Not implemented — a restart is preferred on ESP32
}
