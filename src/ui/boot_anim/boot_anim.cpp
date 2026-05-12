#include "boot_anim.h"
#include "../../app/task_config.h"
#include "../../utils/logger.h"
#include "../../bsp/nm_display_28/config.h"
#include "../../drivers/display/lvgl_port.h"

#include <SPIFFS.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>

// ─── Module-level state shared with static callbacks ─────────────────────────

// Active frame buffer pointer — set before gif.open() so the draw callback can
// reach it without needing a non-static member pointer.
static uint16_t *s_frame_buf = nullptr;

// Active BootAnim instance — needed so _gif_draw can call member data.
static BootAnim *s_instance  = nullptr;

// SPIFFS file handle — one at a time is safe because only one GIF plays at a time.
static File s_gif_file;

// ─── Constructor / Destructor ────────────────────────────────────────────────

BootAnim::BootAnim(Display *disp, Touch *touch, const char *gif_path)
    : _disp(disp), _touch(touch), _gif_path(gif_path)
{}

BootAnim::~BootAnim()
{
    stop();
    // Frame buffer is freed inside _play_loop before the task deletes itself.
}

// ─── Public API ──────────────────────────────────────────────────────────────

void BootAnim::start(ExitCallback on_exit)
{
    if (_task_handle) return;   // already running

    _on_exit    = on_exit;
    _stop_flag  = false;

    // Allocate the full-screen PSRAM frame buffer once.
    _frame_buf = static_cast<uint16_t *>(
        heap_caps_malloc(SCREEN_WIDTH * SCREEN_HEIGHT * sizeof(uint16_t),
                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));

    if (!_frame_buf) {
        LOG_E("BootAnim: PSRAM frame buffer allocation failed — skipping animation");
        if (_on_exit) _on_exit();
        return;
    }

    memset(_frame_buf, 0, SCREEN_WIDTH * SCREEN_HEIGHT * sizeof(uint16_t));

    // Preload the entire GIF file into PSRAM so every frame is decoded from
    // fast PSRAM rather than slow SPIFFS (eliminates per-frame seek latency).
    {
        File f = SPIFFS.open(_gif_path, "r");
        if (f) {
            _gif_data_size = f.size();
            _gif_data = static_cast<uint8_t *>(
                heap_caps_malloc(_gif_data_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
            if (_gif_data) {
                f.read(_gif_data, _gif_data_size);
                LOG_I("BootAnim: GIF preloaded into PSRAM (%u bytes)", _gif_data_size);
            } else {
                LOG_W("BootAnim: PSRAM alloc for GIF data failed — falling back to SPIFFS");
                _gif_data_size = 0;
            }
            f.close();
        }
    }

    xTaskCreatePinnedToCore(
        _task_entry,
        "boot_anim",
        TASK_STACK_BOOT_ANIM,
        this,
        TASK_PRIORITY_BOOT_ANIM,
        &_task_handle,
        BootAnimTaskCore);
}

void BootAnim::stop()
{
    _stop_flag = true;
    // The task will exit on its next frame boundary and call on_exit itself.
}

// ─── FreeRTOS task ───────────────────────────────────────────────────────────

void BootAnim::_task_entry(void *arg)
{
    BootAnim *self = static_cast<BootAnim *>(arg);
    self->_play_loop();
    // _play_loop cleans up and invokes on_exit before returning.
    vTaskDelete(nullptr);
}

// ─── Playback loop ───────────────────────────────────────────────────────────

void BootAnim::_play_loop()
{
    s_instance  = this;
    s_frame_buf = _frame_buf;

    // _gif is a member variable — its ~8 KB internal LZW buffer lives on the
    // heap (with the BootAnim object), NOT on this task's stack.
    _gif.begin(GIF_PALETTE_RGB565_LE);   // 16-bit little-endian matches ST7789

    // Use memory callbacks if GIF was preloaded into PSRAM; otherwise SPIFFS.
    bool opened = _gif_data
        ? _gif.open(_gif_path, _gif_open_mem, _gif_close_mem, _gif_read_mem, _gif_seek_mem, _gif_draw)
        : _gif.open(_gif_path, _gif_open,     _gif_close,     _gif_read,     _gif_seek,     _gif_draw);

    if (!opened) {
        LOG_E("BootAnim: cannot open %s — skipping animation", _gif_path);
        if (_gif_data) { heap_caps_free(_gif_data); _gif_data = nullptr; }
        heap_caps_free(_frame_buf);
        _frame_buf  = nullptr;
        _task_handle = nullptr;
        if (_on_exit) _on_exit();
        return;
    }

    LOG_I("BootAnim: playing %s  (%d x %d)", _gif_path, _gif.getCanvasWidth(), _gif.getCanvasHeight());

    while (!_stop_flag) {
        // Record frame start time so decode+push cost is subtracted from the sleep.
        int64_t t_start = esp_timer_get_time();

        int delay_ms = 0;
        bool has_more = _gif.playFrame(false, &delay_ms);

        // Push the completed frame to the display.
        // Hold the LVGL mutex only for the duration of the SPI transfer so
        // the LVGL tick task is not starved for more than ~10 ms.
        if (LVGL_LOCK(30)) {
            _disp->startWrite();
            _disp->setAddrWindow(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT);
            _disp->pushColors(_frame_buf, SCREEN_WIDTH * SCREEN_HEIGHT, true);
            _disp->endWrite();
            LVGL_UNLOCK();
        }

        // Poll touch directly — LVGL's indev poll runs inside the LVGL task
        // and could be blocked by the mutex above, so we read the hardware directly.
        if (_touch) {
            touch_point_t tp;
            _touch->read(&tp);
            if (tp.pressed) {
                LOG_I("BootAnim: touch detected — exiting animation");
                _stop_flag = true;
            }
        }

        if (!has_more) {
            _gif.reset();    // loop: restart from the first frame
        }

        // Compute how long decode+push actually took, then sleep only the
        // remainder so the total frame time hits the target interval.
        // This prevents decode/push latency from stacking on top of the sleep.
        int target_ms = delay_ms > BOOT_ANIM_MAX_FRAME_MS ? BOOT_ANIM_MAX_FRAME_MS : delay_ms;
        int elapsed_ms = (int)((esp_timer_get_time() - t_start) / 1000);
        int sleep_ms = target_ms - elapsed_ms;
        if (sleep_ms > 1) {
            vTaskDelay(pdMS_TO_TICKS(sleep_ms));
        } else {
            taskYIELD();  // yield briefly so other tasks stay responsive
        }
    }

    _gif.close();
    if (_gif_data) {
        heap_caps_free(_gif_data);
        _gif_data      = nullptr;
        _gif_data_size = 0;
    }
    heap_caps_free(_frame_buf);
    _frame_buf   = nullptr;
    _task_handle = nullptr;
    s_frame_buf  = nullptr;
    s_instance   = nullptr;

    if (_on_exit) _on_exit();
}

// ─── AnimatedGIF draw callback (row-level) ───────────────────────────────────

void BootAnim::_gif_draw(GIFDRAW *pDraw)
{
    if (!s_frame_buf) return;

    // Destination: the row at y=pDraw->y, starting at x=pDraw->iX.
    uint16_t *dest = s_frame_buf + (pDraw->y * SCREEN_WIDTH) + pDraw->iX;
    uint8_t  *src  = pDraw->pPixels;
    uint16_t *pal  = pDraw->pPalette;

    const int width = pDraw->iWidth;

    if (pDraw->ucHasTransparency) {
        const uint8_t trans_index = pDraw->ucTransparent;
        for (int x = 0; x < width; x++) {
            if (src[x] != trans_index) {
                dest[x] = pal[src[x]];
            }
        }
    } else {
        for (int x = 0; x < width; x++) {
            dest[x] = pal[src[x]];
        }
    }
}

// ─── SPIFFS I/O callbacks ─────────────────────────────────────────────────────

void *BootAnim::_gif_open(const char *fname, int32_t *pSize)
{
    s_gif_file = SPIFFS.open(fname, "r");
    if (!s_gif_file) {
        LOG_E("BootAnim: SPIFFS.open(%s) failed", fname);
        return nullptr;
    }
    *pSize = static_cast<int32_t>(s_gif_file.size());
    return static_cast<void *>(&s_gif_file);
}

void BootAnim::_gif_close(void * /*handle*/)
{
    s_gif_file.close();
}

int32_t BootAnim::_gif_read(GIFFILE *pFile, uint8_t *pBuf, int32_t iLen)
{
    File *f = static_cast<File *>(pFile->fHandle);
    int32_t n = static_cast<int32_t>(f->read(pBuf, static_cast<size_t>(iLen)));
    pFile->iPos += n;   // keep library's position counter in sync with SPIFFS
    return n;
}

int32_t BootAnim::_gif_seek(GIFFILE *pFile, int32_t iPosition)
{
    File *f = static_cast<File *>(pFile->fHandle);
    f->seek(static_cast<uint32_t>(iPosition));
    pFile->iPos = iPosition;  // sync library's position counter
    return iPosition;
}

// ─── PSRAM memory I/O callbacks ───────────────────────────────────────────────

void *BootAnim::_gif_open_mem(const char * /*fname*/, int32_t *pSize)
{
    *pSize = static_cast<int32_t>(s_instance->_gif_data_size);
    return s_instance->_gif_data;   // fHandle = raw data pointer
}

void BootAnim::_gif_close_mem(void * /*handle*/) { /* nothing to close */ }

int32_t BootAnim::_gif_read_mem(GIFFILE *pFile, uint8_t *pBuf, int32_t iLen)
{
    const uint8_t *data = static_cast<const uint8_t *>(pFile->fHandle);
    int32_t remaining = pFile->iSize - pFile->iPos;
    if (iLen > remaining) iLen = remaining;
    memcpy(pBuf, data + pFile->iPos, static_cast<size_t>(iLen));
    pFile->iPos += iLen;
    return iLen;
}

int32_t BootAnim::_gif_seek_mem(GIFFILE *pFile, int32_t iPosition)
{
    pFile->iPos = iPosition;
    return iPosition;
}
