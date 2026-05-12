#pragma once

// Boot animation player.
// Loads an animated GIF from SPIFFS, decodes it frame-by-frame into a full-screen
// PSRAM buffer, and pushes each frame directly to the display via SPI (bypassing
// LVGL for maximum throughput).  The LVGL mutex is held only during the SPI push
// so the LVGL tick task continues to run normally.
//
// Exit condition: user taps the touch screen.
// When the animation exits, the supplied ExitCallback is invoked from within the
// animation task.  Callers should start the next application phase (e.g. FactoryTest)
// from that callback.

// Maximum frame delay in milliseconds — caps the GIF's embedded inter-frame delay.
// Lower value = faster playback.  33 ms ≈ 30 fps; 16 ms ≈ 60 fps.
#ifndef BOOT_ANIM_MAX_FRAME_MS
#define BOOT_ANIM_MAX_FRAME_MS  33
#endif

#include <functional>
#include <AnimatedGIF.h>
#include "../../drivers/display/hal.h"
#include "../../drivers/touch/hal.h"

class BootAnim {
public:
    using ExitCallback = std::function<void()>;

    // disp      : fully initialised Display pointer (not null).
    // touch     : Touch pointer, or nullptr to skip touch detection.
    // gif_path  : SPIFFS path for the GIF file, e.g. "/boot.gif".
    BootAnim(Display *disp, Touch *touch, const char *gif_path = "/boot.gif");
    ~BootAnim();

    // Allocate the PSRAM frame buffer and start the animation FreeRTOS task.
    // on_exit is called (from the animation task context) after the animation
    // exits.  on_exit may be nullptr if no callback is needed.
    void start(ExitCallback on_exit = nullptr);

    // Signal the animation to stop.  Returns immediately; the task will exit
    // on its next frame boundary and then invoke on_exit.
    void stop();

    bool is_running() const { return _task_handle != nullptr; }

private:
    // FreeRTOS task entry point.
    static void _task_entry(void *arg);

    // Main playback loop — runs inside the FreeRTOS task.
    void _play_loop();

    // AnimatedGIF draw callback — writes one decoded pixel row into the frame buffer.
    // pDraw->y and pDraw->pPixels / pPalette describe the current row.
    static void _gif_draw(GIFDRAW *pDraw);

    // SPIFFS I/O callbacks (fallback when PSRAM preload fails).
    static void    *_gif_open (const char *fname, int32_t *pSize);
    static void     _gif_close(void *handle);
    static int32_t  _gif_read (GIFFILE *pFile, uint8_t *pBuf, int32_t iLen);
    static int32_t  _gif_seek (GIFFILE *pFile, int32_t iPosition);

    // Memory I/O callbacks — used when the GIF is preloaded into PSRAM.
    static void    *_gif_open_mem (const char *fname, int32_t *pSize);
    static void     _gif_close_mem(void *handle);
    static int32_t  _gif_read_mem (GIFFILE *pFile, uint8_t *pBuf, int32_t iLen);
    static int32_t  _gif_seek_mem (GIFFILE *pFile, int32_t iPosition);

    Display      *_disp;
    Touch        *_touch;
    const char   *_gif_path;
    ExitCallback  _on_exit;
    TaskHandle_t  _task_handle  = nullptr;
    volatile bool _stop_flag    = false;
    uint16_t     *_frame_buf    = nullptr;   // 320×240 × 2 bytes in PSRAM
    uint8_t      *_gif_data     = nullptr;   // entire GIF file preloaded in PSRAM
    size_t        _gif_data_size = 0;

    // AnimatedGIF is a member (heap-allocated with this object) so its ~8 KB
    // internal LZW decode buffer does NOT live on the FreeRTOS task stack.
    AnimatedGIF   _gif;
};
