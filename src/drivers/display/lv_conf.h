/**
 * lv_conf.h — LVGL 8.4.x configuration for NM-Display-28inch
 * Place this file in src/drivers/display/ so that LVGL finds it via
 * -I "./src/drivers/display" combined with -D LV_CONF_INCLUDE_SIMPLE=1.
 */

/* clang-format off */
#if 1   /* Set to 1 to enable this configuration */

#ifndef LV_CONF_H
#define LV_CONF_H

#include <stdint.h>

/*====================
   COLOR SETTINGS
 *====================*/

/* Color depth: 1, 8, 16, 32 */
#define LV_COLOR_DEPTH 16

/* Byte-swap in LVGL frame buffer (needed for 8-bit SPI interfaces).
 * We use swap_bytes=true in SPIScreen::pushColors instead, so keep this 0. */
#define LV_COLOR_16_SWAP 0

/* Transparent background support — not needed for a solid background display. */
#define LV_COLOR_SCREEN_TRANSP 0

/* Rounding for color mix (blending).  0 = round down. */
#define LV_COLOR_MIX_ROUND_OFS 0

/* Chroma-key color for image transparency */
#define LV_COLOR_CHROMA_KEY lv_color_hex(0x00FF00)  /* pure green */

/*=========================
   MEMORY SETTINGS
 *=========================*/

/* Use custom allocator backed by external PSRAM (16 MB available). */
#define LV_MEM_CUSTOM 1
#if LV_MEM_CUSTOM
    #define LV_MEM_CUSTOM_INCLUDE            <esp_heap_caps.h>
    #define LV_MEM_CUSTOM_ALLOC(size)        heap_caps_malloc((size), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
    #define LV_MEM_CUSTOM_FREE               heap_caps_free
    #define LV_MEM_CUSTOM_REALLOC(ptr, size) heap_caps_realloc((ptr), (size), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
#endif

/* Intermediate rendering buffers */
#define LV_MEM_BUF_MAX_NUM 4

#define LV_MEMCPY_MEMSET_STD 0

/*====================
   HAL SETTINGS
 *====================*/

/* Display refresh period in ms (50 fps) */
#define LV_DISP_DEF_REFR_PERIOD 20

/* Touch / indev read period in ms */
#define LV_INDEV_DEF_READ_PERIOD 20

/* Use Arduino millis() as LVGL tick source — no dedicated tick task needed. */
#define LV_TICK_CUSTOM 1
#if LV_TICK_CUSTOM
    #define LV_TICK_CUSTOM_INCLUDE      "Arduino.h"
    #define LV_TICK_CUSTOM_SYS_TIME_EXPR (millis())
#endif

/* Default DPI (does not affect pixels, only default widget sizes) */
#define LV_DPI_DEF 130

/*=======================
 * FEATURE CONFIGURATION
 *=======================*/

/* Complex draw engine — required for shadows, rounded corners, arcs, etc. */
#define LV_DRAW_COMPLEX 1
#if LV_DRAW_COMPLEX
    #define LV_SHADOW_CACHE_SIZE 0
    #define LV_CIRCLE_CACHE_SIZE 4
#endif

/* Simple layer buffer for widgets with opa < 255 */
#define LV_LAYER_SIMPLE_BUF_SIZE          (8 * 1024)
#define LV_LAYER_SIMPLE_FALLBACK_BUF_SIZE (3 * 1024)

/* Image cache — disable unless using complex image decoders */
#define LV_IMG_CACHE_DEF_SIZE 0

/* Gradient stops */
#define LV_GRADIENT_MAX_STOPS 2

/* Default gradient dither bits */
#define LV_DITHER_GRADIENT 0
#define LV_DITHER_ERROR_DIFFUSION 0

/* Rotation granularity for transform APIs */
#define LV_DISP_ROT_MAX_BUF (10 * 1024)

/*-------------
 * GPU
 *-----------*/
#define LV_USE_GPU_STM32_DMA2D 0
#define LV_USE_GPU_SWM341_DMAS 0
#define LV_USE_GPU_NXP_PXP     0
#define LV_USE_GPU_NXP_VG_LITE  0
#define LV_USE_GPU_SDL          0

/*-------------
 * Logging
 *-----------*/
#define LV_USE_LOG 0
#if LV_USE_LOG
    #define LV_LOG_LEVEL LV_LOG_LEVEL_WARN
    #define LV_LOG_PRINTF 0
    #define LV_LOG_USE_TIMESTAMP 1
    #define LV_LOG_USE_FILE_LINE 1
#endif

/*-------------
 * Asserts
 *-----------*/
#define LV_USE_ASSERT_NULL          1
#define LV_USE_ASSERT_MALLOC        1
#define LV_USE_ASSERT_STYLE         0
#define LV_USE_ASSERT_MEM_INTEGRITY 0
#define LV_USE_ASSERT_OBJ           0

#define LV_ASSERT_HANDLER_INCLUDE <stdint.h>
/* Use a complete loop statement so the macro is valid inside if(expr){ LV_ASSERT_HANDLER } */
#define LV_ASSERT_HANDLER for(;;) {}

/*-------------
 * Others
 *-----------*/
#define LV_USE_PERF_MONITOR     0
#define LV_USE_MEM_MONITOR      0
#define LV_USE_REFR_DEBUG       0

#define LV_SPRINTF_CUSTOM 0
#define LV_SPRINTF_USE_FLOAT 0

#define LV_USE_USER_DATA 1

/* GC (garbage collection) — unused on ESP32 */
#define LV_ENABLE_GC 0

/*=====================
 *  COMPILER SETTINGS
 *====================*/
#define LV_BIG_ENDIAN_SYSTEM    0
#define LV_ATTRIBUTE_TICK_INC
#define LV_ATTRIBUTE_TIMER_HANDLER
#define LV_ATTRIBUTE_FLUSH_READY
#define LV_ATTRIBUTE_MEM_ALIGN_SIZE 4
#define LV_ATTRIBUTE_MEM_ALIGN      __attribute__((aligned(4)))
#define LV_ATTRIBUTE_LARGE_CONST
#define LV_ATTRIBUTE_LARGE_RAM_ARRAY
#define LV_ATTRIBUTE_FAST_MEM       IRAM_ATTR
#define LV_ATTRIBUTE_DMA
#define LV_EXPORT_CONST_INT(int_value) struct _silence_gcc_warning
#define LV_USE_LARGE_COORD 0

/*==================
 *   FONT USAGE
 *==================*/

/* Built-in Montserrat fonts (enable what you use) */
#define LV_FONT_MONTSERRAT_8  0
#define LV_FONT_MONTSERRAT_10 0
#define LV_FONT_MONTSERRAT_12 0
#define LV_FONT_MONTSERRAT_14 1   /* Default widget font */
#define LV_FONT_MONTSERRAT_16 1   /* Used for bold-looking test titles */
#define LV_FONT_MONTSERRAT_18 0
#define LV_FONT_MONTSERRAT_20 0
#define LV_FONT_MONTSERRAT_22 0
#define LV_FONT_MONTSERRAT_24 0
#define LV_FONT_MONTSERRAT_26 0
#define LV_FONT_MONTSERRAT_28 0
#define LV_FONT_MONTSERRAT_30 0
#define LV_FONT_MONTSERRAT_32 0
#define LV_FONT_MONTSERRAT_34 0
#define LV_FONT_MONTSERRAT_36 0
#define LV_FONT_MONTSERRAT_38 0
#define LV_FONT_MONTSERRAT_40 0
#define LV_FONT_MONTSERRAT_42 0
#define LV_FONT_MONTSERRAT_44 0
#define LV_FONT_MONTSERRAT_46 0
#define LV_FONT_MONTSERRAT_48 0

/* Special fonts */
#define LV_FONT_MONTSERRAT_12_SUBPX      0
#define LV_FONT_MONTSERRAT_28_COMPRESSED 0
#define LV_FONT_DEJAVU_16_PERSIAN_HEBREW 0
#define LV_FONT_SIMSUN_16_CJK            0

/* Built-in pixel fonts */
#define LV_FONT_UNSCII_8  0
#define LV_FONT_UNSCII_16 0   /* Replaced by Inconsolata_16/20 custom fonts */

/* Custom font declarations — add here if using external fonts */
/* #define LV_FONT_CUSTOM_DECLARE */

/* Default font used by themes and built-in widgets */
#define LV_FONT_DEFAULT &lv_font_montserrat_14

/* Enable built-in font drawing — do not disable */
#define LV_FONT_FMT_TXT_LARGE 0
#define LV_USE_FONT_SUBPX     0
#if LV_USE_FONT_SUBPX
    #define LV_FONT_SUBPX_BGR 0
#endif

/*=================
 *  TEXT SETTINGS
 *===============*/
#define LV_TXT_ENC LV_TXT_ENC_UTF8
#define LV_TXT_BREAK_CHARS " "
#define LV_TXT_LINE_BREAK_LONG_LEN 0
#define LV_TXT_LINE_BREAK_LONG_PRE_MIN_LEN  3
#define LV_TXT_LINE_BREAK_LONG_POST_MIN_LEN 3
#define LV_TXT_COLOR_CMD "#"
#define LV_USE_BIDI     0
#define LV_USE_ARABIC_PERSIAN_CHARS 0

/*==================
 *  WIDGET USAGE
 *================*/

/* Core widgets */
#define LV_USE_ARC        1
#define LV_USE_BAR        1
#define LV_USE_BTN        1
#define LV_USE_BTNMATRIX  1
#define LV_USE_CANVAS     1
#define LV_USE_CHECKBOX   1
#define LV_USE_DROPDOWN   1
#define LV_USE_IMG        1
#define LV_USE_LABEL      1
#if LV_USE_LABEL
    #define LV_LABEL_TEXT_SELECTION      1
    #define LV_LABEL_LONG_TXT_HINT      1
#endif
#define LV_USE_LINE       1
#define LV_USE_ROLLER     1
#if LV_USE_ROLLER
    #define LV_ROLLER_INF_PAGES 7
#endif
#define LV_USE_SLIDER     1
#define LV_USE_SWITCH     1
#define LV_USE_TEXTAREA   1
#if LV_USE_TEXTAREA
    #define LV_TEXTAREA_DEF_PWD_SHOW_TIME 1500
#endif
#define LV_USE_TABLE      1

/*==================
 *  EXTRA WIDGETS
 *================*/
#define LV_USE_ANIMIMG    0
#define LV_USE_CALENDAR   0
#define LV_USE_CHART      0
#define LV_USE_COLORWHEEL 0
#define LV_USE_IMGBTN     0
#define LV_USE_KEYBOARD   0
#define LV_USE_LED        0
#define LV_USE_LIST       0
#define LV_USE_MENU       0
#define LV_USE_METER      0
#define LV_USE_MSGBOX     0
#define LV_USE_SPAN       0
#define LV_USE_SPINBOX    0
#define LV_USE_SPINNER    0
#define LV_USE_TABVIEW    0
#define LV_USE_TILEVIEW   0
#define LV_USE_WIN        0

/*==================
 *  THEMES
 *================*/
#define LV_USE_THEME_DEFAULT 1
#if LV_USE_THEME_DEFAULT
    #define LV_THEME_DEFAULT_DARK 0       /* 0 = light, 1 = dark */
    #define LV_THEME_DEFAULT_GROW 1
    #define LV_THEME_DEFAULT_TRANSITION_TIME 80
#endif

#define LV_USE_THEME_BASIC  0
#define LV_USE_THEME_MONO   0

/*==================
 *  LAYOUTS
 *================*/
#define LV_USE_FLEX 1
#define LV_USE_GRID 1

/*==================
 * 3rd party libs
 *================*/
#define LV_USE_FS_STDIO   0
#define LV_USE_FS_POSIX   0
#define LV_USE_FS_WIN32   0
#define LV_USE_FS_FATFS   0

#define LV_USE_PNG        0
#define LV_USE_BMP        0
#define LV_USE_SJPG       0
#define LV_USE_GIF        0
#define LV_USE_QRCODE     0
#define LV_USE_FFMPEG     0
#define LV_USE_RLOTTIE    0

/*==================
 *  OTHERS
 *================*/
#define LV_USE_SNAPSHOT   0
#define LV_USE_MONKEY     0
#define LV_USE_GRIDNAV    0
#define LV_USE_FRAGMENT   0
#define LV_USE_IMGFONT    0
#define LV_USE_MSG        0
#define LV_USE_IME_PINYIN 0

/*==================
 * EXAMPLES
 *================*/
#define LV_BUILD_EXAMPLES 0

/*===================
 * DEMO USAGE
 *==================*/
#define LV_USE_DEMO_WIDGETS        0
#define LV_USE_DEMO_KEYPAD_AND_ENCODER 0
#define LV_USE_DEMO_BENCHMARK      0
#define LV_USE_DEMO_STRESS         0
#define LV_USE_DEMO_MUSIC          0

/*--END OF LV_CONF_H--*/

#endif /* LV_CONF_H */

#endif /* #if 1 */
