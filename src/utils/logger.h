#pragma once
#include <Arduino.h>
#include <cstring>
#include <sys/time.h>

// ---------------------------------------------------------------------------
// Output backend — all log macros route through this single point.
// Swap to Serial0 or another stream if needed.
// ---------------------------------------------------------------------------
#define _LOG_PRINTF(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)

// ---------------------------------------------------------------------------
// ANSI color toggle — comment out to strip color codes from output.
// ---------------------------------------------------------------------------
#define LOG_COLOR_ENABLE

// ---------------------------------------------------------------------------
// Undefine Arduino ESP32 SDK macros so ours take precedence.
// ---------------------------------------------------------------------------
#ifdef log_i
#undef log_i
#endif
#ifdef log_d
#undef log_d
#endif
#ifdef log_w
#undef log_w
#endif
#ifdef log_e
#undef log_e
#endif

namespace dbg {

// Basename of __FILE__ (works on both '/' and '\' separators)
#define FILENAME \
    (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : \
    (strrchr(__FILE__, '/')  ? strrchr(__FILE__, '/')  + 1 : __FILE__))

// Returns a timestamp string.
// Format: "MM/DD HH:MM:SS:mmm" when NTP-synced, "+HH:MM:SS:mmm" otherwise.
static inline const char* log_time() {
    static char buf[20];
    struct timeval tv;
    gettimeofday(&tv, NULL);
    if (tv.tv_sec > 1577836800L) {          // after 2020-01-01 → NTP synced
        struct tm* t = localtime(&tv.tv_sec);
        snprintf(buf, sizeof(buf), "%02d/%02d %02d:%02d:%02d:%03d",
                 t->tm_mon + 1, t->tm_mday,
                 t->tm_hour, t->tm_min, t->tm_sec,
                 (int)(tv.tv_usec / 1000));
    } else {
        uint32_t ms = millis();
        snprintf(buf, sizeof(buf), "+%02u:%02u:%02u:%03u",
                 ms / 3600000u,
                 (ms % 3600000u) / 60000u,
                 (ms % 60000u) / 1000u,
                 ms % 1000u);
    }
    return buf;
}

// ---------------------------------------------------------------------------
// Internal header / footer macros
// color_n is a raw ANSI SGR number (e.g. 32 = green, 31 = red, 33 = yellow).
// new_line == true  → print timestamp prefix
// new_line == false → print color only (for inline continuations)
// ---------------------------------------------------------------------------
#ifdef LOG_COLOR_ENABLE
    #define _LOG_HDR(new_line, color_n) \
        do { \
            if (new_line) { \
                _LOG_PRINTF("\033[37m[%s]\033[" #color_n "m ", dbg::log_time()); \
            } else { \
                _LOG_PRINTF("\033[" #color_n "m"); \
            } \
        } while (0)
    #define _LOG_END_NL   _LOG_PRINTF("\033[0m\r\n")
    #define _LOG_END_NONE _LOG_PRINTF("\033[0m")
#else
    #define _LOG_HDR(new_line, color_n) \
        do { \
            if (new_line) { _LOG_PRINTF("[%s] ", dbg::log_time()); } \
        } while (0)
    #define _LOG_END_NL   _LOG_PRINTF("\r\n")
    #define _LOG_END_NONE ((void)0)
#endif

// Core line macro — not intended for direct use.
#define _log_line(new_line, color_n, fmt, ...) \
    do { \
        _LOG_HDR(new_line, color_n); \
        _LOG_PRINTF(fmt, ##__VA_ARGS__); \
        if (new_line) { _LOG_END_NL; } else { _LOG_END_NONE; } \
    } while (0)

// ---------------------------------------------------------------------------
// Level constants
// ---------------------------------------------------------------------------
#define LOG_LEVEL_ERROR   0
#define LOG_LEVEL_WARNING 1
#define LOG_LEVEL_INFO    2
#define LOG_LEVEL_LOG     3
#define LOG_LEVEL_DEBUG   4

#ifndef DBG_LEVEL
#define DBG_LEVEL LOG_LEVEL_LOG
#endif

// ---------------------------------------------------------------------------
// Public macros
//   LOG_x(fmt, ...)  — new line, with timestamp prefix
//   log_x(fmt, ...)  — inline / continuation, color only, no newline prefix
// ---------------------------------------------------------------------------

// DEBUG — white (color 0)
#if (DBG_LEVEL >= LOG_LEVEL_DEBUG)
    #define LOG_D(fmt, ...) _log_line(true,  0, fmt, ##__VA_ARGS__)
    #define log_d(fmt, ...) _log_line(false, 0, fmt, ##__VA_ARGS__)
#else
    #define LOG_D(...) ((void)0)
    #define log_d(...) ((void)0)
#endif

// LOG — cyan (color 36)
#if (DBG_LEVEL >= LOG_LEVEL_LOG)
    #define LOG_L(fmt, ...) _log_line(true,  36, fmt, ##__VA_ARGS__)
    #define log_l(fmt, ...) _log_line(false, 36, fmt, ##__VA_ARGS__)
#else
    #define LOG_L(...) ((void)0)
    #define log_l(...) ((void)0)
#endif

// INFO — green (color 32)
#if (DBG_LEVEL >= LOG_LEVEL_INFO)
    #define LOG_I(fmt, ...) _log_line(true,  32, fmt, ##__VA_ARGS__)
    #define log_i(fmt, ...) _log_line(false, 32, fmt, ##__VA_ARGS__)
#else
    #define LOG_I(...) ((void)0)
    #define log_i(...) ((void)0)
#endif

// WARNING — yellow (color 33)
#if (DBG_LEVEL >= LOG_LEVEL_WARNING)
    #define LOG_W(fmt, ...) _log_line(true,  33, fmt, ##__VA_ARGS__)
    #define log_w(fmt, ...) _log_line(false, 33, fmt, ##__VA_ARGS__)
#else
    #define LOG_W(...) ((void)0)
    #define log_w(...) ((void)0)
#endif

// ERROR — red (color 31), always enabled; includes file/line location variant
#define LOG_E(fmt, ...)     _log_line(true,  31, fmt, ##__VA_ARGS__)
#define log_e(fmt, ...)     _log_line(false, 31, fmt, ##__VA_ARGS__)
#define LOG_E_LOC(fmt, ...) LOG_E("%s:%d [%s] " fmt, FILENAME, __LINE__, __func__, ##__VA_ARGS__)

} // namespace dbg
