#pragma once
#include <Arduino.h>

// Forward declarations
void* create_board();
class Display;
class Touch;
class Button;
class Led;
class Camera;
class Audio;

// BoardProfile — returned by each BSP Board::get_profile().
// Application::init() and begin() use this to conditionally enable
// peripherals and spawn FreeRTOS tasks.
struct BoardProfile {
    // Display
    uint16_t screen_width;
    uint16_t screen_height;
    // Core peripherals
    bool has_touch;
    bool has_button;
    bool has_led;
    bool has_gauge;
    // Extended peripherals
    bool has_camera;   // DVP camera (OV series)
    bool has_audio;    // I2S codec (ES8311)
    bool has_imu;      // 6-axis IMU (QMI8658)
    bool has_pmu;      // Power management (AXP2101)
    bool has_rtc;      // Real-time clock (PCF85063)
    bool has_sdcard;   // SDMMC SD card
};

// Board — abstract base class for all BSP implementations.
class Board {
private:
    Board(const Board&) = delete;
    Board& operator=(const Board&) = delete;

protected:
    Board() {}

public:
    static Board& GetInstance() {
        static Board* instance = static_cast<Board*>(create_board());
        return *instance;
    }

    virtual ~Board() = default;

    virtual void    init()    = 0;
    virtual void    deinit()  = 0;
    virtual void    restart() = 0;

    virtual Display* get_display()      = 0;
    virtual Touch*   get_touch()        = 0;
    virtual Stream&  get_uart()         = 0;

    virtual Button*  get_boot_button()  { return nullptr; }
    virtual Button*  get_user_button()  { return nullptr; }
    virtual Led*     get_led()          { return nullptr; }
    virtual Camera*  get_camera()       { return nullptr; }
    virtual Audio*   get_audio()        { return nullptr; }

    virtual String   get_board_model()  = 0;
    virtual float    get_mcu_temp()     = 0;

    virtual const BoardProfile& get_profile() const = 0;
};

// Factory macro — place at the end of each BSP implementation file.
#define DECLARE_BOARD(BOARD_CLASS_NAME)     \
void* create_board() {                      \
    return new BOARD_CLASS_NAME();          \
}
