#pragma once

// Board identity
#define BSP_BOARD_MODEL  "NMTech Display 2.8"

// Feature flags — used by driver layer for conditional compilation
// and by Application to conditionally init peripherals and tasks.
#define HAS_TOUCH_FEATURE
#define HAS_CAMERA_FEATURE
#define HAS_AUDIO_FEATURE          // ES8311
#define HAS_IMU_FEATURE            // QMI8658
#define HAS_PMU_FEATURE            // AXP2101
#define HAS_RTC_FEATURE            // PCF85063
#define HAS_SDCARD_FEATURE
#define HAS_IO_EXPANDER_FEATURE    // TCA9554
#define HAS_WIFI_FEATURE
#define HAS_ONE_HW_BTN             // BOOT button on GPIO0

// Display — ST7789 SPI, logical 320x240 after 270° rotation
#define SCREEN_WIDTH      320
#define SCREEN_HEIGHT     240
#define DISPLAY_ROTATION  90

// SPI bus — dedicated to LCD (SPI2)
#define LCD_SPI_HOST      SPI2_HOST
#define LCD_MOSI_PIN      1
#define LCD_SCLK_PIN      5
#define LCD_MISO_PIN      -1
#define LCD_CS_PIN        -1
#define LCD_DC_PIN        3
#define LCD_RST_PIN       -1
#define LCD_BL_PIN        6
#define LCD_SPI_FREQ_HZ   80000000   // 80 MHz

// Backlight LEDC — CH0 is reserved for camera XCLK; backlight must use CH1.
#define LCD_BL_LEDC_CH    1
#define LCD_BL_LEDC_FREQ  5000
#define LCD_BL_LEDC_RES   10          // 10-bit, duty range 0~1023

// Shared I2C bus — Touch, IMU, PMU, RTC, Audio, IO-Expander
#define I2C_SDA_PIN       8
#define I2C_SCL_PIN       7
#define I2C_FREQ_HZ       400000

// Camera DVP — XCLK uses LEDC_TIMER_0 + LEDC_CHANNEL_0 (do not conflict with backlight CH1).
#define CAM_XCLK_PIN      38
#define CAM_PCLK_PIN      41
#define CAM_VSYNC_PIN     17
#define CAM_HREF_PIN      18
#define CAM_D0_PIN        45
#define CAM_D1_PIN        47
#define CAM_D2_PIN        48
#define CAM_D3_PIN        46
#define CAM_D4_PIN        42
#define CAM_D5_PIN        40
#define CAM_D6_PIN        39
#define CAM_D7_PIN        21
#define CAM_PWDN_PIN      -1
#define CAM_RESET_PIN     -1
#define CAM_LEDC_TIMER    LEDC_TIMER_0    // dedicated to camera XCLK
#define CAM_LEDC_CH       LEDC_CHANNEL_0  // dedicated to camera XCLK

// I2S audio — ES8311
#define I2S_MCLK_PIN      12
#define I2S_BCLK_PIN      13
#define I2S_LRCK_PIN      15
#define I2S_DOUT_PIN      16   // playback  (Master -> Codec)
#define I2S_DIN_PIN       14   // recording (Codec  -> Master)
#define I2S_SAMPLE_RATE   16000

// SD card — SDMMC 1-bit mode
#define SD_CLK_PIN        11
#define SD_CMD_PIN        10
#define SD_D0_PIN         9

// Button
#define BOOT_BTN_PIN      0

// I2C device addresses
#define TCA9554_I2C_ADDR  0x20   // A2A1A0 = 000
#define FT6336_I2C_ADDR   0x38
#define AXP2101_I2C_ADDR  0x34
#define ES8311_I2C_ADDR   0x18
// QMI8658 and PCF85063 use built-in address constants from the sensor library.

// TCA9554 pin mapping
#define IO_EXP_LCD_RST_PIN  1   // TCA9554 IO1 -> LCD RESET
