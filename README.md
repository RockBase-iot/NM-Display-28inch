**[中文](readme_cn.md) | English**

# NM-Display-2.8inch Factory Test Firmware (Arduino)

This project is the **PlatformIO / Arduino-framework** port of the NM-Display 2.8-inch factory-test firmware for the **NM-Display 2.8-inch development board** (ESP32-S3).  
It provides a sequential 10-item hardware self-test driven by an LVGL 8.x touch UI.

> **Note:** The original IDF version lives in a separate repository. This repo targets **PlatformIO + Arduino framework** only. The build system, library management, and driver APIs differ significantly — do not mix the two.

---

## Table of Contents

- [Hardware Overview](#hardware-overview)
- [IO Pin Assignment](#io-pin-assignment)
- [Environment & Dependencies](#environment--dependencies)
- [Project Structure](#project-structure)
- [Build & Flash](#build--flash)
- [Factory Test Flow](#factory-test-flow)
- [Test Items](#test-items)
- [Porting Guide](#porting-guide)
- [Notes](#notes)

---

## Hardware Overview

![NM-Display-2.8 inch Board](image/nm-display-28.png)

| Item         | Specification                               |
|--------------|---------------------------------------------|
| MCU          | ESP32-S3, 240 MHz, dual-core                |
| Flash        | 16 MB QIO                                   |
| PSRAM        | Octal PSRAM (OPI mode)                      |
| Display      | 2.8" ST7789, 320×240, SPI 80 MHz            |
| Touch        | FT6336 (I2C capacitive)                     |
| Audio Codec  | ES8311 (I2C control + I2S data, 16 kHz)     |
| IMU          | QMI8658 (6-axis accel/gyro, I2C)            |
| PMU          | AXP2101 (power management, I2C 0x34; Chip ID = **0x47** on this board, standard is 0x4A) |
| RTC          | PCF85063 (real-time clock, I2C)             |
| IO Expander  | TCA9554 (I2C, addr 0x20)                    |
| Camera       | DVP interface (OV series), RGB565 320×480   |
| SD Card      | SDMMC 1-bit mode                            |
| Wi-Fi        | ESP32-S3 built-in                           |
| Backlight    | LEDC PWM (10-bit, 5 kHz, CH1)              |
| Button       | BOOT button on GPIO0 (active low)           |

---

## IO Pin Assignment

### I2C Bus (Shared)

| GPIO | Function |
|------|----------|
| IO7  | I2C SCL  |
| IO8  | I2C SDA  |

> All peripherals share one I2C bus at 400 kHz: TCA9554 (0x20), FT6336 (0x38), AXP2101 (0x34), ES8311 (0x18), QMI8658 (0x6A), PCF85063 (0x51).

---

### LCD Display (ST7789, SPI)

| GPIO | Function        |
|------|-----------------|
| IO1  | SPI MOSI        |
| IO5  | SPI SCLK        |
| IO3  | LCD DC          |
| IO6  | Backlight (PWM) |
| NC   | SPI CS (tied low on board) |
| NC   | LCD RST (via TCA9554 IO1)  |

- SPI clock: 80 MHz, Mode 3
- Logical resolution after 90° rotation: 320×240

---

### Touch Panel (FT6336, I2C)

| GPIO | Function |
|------|----------|
| IO7  | I2C SCL  |
| IO8  | I2C SDA  |

- Reset: controlled by TCA9554 IO1

---

### IO Expander (TCA9554, I2C 0x20)

| TCA9554 Pin | Function                         |
|-------------|----------------------------------|
| IO1         | LCD + Touch module RESET (output)|
| IO7         | PA_CTRL — NS4150B amp enable (output) |

- Config register 0x03 = 0x7D (IO1 and IO7 as output, rest input)
- Output register 0x01: `0x82` = PA ON, `0x02` = PA OFF

---

### Audio Codec (ES8311, I2C 0x18 + I2S)

| GPIO | Function              |
|------|-----------------------|
| IO7  | I2C SCL (control)     |
| IO8  | I2C SDA (control)     |
| IO12 | I2S MCLK (256 × SR)  |
| IO13 | I2S BCLK              |
| IO15 | I2S LRCK              |
| IO16 | I2S DOUT (playback)   |
| IO14 | I2S DIN  (record)     |

- Sample rate: 16 000 Hz, 16-bit stereo, Philips/I2S format
- MCLK = 256 × 16 000 = 4 096 000 Hz
- PA amplifier (NS4150B) controlled via TCA9554 IO7

---

### IMU (QMI8658, I2C 0x6A)

| GPIO | Function |
|------|----------|
| IO7  | I2C SCL  |
| IO8  | I2C SDA  |

- Accel: ±4 G, ODR 1000 Hz
- Gyro: ±512 DPS, ODR 1000 Hz
- Mahony AHRS fusion (quaternion output)

---

### RTC (PCF85063, I2C 0x51)

- Resets to 2025-01-01 12:00:00 if stored year < 2025

---

### PMU (AXP2101, I2C 0x34)

- Monitors battery, VBUS, system voltage, DC/LDO rails
- Chip ID reported as `0x47` on this board (handled in driver)

---

### Camera (DVP)

| GPIO | Function  |
|------|-----------|
| IO38 | XCLK (20 MHz, LEDC CH0) |
| IO17 | VSYNC     |
| IO18 | HREF      |
| IO41 | PCLK      |
| IO45–IO21 | D0–D7  |
| IO8  | SCCB SDA  |
| IO7  | SCCB SCL  |

---

### SD Card (SDMMC 1-bit)

| GPIO | Function   |
|------|------------|
| IO9  | SDMMC D0   |
| IO10 | SDMMC CMD  |
| IO11 | SDMMC CLK  |

---

### Button

| GPIO | Function                |
|------|-------------------------|
| IO0  | BOOT button (active low)|

---

## Environment & Dependencies

### Required Tools

| Tool            | Version / Note                        |
|-----------------|---------------------------------------|
| PlatformIO IDE  | VS Code extension or CLI              |
| Python          | 3.9 – 3.12 (required by PlatformIO)   |
| Platform        | `espressif32 @ 6.6.0`                 |
| Framework       | Arduino (via PlatformIO)              |

### Library Dependencies (auto-resolved by PlatformIO)

| Library     | Version  | Note                                     |
|-------------|----------|------------------------------------------|
| lvgl/lvgl   | ^8.4.0   | Declared in `platformio.ini` `lib_deps`  |

> All other drivers (ST7789, FT6336, ES8311, QMI8658, AXP2101, PCF85063, TCA9554, camera) are included as **source files inside this repository** under `src/drivers/` and `src/bsp/`. No separate installation is required.

### Key Differences from the IDF Version

| Aspect              | IDF Version                        | This (Arduino) Version                          |
|---------------------|------------------------------------|-------------------------------------------------|
| Build system        | CMake + idf.py                     | PlatformIO (`pio run`)                          |
| Framework           | ESP-IDF v5.4.x                     | Arduino-ESP32 (via espressif32 @ 6.6.0)         |
| I2S driver API      | `i2s_std` (new IDF 5.x API)        | Legacy `driver/i2s.h` (IDF 4.4-style)           |
| LVGL port           | esp-lvgl-port component            | Custom `lvgl_port.cpp` in `src/drivers/display/`|
| Component manager   | idf_component_manager              | PlatformIO `lib_deps` + local source            |
| PSRAM config        | sdkconfig (menuconfig)             | `board_build.arduino.memory_type = qio_opi`     |
| Partition table     | `partitions.csv` via CMake         | `partitions/nm_display_28.csv` via `platformio.ini` |
| Camera driver       | esp32-camera component             | `esp32-camera` source included in repo          |

---

## Project Structure

```
NM-Display-28inch/
├── platformio.ini              # PlatformIO project config (board, libs, flags)
├── boards/
│   └── nm_display_28.json      # Custom board definition (ESP32-S3 + OPI PSRAM)
├── partitions/
│   └── nm_display_28.csv       # 16 MB partition table
│       # nvs 20 KB | otadata 8 KB | app0 4 MB | app1 4 MB | spiffs 6 MB
├── data/                       # SPIFFS assets (boot.gif, fonts, etc.)
├── scripts/
│   └── upload_all.py           # Post-build script: auto upload firmware + SPIFFS
├── src/
│   ├── main.cpp                # Arduino setup() / loop() — init sequence
│   ├── version.h               # APP_VERSION_STR "0.1.0"
│   ├── alg/
│   │   └── quaternion/         # Mahony AHRS, IMU bias, quaternion math
│   ├── app/
│   │   ├── application.cpp/.h  # Top-level app: task creation, peripheral init
│   │   └── task_config.h       # Core assignment & task priorities/stack sizes
│   ├── bsp/
│   │   └── nm_display_28/
│   │       ├── config.h        # All GPIO, I2C addr, SPI freq definitions
│   │       └── nm_display_28.cpp  # Board bring-up (I2C, SPI, I2S, SD, camera)
│   ├── drivers/
│   │   ├── devices/            # hal.cpp — peripheral device drivers
│   │   ├── display/
│   │   │   ├── lv_conf.h       # LVGL configuration (depth=16, swap=0, fonts…)
│   │   │   ├── lvgl_port.cpp/.h  # LVGL tick task + flush callback
│   │   │   └── spiport.h       # ST7789 SPI write helpers
│   │   └── touch/
│   │       ├── hal.h           # Touch HAL interface
│   │       └── iicport.h       # FT6336 I2C read
│   └── ui/
│       ├── boot_anim/          # Boot animation (GIF player)
│       └── factory_test/
│           ├── factory_test.h/.cpp  # 10-item sequential factory test UI
│           └── fonts/          # Inconsolata_16, Inconsolata_20 bitmaps
```

---

## Build & Flash

### 1. Install PlatformIO

```bash
# Via pip (if not using VS Code extension)
pip install platformio
```

### 2. Build firmware

```bash
cd NM-Display-28inch
pio run -e nm-display-28
```

### 3. Flash firmware + SPIFFS in one step

The project includes a post-build script (`scripts/upload_all.py`) that automatically builds and uploads the SPIFFS filesystem image after the firmware upload completes.

```bash
pio run -e nm-display-28 --target upload --upload-port COMx
```

Replace `COMx` with the actual serial port (e.g. `COM48` on Windows, `/dev/ttyACM0` on Linux).

> **Tip:** In VS Code with PlatformIO extension, clicking the "Upload" button performs both steps automatically.

### 4. Flash firmware only (skip SPIFFS)

```bash
pio run -e nm-display-28 --target upload --upload-port COMx
pio run -e nm-display-28 --target uploadfs --upload-port COMx
```

### 5. Serial monitor

```bash
pio device monitor -e nm-display-28 --port COMx
```

Baud rate: **115 200**. The monitor filter includes `esp32_exception_decoder` and `time` prefix.

### 6. Partition table

| Partition | Type   | Offset    | Size   |
|-----------|--------|-----------|--------|
| nvs       | data   | 0x009000  | 20 KB  |
| otadata   | data   | 0x00E000  | 8 KB   |
| app0      | ota_0  | 0x010000  | 4 MB   |
| app1      | ota_1  | 0x410000  | 4 MB   |
| spiffs    | spiffs | 0x810000  | 6 MB   |

---

## Factory Test Flow

On power-on, the board runs a boot animation (GIF), then launches the factory test sequence automatically.

```
Power-on
    ↓
Boot animation (SPIFFS GIF)
    ↓
FactoryTest::run()
    ├─ T01  Display    — color fill (R/G/B/W/K) + PASS/FAIL
    ├─ T02  Touch      — draw touch trail, tap OK when done
    ├─ T03  SD Card    — mount + read capacity
    ├─ T04  Wi-Fi      — scan + connect (credentials in config)
    ├─ T05  IMU        — QMI8658 WHO_AM_I + accel/gyro live data
    ├─ T06  PMU        — AXP2101 chip ID + voltage rails
    ├─ T07  RTC        — PCF85063 probe + time read/write
    ├─ T08  Camera     — DVP capture + live preview
    ├─ T09  Codec      — ES8311 ID check + sweep (500~4 kHz) + Ode to Joy
    └─ T10  Mic        — ES8311 ADC record 5 s + auto-playback
    ↓
Summary screen (P/F/S per item) + Reboot button
```

Each test shows a title bar with **Failed** / **Ok** buttons. The tester can override the automatic result at any time.

---

## Test Items

| # | Name    | Pass Criteria                                                    |
|---|---------|------------------------------------------------------------------|
| 1 | Display | All 5 colors (R/G/B/W/K) render correctly, tester confirms       |
| 2 | Touch   | Touch trail visible on screen, tester confirms                   |
| 3 | SD Card | Mounts successfully, capacity > 0                                |
| 4 | Wi-Fi   | Connects to AP and obtains IP address                            |
| 5 | IMU     | WHO_AM_I = 0x05, accel/gyro values plausible (non-zero)         |
| 6 | PMU     | Chip ID = 0x47, all voltage rails within expected range          |
| 7 | RTC     | I2C probe OK, time can be set and read back                      |
| 8 | Camera  | Frame captured and displayed (non-black image)                   |
| 9 | Codec   | ID[FD]=0x83 + ID[FE]=0x11; sweep tone and melody audible        |
|10 | Mic     | Record 5 s via analog MIC (ES8311 ADC), playback audible        |

---

## Porting Guide

### Change board pins

All GPIO numbers are centralised in `src/bsp/nm_display_28/config.h`. Edit only that file for hardware changes.

### Change Wi-Fi credentials

Edit `src/app/application.cpp` (or wherever `wifi_init()` is called):

```cpp
wifi_connect("YOUR_SSID", "YOUR_PASSWORD");
```

### Add a new BSP

1. Create `src/bsp/<board_name>/config.h` with the new pin definitions.
2. Create `src/bsp/<board_name>/<board_name>.cpp` for board bring-up.
3. Add a new `[env:<board_name>]` section in `platformio.ini`.
4. Update `build_src_filter` to include the new BSP and exclude the old one.

### LVGL configuration

Edit `src/drivers/display/lv_conf.h`. Key settings for this board:

```c
#define LV_COLOR_DEPTH     16
#define LV_COLOR_16_SWAP   0   // ST7789 on this board does NOT need byte-swap
#define LV_HOR_RES_MAX     320
#define LV_VER_RES_MAX     240
```

---

## Notes

- **AXP2101 Chip ID**: The standard AXP2101 Chip ID is `0x4A`. This board reports `0x47` instead. The driver accepts both values; no user action is required. `board_build.arduino.memory_type = qio_opi` in `platformio.ini` is mandatory. Removing it will cause a boot crash.
- **I2S driver**: This project uses the **legacy** `driver/i2s.h` API (IDF 4.4 style), not the newer `i2s_std` / `i2s_pdm` APIs available in IDF 5.x. This is intentional for compatibility with the Arduino-ESP32 framework version pinned in `platformio.ini`.
- **LEDC channel conflict**: Camera XCLK uses `LEDC_TIMER_0 / LEDC_CHANNEL_0`. Backlight PWM uses `LEDC_CHANNEL_1`. Do not share channels.
- **TCA9554 IO7 (PA_CTRL)**: The power amplifier (NS4150B) is not directly connected to an ESP32 GPIO — it is controlled via TCA9554 IO7. Writing to the TCA9554 output register is required before audio playback; omitting this step results in silence.
- **ES8311 register read**: The ES8311 does **not** support register auto-increment. Registers 0xFD and 0xFE (chip ID) must be read individually with separate `requestFrom()` calls.
- **Boot animation**: The GIF file must be uploaded to SPIFFS. If the SPIFFS partition is empty, the boot animation is skipped and the factory test starts immediately.
- **Firmware version**: Defined in `src/version.h` as `APP_VERSION_STR "0.1.0"`.
