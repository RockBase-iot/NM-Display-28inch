# NM-Display-28inch 迁移方案
## ESP-IDF → PlatformIO + Arduino（NMMiner 分层架构风格）

> **本文档面向 AI Agent**：每一节均包含「当前状态 → 目标状态 → 具体操作」三段式说明，代码片段均可直接执行，无需人工补全关键信息。
>
> 源项目路径：`C:\Users\bitpony\Desktop\NM-Display-28inch-idf`  
> 目标项目路径：`D:\code\pony\IoT\NM-Display-28inch`（当前 workspace）  
> 参考架构来源：`D:\code\pony\lotto\NMMiner`  
> 生成日期：2026-05-12

---

## 第一部分：背景与可行性结论

### 1.1 源项目技术栈

| 维度 | 值 |
|------|----|
| 芯片 | ESP32-S3 |
| 框架 | ESP-IDF v5.x |
| 构建 | CMake |
| Flash | 16MB QIO 80MHz |
| PSRAM | 16MB Octal 80MHz（必须） |
| CPU | 240MHz dual-core |
| LVGL | 8.4.0 |
| LCD | ST7789 SPI 2.8" 240×320（使用时旋转 270° → 逻辑分辨率 320×240） |

### 1.2 可行性结论

**结论：完全可行，整体工作量约 15～25 人天（有 ESP32 Arduino 经验）。**

| 子系统 | 迁移难度 | 摘要 |
|--------|----------|------|
| 摄像头（esp32-camera） | ⭐⭐ 低 | `esp_camera_init/fb_get/fb_return` API 在 Arduino 下完全相同，改动 <20 行 |
| PMU（AXP2101） | ⭐ 极低 | XPowersLib 原生支持 Arduino，调用代码零改动 |
| IMU（QMI8658） | ⭐ 极低 | sensorlib 原生支持 Arduino，调用代码零改动 |
| RTC（PCF85063） | ⭐ 极低 | sensorlib 原生支持 Arduino，调用代码零改动 |
| SD 卡 | ⭐⭐ 低 | `SD_MMC` 库直接替换，3 行代码 |
| WiFi | ⭐ 极低 | `WiFi.h` 替换约 200 行 IDF 代码 |
| LVGL UI 业务层 | ⭐⭐ 低 | tileview/*.cpp 中 LVGL API 90% 可直接复用 |
| 背光 PWM | ⭐ 极低 | `ledcSetup/Write` 替换 LEDC 驱动 |
| TCA9554 IO 扩展 | ⭐⭐ 低 | Wire 直接操作，逻辑简单 |
| FT6336 触控 | ⭐⭐⭐ 中 | 手写 I2C 寄存器读取或社区库 |
| ST7789 LCD 驱动 | ⭐⭐⭐ 中 | 参考 NMMiner `SPIScreen`，复用其 SPI 驱动架构 |
| LVGL 框架集成 | ⭐⭐⭐⭐ 高 | 替换 `esp_lvgl_port`，手动实现 flush/tick/mutex |
| ES8311 音频 | ⭐⭐⭐⭐ 高 | `esp_codec_dev` 无 Arduino 版，需重写 codec 寄存器初始化 |

### 1.3 摄像头专项结论（重要）

**摄像头不依赖 IDF，在 Arduino 框架下可以直接运行。**

- `esp_camera.h` 的全部 API（`esp_camera_init`、`esp_camera_fb_get`、`esp_camera_fb_return`、`esp_camera_sensor_get`）在 Arduino ESP32 下完全可用。
- `camera_config_t` 结构体、所有引脚定义、`CAMERA_FB_IN_PSRAM` 模式均无需改动。
- 唯一需要修改的是 `camera_tile.cpp` 中的 `lvgl_port_lock/unlock`（IDF 专属），用 FreeRTOS mutex 替换，改动约 10 行。

---

## 第二部分：NMMiner 架构解析（参考蓝本）

> Agent 在生成代码前必须理解此节，所有新代码均须遵循这套分层规则。

### 2.1 NMMiner 目录树（精简）

```
NMMiner/src/
├── main.cpp               ← 极简入口：setup()/loop() 各 2 行
├── version.h
│
├── bsp/                   ← 板级支持包（BSP）—— 每个硬件型号一个子目录
│   ├── esp32_2432s028r/
│   │   ├── config.h       ← 所有引脚 #define 和 feature #define
│   │   └── esp32_2432s028r.cpp  ← 实现 Board 抽象类（含 DECLARE_BOARD 宏）
│   └── ...（其他板子）
│
├── drivers/               ← 驱动抽象层（HAL + 具体实现）
│   ├── devices/
│   │   ├── hal.h          ← Board 抽象类 + BoardProfile 能力描述符
│   │   ├── hal.cpp
│   │   └── *.h            ← 每个板子的 device 特性 #define 文件
│   ├── displays/
│   │   ├── hal.h          ← Display 抽象类（纯虚：init/flush/rotate/blctrl）
│   │   ├── spiport.h      ← SPIScreen：Display 的 SPI 实现
│   │   └── display.h/cpp  ← LVGL 集成层（flush_cb/tick_task/mutex）
│   ├── touch/
│   │   ├── hal.h          ← Touch 抽象类（纯虚：init/read）
│   │   └── i2cport.h      ← I2C Touch 实现
│   ├── button/
│   │   ├── hal.h          ← Button 抽象类
│   │   └── onebutton_port.h ← OneButton 实现
│   ├── led/
│   ├── storage/           ← NVS Settings
│   └── gauge/
│
├── app/
│   ├── application.h/cpp  ← NMMiner 单例：init() 初始化所有硬件，begin() 启动所有 FreeRTOS 任务
│   ├── task_config.h      ← 所有 FreeRTOS 任务的 Core/Priority 常量
│   └── app_state.h        ← 应用状态枚举
│
├── ui/                    ← LVGL UI 层
│   ├── ui_manager.h/cpp   ← UI 初始化与页面路由
│   ├── layouts/           ← 不同屏幕分辨率的布局
│   └── pages/             ← 各功能页面
│
├── platform/
│   └── chip_target.h      ← 芯片级宏（ESP32-S3 等）
│
└── utils/
    ├── logger.h/cpp       ← 日志封装
    └── helper.h/cpp       ← 通用工具函数
```

### 2.2 NMMiner 核心设计原则（Agent 必读）

**原则一：main.cpp 极简**
```cpp
// NMMiner/src/main.cpp ← 这就是全部内容
#include "app/application.h"

void setup() {
    auto& app = NMDisplay28App::instance();
    app.init();
    app.begin();
}

void loop() {
    delay(1000);  // 所有逻辑在 FreeRTOS tasks 中运行
}
```

**原则二：Board 抽象 + DECLARE_BOARD 工厂**
```cpp
// bsp/nm_display_28/nm_display_28.cpp 的最后一行
DECLARE_BOARD(NMDisplay28Board);
// 展开为：void* create_board() { return new NMDisplay28Board(); }
// Board::GetInstance() 通过 create_board() 获取板级单例
```

**原则三：BoardProfile 能力描述符**
```cpp
// Board::get_profile() 返回此结构体
// Application::init() 据此决定初始化哪些外设
// Application::begin() 据此决定启动哪些 FreeRTOS 任务
struct BoardProfile {
    bool has_touch;
    bool has_camera;
    bool has_audio;
    bool has_imu;
    bool has_pmu;
    bool has_rtc;
    bool has_sdcard;
    // ... 等
};
```

**原则四：驱动层只暴露抽象接口**
- `Display`、`Touch`、`Button`、`Camera`、`Audio` 均为纯虚基类。
- BSP 通过 `Board::get_display()` 等方法返回具体实现指针。
- 上层代码（Application、UI）只依赖抽象接口，不感知具体硬件型号。

**原则五：FreeRTOS 任务由 Application::begin() 统一管理**
- 所有任务优先级、Core 分配、栈大小集中定义在 `task_config.h`。
- 任务函数声明为 `static void xxx_task(void*)` 并通过 `xTaskCreatePinnedToCore` 启动。

---

## 第三部分：目标项目目录结构

> 这是迁移完成后的完整目录树。Agent 应严格按此结构创建文件，不得随意新增或合并目录。

```
NM-Display-28inch/                     ← workspace 根（已存在）
├── platformio.ini                     ← 【新建】PlatformIO 配置
├── boards/
│   └── nm_display_28.json               ← 【新建】自定义板级文件（ESP32-S3, 16MB, Octal PSRAM）
├── partitions/                        ← 【新建】分区表目录（将来可扩充其他版本）
│   └── nm_display_28.csv                ← 【新建】 6MB app 分区表
├── migration-idf-to-arduino.md        ← 本文档
│
├── src/
│   ├── main.cpp                       ← 【新建】极简入口，仅 setup()/loop()
│   ├── version.h                      ← 【新建】项目版本号
│   │
│   ├── bsp/
│   │   └── nm_display_28/
│   │       ├── config.h               ← 【新建】所有引脚 #define + feature #define
│   │       └── nm_display_28.cpp      ← 【新建】实现 Board 抽象类，DECLARE_BOARD 结尾
│   │
│   ├── drivers/
│   │   ├── devices/
│   │   │   ├── hal.h                  ← 【复制 NMMiner】Board 抽象类 + BoardProfile（新增 camera/audio 字段）
│   │   │   └── hal.cpp                ← 【复制 NMMiner】
│   │   │
│   │   ├── display/
│   │   │   ├── hal.h                  ← 【复制 NMMiner】Display 抽象类
│   │   │   ├── spiport.h              ← 【复制 NMMiner】SPIScreen SPI 实现（ST7789）
│   │   │   ├── lvgl_port.h/cpp        ← 【新建】替代 esp_lvgl_port：flush_cb/tick/mutex
│   │   │   └── lv_conf.h              ← 【新建】LVGL 配置（从 sdkconfig 转换）
│   │   │
│   │   ├── touch/
│   │   │   ├── hal.h                  ← 【复制 NMMiner】Touch 抽象类
│   │   │   └── ft6336_port.h/cpp      ← 【新建】FT6336 I2C 实现
│   │   │
│   │   ├── button/
│   │   │   ├── hal.h                  ← 【复制 NMMiner】Button 抽象类
│   │   │   └── onebutton_port.h       ← 【复制 NMMiner】OneButton 实现
│   │   │
│   │   ├── io_expander/
│   │   │   └── tca9554.h/cpp          ← 【新建】TCA9554 Wire 直接实现（<50行）
│   │   │
│   │   ├── camera/
│   │   │   ├── hal.h                  ← 【新建】Camera 抽象接口
│   │   │   └── esp32cam_port.h/cpp    ← 【迁移】esp_camera_port.cpp，改动极小
│   │   │
│   │   ├── audio/
│   │   │   ├── hal.h                  ← 【新建】Audio 抽象接口
│   │   │   └── es8311_port.h/cpp      ← 【新建】重写（最大工作量）
│   │   │
│   │   ├── imu/
│   │   │   └── qmi8658_port.h/cpp     ← 【迁移】esp_qmi8658_port.cpp，修改 init() 参数
│   │   │
│   │   ├── pmu/
│   │   │   └── axp2101_port.h/cpp     ← 【迁移】esp_axp2101_port.cpp，修改 init() 参数
│   │   │
│   │   ├── rtc/
│   │   │   └── pcf85063_port.h/cpp    ← 【迁移】esp_pcf85063_port.cpp，修改 init() 参数
│   │   │
│   │   ├── storage/
│   │   │   └── sdcard_port.h/cpp      ← 【新建】SD_MMC.h 封装（约 30 行）
│   │   │
│   │   └── network/
│   │       └── wifi_port.h/cpp        ← 【新建】WiFi.h 封装（替换 200 行 IDF 代码）
│   │
│   ├── app/
│   │   ├── application.h/cpp          ← 【新建】NMDisplay28App 单例（init/begin）
│   │   └── task_config.h              ← 【新建】所有 FreeRTOS 任务常量
│   │
│   ├── ui/
│   │   ├── ui_manager.h/cpp           ← 【新建】替代 lvgl_ui.cpp，LVGL tileview 路由
│   │   └── tiles/                     ← 【迁移】从 lvgl_ui/tileview/ 复制，修改 include
│   │       ├── rgb_tile.h/cpp
│   │       ├── system_tile.h/cpp
│   │       ├── axp2101_tile.h/cpp
│   │       ├── qmi8658_tile.h/cpp
│   │       ├── camera_tile.h/cpp      ← 修改 lvgl_port_lock → LVGL_LOCK 宏
│   │       └── wifi_tile.h/cpp
│   │
│   └── utils/
│       ├── logger.h/cpp               ← 【复制 NMMiner 或新建】Serial.printf 封装
│       └── helper.h/cpp               ← 【新建】通用工具
│
└── boards/                            ← 【可选】自定义 board JSON
```

---

## 第四部分：关键文件实现规范

> Agent 生成每个文件时，以本节为规范。每节给出文件的完整骨架，Agent 负责填充实现。

### 4.1 `src/main.cpp`（极简，不得添加任何业务逻辑）

```cpp
// src/main.cpp
#include "app/application.h"

void setup() {
    auto& app = NMDisplay28App::instance();
    app.init();
    app.begin();
}

void loop() {
    delay(1000);  // 所有逻辑运行在 FreeRTOS 任务中
}
```

---

### 4.2 `src/bsp/nm_display_28/config.h`（所有硬件定义集中在此，不得分散）

```cpp
// src/bsp/nm_display_28/config.h
#pragma once

// ── 板子标识 ──────────────────────────────────────────────────────────────
#define BSP_BOARD_MODEL  "NMTech Display 2.8"

// ── Feature Flags（驱动层据此条件编译，Application 据此装配任务）─────────
#define HAS_TOUCH_FEATURE
#define HAS_CAMERA_FEATURE
#define HAS_AUDIO_FEATURE          // ES8311
#define HAS_IMU_FEATURE            // QMI8658
#define HAS_PMU_FEATURE            // AXP2101
#define HAS_RTC_FEATURE            // PCF85063
#define HAS_SDCARD_FEATURE
#define HAS_IO_EXPANDER_FEATURE    // TCA9554
#define HAS_WIFI_FEATURE
#define HAS_ONE_HW_BTN             // BOOT 按键 GPIO0

// ── 屏幕（SPI ST7789，逻辑分辨率旋转后 320×240）────────────────────────
#define SCREEN_WIDTH     320
#define SCREEN_HEIGHT    240
#define DISPLAY_ROTATION 270       // 硬件旋转角度

// ── SPI 总线（LCD 独占 SPI2）────────────────────────────────────────────
#define LCD_SPI_HOST     SPI2_HOST
#define LCD_MOSI_PIN     1
#define LCD_SCLK_PIN     5
#define LCD_MISO_PIN     -1
#define LCD_CS_PIN       -1
#define LCD_DC_PIN       3
#define LCD_RST_PIN      -1
#define LCD_BL_PIN       6
#define LCD_SPI_FREQ_HZ  80000000  // 80MHz

// ── 背光 LEDC（注意：CH0 被摄像头 XCLK 占用，背光必须用 CH1）────────
#define LCD_BL_LEDC_CH   1
#define LCD_BL_LEDC_FREQ 5000
#define LCD_BL_LEDC_RES  10        // 10bit → 0~1023

// ── I2C 总线（Touch/IMU/PMU/RTC/Audio/IO-Expander 共用）────────────────
#define I2C_SDA_PIN      8
#define I2C_SCL_PIN      7
#define I2C_FREQ_HZ      400000

// ── 摄像头 DVP（XCLK 使用 LEDC_TIMER_0 + LEDC_CHANNEL_0）────────────
#define CAM_XCLK_PIN     38
#define CAM_PCLK_PIN     41
#define CAM_VSYNC_PIN    17
#define CAM_HREF_PIN     18
#define CAM_D0_PIN       45
#define CAM_D1_PIN       47
#define CAM_D2_PIN       48
#define CAM_D3_PIN       46
#define CAM_D4_PIN       42
#define CAM_D5_PIN       40
#define CAM_D6_PIN       39
#define CAM_D7_PIN       21
#define CAM_PWDN_PIN     -1
#define CAM_RESET_PIN    -1
#define CAM_LEDC_TIMER   LEDC_TIMER_0    // 摄像头 XCLK 专用
#define CAM_LEDC_CH      LEDC_CHANNEL_0  // 摄像头 XCLK 专用

// ── I2S 音频 ES8311 ──────────────────────────────────────────────────────
#define I2S_MCLK_PIN     12
#define I2S_BCLK_PIN     13
#define I2S_LRCK_PIN     15
#define I2S_DOUT_PIN     16   // 播放（Master→Codec）
#define I2S_DIN_PIN      14   // 录音（Codec→Master）
#define I2S_SAMPLE_RATE  16000

// ── SD 卡（SDMMC 1-bit）────────────────────────────────────────────────
#define SD_CLK_PIN       11
#define SD_CMD_PIN       10
#define SD_D0_PIN        9

// ── 按键 ─────────────────────────────────────────────────────────────────
#define BOOT_BTN_PIN     0

// ── I2C 设备地址 ──────────────────────────────────────────────────────────
#define TCA9554_I2C_ADDR 0x20   // A2A1A0 = 000
#define FT6336_I2C_ADDR  0x38
#define AXP2101_I2C_ADDR 0x34
#define ES8311_I2C_ADDR  0x18
// QMI8658 / PCF85063 使用 sensorlib 内置地址常量

// ── TCA9554 引脚分配（控制 LCD 复位）─────────────────────────────────────
#define IO_EXP_LCD_RST_PIN  1   // TCA9554 的 IO1 控制 LCD 复位
```

---

### 4.3 `src/bsp/nm_display_28/nm_display_28.cpp`（BSP 实现骨架）

```cpp
// src/bsp/nm_display_28/nm_display_28.cpp
#if defined(NM_DISPLAY_28)

#include "config.h"
#include "../../drivers/devices/hal.h"
#include "../../drivers/display/spiport.h"
#include "../../drivers/touch/ft6336_port.h"
#include "../../drivers/button/onebutton_port.h"

// ST7789 LCD vendor 初始化序列（参考 IDF esp_3inch5_lcd_port.cpp 的逻辑转换）
static const lcd_init_cmd_t st7789_init_cmds[] = { /* Agent：填充 ST7789 标准初始化序列 */ };
static const lcd_vendor_config_t st7789_vendor_cfg = {
    .init_cmds      = st7789_init_cmds,
    .init_cmds_size = sizeof(st7789_init_cmds) / sizeof(lcd_init_cmd_t),
    .color_inverted = true,   // IDF: esp_lcd_panel_invert_color = true
    .order_rgb      = false,  // BGR
    .swap_bytes     = true,
    .spi_mode       = 3,      // ST7789 需要 SPI MODE3
};

class NMDisplay28Board : public Board {
private:
    SPIClass      *_spi      = nullptr;
    SPIScreen     *_display  = nullptr;
    FT6336Touch   *_touch    = nullptr;
    OneButtonPort *_boot_btn = nullptr;

public:
    void init() override {
        Serial.begin(115200);

        // ① I2C 总线（Touch/PMU/IMU/RTC/Audio/IO-EXP 共用）
        Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);

        // ② TCA9554：复位 LCD（等效原 IDF io_expander_init）
        _tca9554_lcd_reset();

        // ③ SPI 总线 + ST7789 LCD
        _spi = new SPIClass(SPI2_HOST);
        _spi->begin(LCD_SCLK_PIN, LCD_MISO_PIN, LCD_MOSI_PIN, -1);
        _display = new SPIScreen(_spi, LCD_DC_PIN, LCD_RST_PIN, LCD_CS_PIN, LCD_BL_PIN,
                                 SCREEN_WIDTH, SCREEN_HEIGHT, LCD_SPI_FREQ_HZ);
        _display->load_vendor_config(st7789_vendor_cfg);
        _display->init();
        _display->rotate(DISPLAY_ROTATION);
        _display->blctrl(0.5f);

        // ④ FT6336 触控
        _touch = new FT6336Touch(SCREEN_WIDTH, SCREEN_HEIGHT);
        _touch->init();

        // ⑤ BOOT 按键
        _boot_btn = new OneButtonPort(BOOT_BTN_PIN, false, true);
    }

    void deinit()  override {}
    void restart() override { ESP.restart(); }

    Display* get_display()     override { return _display; }
    Touch*   get_touch()       override { return _touch; }
    Button*  get_boot_button() override { return _boot_btn; }
    Stream&  get_uart()        override { return Serial; }
    String   get_board_model() override { return BSP_BOARD_MODEL; }
    String   get_devcie_code() override { return ""; /* 可参考 NMMiner SHA256(MAC) 实现 */ }
    float    get_mcu_temp()    override { return temperatureRead(); }

    const BoardProfile& get_profile() const override {
        static const BoardProfile p = {
            .screen_width  = SCREEN_WIDTH,
            .screen_height = SCREEN_HEIGHT,
            .has_touch  = true,
            .has_button = true,
            .has_camera = true,
            .has_audio  = true,
            .has_imu    = true,
            .has_pmu    = true,
            .has_rtc    = true,
            .has_sdcard = true,
        };
        return p;
    }

private:
    void _tca9554_lcd_reset() {
        // 配置 IO1 为输出（Configuration register 0x03）
        Wire.beginTransmission(TCA9554_I2C_ADDR);
        Wire.write(0x03); Wire.write(0xFD);
        Wire.endTransmission();
        // 拉低 100ms → 拉高 100ms（等效 IDF io_expander_init 的复位时序）
        _tca9554_io1(0); delay(100);
        _tca9554_io1(1); delay(100);
    }

    void _tca9554_io1(uint8_t level) {
        Wire.beginTransmission(TCA9554_I2C_ADDR);
        Wire.write(0x01);  // Output port register
        Wire.write(level ? 0x02 : 0x00);
        Wire.endTransmission();
    }
};

DECLARE_BOARD(NMDisplay28Board);  // 工厂函数：void* create_board() { return new NMDisplay28Board(); }

#endif // NM_DISPLAY_28
```

---

### 4.4 `src/drivers/devices/hal.h`（在 NMMiner 版本基础上新增字段）

复制 NMMiner `src/drivers/devices/hal.h`，在 `BoardProfile` 结构体中**新增**以下字段：

```cpp
struct BoardProfile {
    // ── 原 NMMiner 字段（保持不变）────────────────────────────────────────
    uint16_t screen_width;
    uint16_t screen_height;
    bool has_touch;
    bool has_button;
    bool has_led;
    bool has_gauge;
    // ── NM-Display-28 新增字段 ─────────────────────────────────────────────
    bool has_camera;     // OV 系列 DVP 摄像头
    bool has_audio;      // ES8311 I2S 音频编解码
    bool has_imu;        // QMI8658 六轴 IMU
    bool has_pmu;        // AXP2101 电源管理
    bool has_rtc;        // PCF85063 实时时钟
    bool has_sdcard;     // SDMMC SD 卡
};
```

`Board` 纯虚接口新增（其余方法从 NMMiner 原样复制）：
```cpp
virtual Camera* get_camera() { return nullptr; }  // 无摄像头的板子返回 nullptr
virtual Audio*  get_audio()  { return nullptr; }   // 无音频的板子返回 nullptr
```

---

### 4.5 `src/drivers/display/lvgl_port.h/cpp`（替代 esp_lvgl_port，最关键新文件）

```cpp
// src/drivers/display/lvgl_port.h
#pragma once
#include <lvgl.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "hal.h"
#include "../touch/hal.h"

// ── LVGL 全局互斥量（替代 esp_lvgl_port 的 lvgl_port_lock/unlock）────────
// 所有操作 LVGL 的地方都必须先 LVGL_LOCK 再 LVGL_UNLOCK，包括 camera_tile
extern SemaphoreHandle_t g_lvgl_mutex;

#define LVGL_LOCK(timeout_ms)  (xSemaphoreTake(g_lvgl_mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE)
#define LVGL_UNLOCK()          xSemaphoreGive(g_lvgl_mutex)

class LvglPort {
public:
    // 初始化 LVGL：创建 PSRAM 帧缓冲、注册 Display/Touch 驱动、创建 tick 任务
    static void init(Display* display, Touch* touch, uint16_t w, uint16_t h);

    // LVGL tick + timer handler 任务（5ms 周期，Core0 高优先级）
    static void tick_task(void* arg);

    // LVGL flush 回调 → 转发到 Display::setAddrWindow + pushColors
    static void flush_cb(lv_disp_drv_t* drv, const lv_area_t* area, lv_color_t* color_p);

    // LVGL touch 读取回调 → 转发到 Touch::read
    static void touch_read_cb(lv_indev_drv_t* drv, lv_indev_data_t* data);
};
```

```cpp
// src/drivers/display/lvgl_port.cpp
#include "lvgl_port.h"
#include "../../app/task_config.h"

SemaphoreHandle_t g_lvgl_mutex = nullptr;

static Display* s_display = nullptr;
static Touch*   s_touch   = nullptr;

void LvglPort::init(Display* display, Touch* touch, uint16_t w, uint16_t h) {
    s_display = display;
    s_touch   = touch;

    lv_init();

    // ── 帧缓冲分配在 PSRAM（必须用 ps_malloc，内部 SRAM 不够）────────────
    // 大小：1/4 屏 × 2 buffer（双缓冲，避免撕裂）
    const uint32_t buf_px = w * h / 4;
    static lv_color_t* buf1 = (lv_color_t*)ps_malloc(buf_px * sizeof(lv_color_t));
    static lv_color_t* buf2 = (lv_color_t*)ps_malloc(buf_px * sizeof(lv_color_t));
    assert(buf1 != nullptr && buf2 != nullptr);  // 若 PSRAM 未启用则 crash，便于排查

    static lv_disp_draw_buf_t draw_buf;
    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, buf_px);

    // ── 注册显示驱动 ──────────────────────────────────────────────────────
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res      = w;
    disp_drv.ver_res      = h;
    disp_drv.flush_cb     = LvglPort::flush_cb;
    disp_drv.draw_buf     = &draw_buf;
    disp_drv.sw_rotate    = 1;   // 软件旋转（与 IDF flags.sw_rotate = 1 一致）
    disp_drv.full_refresh = 0;
    lv_disp_drv_register(&disp_drv);

    // ── 注册触控驱动 ──────────────────────────────────────────────────────
    if (touch) {
        static lv_indev_drv_t indev_drv;
        lv_indev_drv_init(&indev_drv);
        indev_drv.type    = LV_INDEV_TYPE_POINTER;
        indev_drv.read_cb = LvglPort::touch_read_cb;
        lv_indev_drv_register(&indev_drv);
    }

    // ── 互斥量（所有调用 LVGL API 的地方必须先持有此锁）─────────────────
    g_lvgl_mutex = xSemaphoreCreateMutex();

    // ── LVGL tick 任务：5ms 周期，Core0，最高优先级 ────────────────────
    xTaskCreatePinnedToCore(LvglPort::tick_task, "lvgl_tick",
                            TASK_STACK_LVGL, nullptr,
                            TASK_PRIORITY_LVGL_DRV, nullptr, LvglTaskCore);
}

void LvglPort::tick_task(void* arg) {
    const TickType_t period = pdMS_TO_TICKS(5);
    TickType_t last_wake    = xTaskGetTickCount();
    while (true) {
        if (LVGL_LOCK(5)) {
            lv_timer_handler();  // 驱动 LVGL 内部定时器
            LVGL_UNLOCK();
        }
        lv_tick_inc(5);
        vTaskDelayUntil(&last_wake, period);
    }
}

void LvglPort::flush_cb(lv_disp_drv_t* drv, const lv_area_t* area, lv_color_t* color_p) {
    if (!s_display) { lv_disp_flush_ready(drv); return; }
    uint16_t w = (uint16_t)(area->x2 - area->x1 + 1);
    uint16_t h = (uint16_t)(area->y2 - area->y1 + 1);
    s_display->setAddrWindow(area->x1, area->y1, w, h);
    s_display->pushColors((uint16_t*)color_p, w * h);
    lv_disp_flush_ready(drv);  // 必须调用，否则 LVGL 永久等待
}

void LvglPort::touch_read_cb(lv_indev_drv_t* drv, lv_indev_data_t* data) {
    if (!s_touch) { data->state = LV_INDEV_STATE_REL; return; }
    touch_point_t tp = s_touch->read();
    data->state   = tp.pressed ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
    data->point.x = (lv_coord_t)tp.x;
    data->point.y = (lv_coord_t)tp.y;
}
```

---

### 4.6 `src/app/task_config.h`

```cpp
// src/app/task_config.h
#pragma once
#include <freertos/FreeRTOS.h>

// ── Core 分配 ─────────────────────────────────────────────────────────────
#define CORE_0  0
#define CORE_1  1

#define LvglTaskCore    CORE_0  // LVGL tick（UI 流畅性最优先）
#define CameraTaskCore  CORE_1  // 摄像头帧读取（Core1 避免与 LVGL tick 竞争）
#define AudioTaskCore   CORE_0
#define SensorTaskCore  CORE_0
#define WifiTaskCore    CORE_0

// ── 优先级（数字越大优先级越高，配合 FreeRTOS preemptive scheduler）────
enum TaskPriority {
    TASK_PRIORITY_IDLE = 0,
    TASK_PRIORITY_SENSOR,       // IMU/RTC 轮询（最低，可被其他任务抢占）
    TASK_PRIORITY_AUDIO,        // ES8311 音频
    TASK_PRIORITY_WIFI,         // WiFi 扫描/连接
    TASK_PRIORITY_BTN,          // 按键去抖
    TASK_PRIORITY_CAMERA,       // 摄像头帧读取
    TASK_PRIORITY_LVGL_DRV,     // LVGL tick（最高，确保 5ms 周期不被打断）
};

// ── 栈大小（字节）────────────────────────────────────────────────────────
#define TASK_STACK_LVGL    8192
#define TASK_STACK_CAMERA  16384   // 摄像头：帧处理 + LVGL 图像更新
#define TASK_STACK_AUDIO   8192
#define TASK_STACK_SENSOR  4096
#define TASK_STACK_WIFI    8192
#define TASK_STACK_BTN     2048
```

---

### 4.7 `src/app/application.h/cpp`（应用单例骨架）

```cpp
// src/app/application.h
#pragma once
#include <Arduino.h>
#include "task_config.h"

class NMDisplay28App {
public:
    static NMDisplay28App& instance();
    bool init();    // 第一步：初始化所有硬件（在 setup() 中调用）
    void begin();   // 第二步：启动所有 FreeRTOS 任务（在 init() 成功后调用）
private:
    NMDisplay28App() = default;
    NMDisplay28App(const NMDisplay28App&) = delete;
    NMDisplay28App& operator=(const NMDisplay28App&) = delete;
};
```

```cpp
// src/app/application.cpp
#include "application.h"
#include "../drivers/devices/hal.h"
#include "../drivers/display/lvgl_port.h"
#include "../drivers/camera/esp32cam_port.h"
#include "../drivers/audio/es8311_port.h"
#include "../drivers/imu/qmi8658_port.h"
#include "../drivers/pmu/axp2101_port.h"
#include "../drivers/rtc/pcf85063_port.h"
#include "../drivers/storage/sdcard_port.h"
#include "../drivers/network/wifi_port.h"
#include "../ui/ui_manager.h"
#include "../bsp/nm_display_28/config.h"

NMDisplay28App& NMDisplay28App::instance() {
    static NMDisplay28App app;
    return app;
}

bool NMDisplay28App::init() {
    Board& board             = Board::GetInstance();
    const BoardProfile& prof = board.get_profile();

    // ① BSP 初始化：Serial/Wire/SPI/LCD/Touch/Button（board.cpp 实现）
    board.init();

    // ② 按 Feature Flag 条件初始化外设（顺序与原 IDF main.cpp 一致）
#if defined(HAS_PMU_FEATURE)
    axp2101_port_init();         // AXP2101：I2C 回调方式，Wire 接口
#endif
#if defined(HAS_AUDIO_FEATURE)
    es8311_port_init();          // ES8311：I2S + I2C 寄存器
#endif
#if defined(HAS_IMU_FEATURE)
    qmi8658_port_init();         // QMI8658：sensorlib Wire 接口
#endif
#if defined(HAS_RTC_FEATURE)
    pcf85063_port_init();        // PCF85063：sensorlib Wire 接口
#endif
#if defined(HAS_SDCARD_FEATURE)
    sdcard_port_init();          // SD_MMC 1-bit 模式
#endif
#if defined(HAS_CAMERA_FEATURE)
    camera_port_init();          // esp_camera_init，几乎零改动
#endif
#if defined(HAS_WIFI_FEATURE)
    wifi_port_init("NMTech-2.4G", "NMMiner2048");
#endif

    // ③ LVGL 初始化（依赖 Board::get_display/touch，必须在 BSP init 之后）
    LvglPort::init(board.get_display(), board.get_touch(),
                   prof.screen_width, prof.screen_height);

    return true;
}

void NMDisplay28App::begin() {
    // 摄像头任务：Core1，避免与 LVGL tick（Core0）竞争
#if defined(HAS_CAMERA_FEATURE)
    xTaskCreatePinnedToCore(camera_task, "cam_task",
                            TASK_STACK_CAMERA, nullptr,
                            TASK_PRIORITY_CAMERA, nullptr, CameraTaskCore);
#endif

    // 传感器轮询任务（IMU + RTC + PMU 状态更新）
    xTaskCreatePinnedToCore(sensor_task, "sensor_task",
                            TASK_STACK_SENSOR, nullptr,
                            TASK_PRIORITY_SENSOR, nullptr, SensorTaskCore);

    // WiFi 任务（扫描 + 连接状态维护）
#if defined(HAS_WIFI_FEATURE)
    xTaskCreatePinnedToCore(wifi_task, "wifi_task",
                            TASK_STACK_WIFI, nullptr,
                            TASK_PRIORITY_WIFI, nullptr, WifiTaskCore);
#endif

    // UI 初始化（在 LVGL 锁保护下，等效原 IDF 的 lvgl_port_lock(0) + lvgl_ui_init()）
    if (LVGL_LOCK(portMAX_DELAY)) {
        UIManager::instance().init();
        LVGL_UNLOCK();
    }
}
```

---

### 4.8 摄像头模块（改动最小，重点说明）

```cpp
// src/drivers/camera/esp32cam_port.cpp
// ─── 与 IDF esp_camera_port.cpp 的差异：仅 2 处 ──────────────────────────
// 【差异1】引脚从 config.h 宏读取（原来硬编码在函数内）
// 【差异2】camera_task 中 lvgl_port_lock/unlock → LVGL_LOCK/UNLOCK 宏
// 其余代码：camera_config_t 结构体、esp_camera_init、fb_get/return 完全不变

#include "esp32cam_port.h"
#include "../../bsp/nm_display_28/config.h"
#include "../display/lvgl_port.h"  // 提供 LVGL_LOCK/UNLOCK 宏

// ── 由 camera_tile.cpp extern 引用 ──────────────────────────────────────
lv_obj_t* cam_img_obj = nullptr;  // camera_tile_init() 创建后赋值

void camera_port_init() {
    camera_config_t config = {};
    config.ledc_channel = CAM_LEDC_CH;     // LEDC_CHANNEL_0（与背光 CH1 不冲突）
    config.ledc_timer   = CAM_LEDC_TIMER;  // LEDC_TIMER_0
    config.pin_d0 = CAM_D0_PIN;   config.pin_d1 = CAM_D1_PIN;
    config.pin_d2 = CAM_D2_PIN;   config.pin_d3 = CAM_D3_PIN;
    config.pin_d4 = CAM_D4_PIN;   config.pin_d5 = CAM_D5_PIN;
    config.pin_d6 = CAM_D6_PIN;   config.pin_d7 = CAM_D7_PIN;
    config.pin_xclk     = CAM_XCLK_PIN;
    config.pin_pclk     = CAM_PCLK_PIN;
    config.pin_vsync    = CAM_VSYNC_PIN;
    config.pin_href     = CAM_HREF_PIN;
    config.pin_sccb_sda = -1;   // 共用 Wire，无需独立 SCCB 引脚
    config.pin_sccb_scl = -1;
    config.sccb_i2c_port = 0;
    config.pin_pwdn   = CAM_PWDN_PIN;
    config.pin_reset  = CAM_RESET_PIN;
    config.xclk_freq_hz  = 20000000;
    config.frame_size    = FRAMESIZE_320X480;
    config.pixel_format  = PIXFORMAT_RGB565;
    config.grab_mode     = CAMERA_GRAB_WHEN_EMPTY;
    config.fb_location   = CAMERA_FB_IN_PSRAM;  // 必须 PSRAM
    config.jpeg_quality  = 12;
    config.fb_count      = 2;

    if (esp_camera_init(&config) != ESP_OK) {
        Serial.println("[Camera] Init failed");
        return;
    }
    esp_camera_sensor_get()->set_vflip(esp_camera_sensor_get(), 1);
}

// ── 关键改动：lvgl_port_lock → LVGL_LOCK（宏行为完全等价）──────────────
void camera_task(void* arg) {
    lv_img_dsc_t img_dsc = {};
    img_dsc.header.w  = 320;
    img_dsc.header.h  = 480;
    img_dsc.data_size = 320 * 480 * 2;
    img_dsc.header.cf = LV_IMG_CF_TRUE_COLOR;

    while (true) {
        camera_fb_t* pic = esp_camera_fb_get();
        if (pic && pic->len == (320 * 480 * 2) && cam_img_obj) {
            img_dsc.data = pic->buf;
            if (LVGL_LOCK(10)) {
                lv_img_set_src(cam_img_obj, &img_dsc);
                LVGL_UNLOCK();
            }
        }
        if (pic) esp_camera_fb_return(pic);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
```

---

### 4.9 传感器/PMU/RTC 迁移（init 签名变化说明）

IDF 版本传入 `i2c_master_bus_handle_t`，Arduino 版本使用全局 `Wire` 对象，无需传参。

```cpp
// AXP2101：axp2101_port_init()
// 原：esp_err_t esp_axp2101_port_init(i2c_master_bus_handle_t bus_handle)
// 新：void axp2101_port_init()
if (power.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA_PIN, I2C_SCL_PIN)) { /* 成功 */ }
// power.setVbusVoltageLimit / setVbusCurrentLimit 等后续调用完全不变

// QMI8658：qmi8658_port_init()
// 原：void esp_qmi8658_port_init(i2c_master_bus_handle_t bus_handle)
// 新：void qmi8658_port_init()
while (!qmi.begin(Wire, QMI8658_L_SLAVE_ADDRESS, I2C_SDA_PIN, I2C_SCL_PIN)) { delay(1000); }
// configAccelerometer / configGyroscope 完全不变，代码零修改

// PCF85063：pcf85063_port_init()
// 原：void esp_pcf85063_port_init(i2c_master_bus_handle_t bus_handle)
// 新：void pcf85063_port_init()
while (!rtc.begin(Wire, PCF85063_SLAVE_ADDRESS, I2C_SDA_PIN, I2C_SCL_PIN)) { delay(1000); }
// getDateTime / setDateTime / start() 完全不变
```

---

### 4.10 ES8311 音频（最大工作量，详细说明）

**策略**：I2S 驱动直接复用 IDF API（Arduino ESP32 内置），ES8311 寄存器初始化需重写。

```cpp
// src/drivers/audio/es8311_port.cpp

// ── Part 1：I2S 初始化（与 IDF 版本几乎完全相同）────────────────────────
// 以下头文件在 Arduino ESP32 框架下可直接 include（IDF 内置）
#include <driver/i2s_std.h>

static i2s_chan_handle_t tx_handle, rx_handle;

static void i2s_init() {
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    i2s_new_channel(&chan_cfg, &tx_handle, &rx_handle);

    i2s_std_config_t std_cfg = {};
    std_cfg.clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(I2S_SAMPLE_RATE);
    std_cfg.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
    std_cfg.gpio_cfg.mclk = (gpio_num_t)I2S_MCLK_PIN;
    std_cfg.gpio_cfg.bclk = (gpio_num_t)I2S_BCLK_PIN;
    std_cfg.gpio_cfg.ws   = (gpio_num_t)I2S_LRCK_PIN;
    std_cfg.gpio_cfg.dout = (gpio_num_t)I2S_DOUT_PIN;
    std_cfg.gpio_cfg.din  = (gpio_num_t)I2S_DIN_PIN;
    i2s_channel_init_std_mode(tx_handle, &std_cfg);
    i2s_channel_init_std_mode(rx_handle, &std_cfg);
    i2s_channel_enable(tx_handle);
    i2s_channel_enable(rx_handle);
}

// ── Part 2：ES8311 寄存器初始化（替代 esp_codec_dev）──────────────────
// 参考来源：https://github.com/espressif/esp-codec-dev（MIT License）
// 文件位置：components/esp_codec_dev/device/es8311/es8311_codec.c
// 需要的寄存器写入序列（约 25 个写操作）：
static void es8311_write_reg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(ES8311_I2C_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

static void es8311_init_registers() {
    es8311_write_reg(0x00, 0x1F);  // RESET：复位
    delay(10);
    es8311_write_reg(0x00, 0x00);  // RESET：正常工作
    // Agent：参考 esp_codec_dev 源码 es8311_codec.c 填充完整寄存器序列
    // 关键寄存器：时钟（0x01-0x0A）、ADC增益（0x14）、DAC音量（0x32）
}

void es8311_port_init() {
    i2s_init();
    es8311_init_registers();
}
```

---

### 4.11 其余驱动（简单，直接给出完整实现）

**SD 卡**：
```cpp
// src/drivers/storage/sdcard_port.cpp
#include <SD_MMC.h>
#include "../../bsp/nm_display_28/config.h"

void sdcard_port_init() {
    SD_MMC.setPins(SD_CLK_PIN, SD_CMD_PIN, SD_D0_PIN);
    if (!SD_MMC.begin("/sdcard", true)) {  // true = 1-bit（与 IDF slot_config.width=1 一致）
        Serial.println("[SD] Mount failed");
        return;
    }
    Serial.printf("[SD] Size: %.1f MB\n", (float)SD_MMC.cardSize() / 1048576.0f);
}

uint64_t sdcard_get_size() { return SD_MMC.cardSize(); }
```

**WiFi**：
```cpp
// src/drivers/network/wifi_port.cpp
#include <WiFi.h>

void wifi_port_init(const char* ssid, const char* password) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
}
bool wifi_port_is_connected() { return WiFi.status() == WL_CONNECTED; }
String wifi_port_get_ip()     { return WiFi.localIP().toString(); }
int wifi_port_get_rssi()      { return WiFi.RSSI(); }

// 同步扫描（在 WiFi task 中调用，不阻塞 UI）
int wifi_port_scan(char ssid_list[][33], int8_t rssi_list[], int max_count) {
    int n = WiFi.scanNetworks();
    for (int i = 0; i < n && i < max_count; i++) {
        strncpy(ssid_list[i], WiFi.SSID(i).c_str(), 32);
        rssi_list[i] = (int8_t)WiFi.RSSI(i);
    }
    WiFi.scanDelete();
    return min(n, max_count);
}
```

---

## 第五部分：`platformio.ini` 完整配置

```ini
; platformio.ini
[platformio]
src_dir   = src

[env:nm-display-28]
platform  = espressif32@6.6.0
board     = lilygo-t-display-s3   ; 借用：ESP32-S3、16MB Flash、兼容 qio_opi PSRAM
framework = arduino

; ── 上传/监控 ──────────────────────────────────────────────────────────────
upload_speed  = 921600
monitor_speed = 115200
monitor_filters = esp32_exception_decoder, time

; ── Flash / PSRAM ──────────────────────────────────────────────────────────
board_build.partitions          = partitions/nm_display_28.csv  ; 存放在 partitions/ 子目录，方便后续扩充其他分区表
board_build.flash_size          = 16MB
board_build.flash_mode          = qio
board_build.flash_freq          = 80m
board_build.arduino.memory_type = qio_opi             ; Octal PSRAM 必须

; ── 编译标志 ───────────────────────────────────────────────────────────────
build_flags =
    -I "./src"
    -I "./src/drivers/display"   ; LVGL 寻找 lv_conf.h
    ; ── 目标板标识（BSP 条件编译）
    -D NM_DISPLAY_28=1
    ; ── ESP32-S3 相关
    -D CHIP_ESP32_S3=1
    -D BOARD_HAS_PSRAM
    -D CONFIG_SPIRAM_CACHE_WORKAROUND=1
    -D ARDUINO_USB_MODE=1
    -D ARDUINO_USB_CDC_ON_BOOT=1
    ; ── LVGL 配置（与 IDF sdkconfig 对应项保持一致）
    -D LVGL_ENABLE=1
    -D LV_LVGL_H_INCLUDE_SIMPLE=1
    -D LV_CONF_INCLUDE_SIMPLE=1
    -D LV_COLOR_16_SWAP=1          ; 对应 CONFIG_LV_COLOR_16_SWAP=y
    ; ── UI 布局（320×240）
    -D UI_LAYOUT_320x240=1
    ; ── 调试（开发期 3，发布期 0）
    -D CORE_DEBUG_LEVEL=3

; ── 源文件过滤（只编译本板 BSP，排除其他板）──────────────────────────────
build_src_filter =
    +<*>
    -<bsp/**>
    +<bsp/nm_display_28/**>

; ── 依赖库 ────────────────────────────────────────────────────────────────
lib_deps =
    ; LVGL 8.4.x（对齐 IDF 项目版本）
    lvgl/lvgl@^8.4.0

    ; 摄像头（API 与 IDF 完全相同）
    https://github.com/espressif/esp32-camera

    ; 传感器库（QMI8658 + PCF85063，原生 Arduino 支持）
    lewisxhe/SensorLib@^0.2.2

    ; 电源管理（AXP2101，原生 Arduino 支持）
    lewisxhe/XPowersLib@^0.2.5

    ; 按键（替代 iot_button 组件）
    mathertel/OneButton@^2.5.0

    ; JSON（可选，WiFi 扫描结果等）
    bblanchon/ArduinoJson@^6.21.5
```

---

## 第六部分：IDF API → Arduino 对照速查表

> Agent 替换代码时查此表，不得使用表外猜测替换。

| IDF API / 概念 | Arduino 等价 | 备注 |
|----------------|-------------|------|
| `nvs_flash_init()` | `Preferences` 类 | — |
| `i2c_new_master_bus()` | `Wire.begin(sda, scl, freq)` | — |
| `i2c_master_transmit()` | `Wire.write()` + `endTransmission()` | — |
| `i2c_master_transmit_receive()` | `Wire.requestFrom()` + `read()` | — |
| `spi_bus_initialize()` | `SPIClass.begin(sclk, miso, mosi)` | — |
| `ledc_timer_config()` | `ledcSetup(ch, freq, res)` | — |
| `ledc_channel_config()` | `ledcAttachPin(pin, ch)` | — |
| `ledc_set_duty()` + `ledc_update_duty()` | `ledcWrite(ch, duty)` | — |
| `esp_vfs_fat` + `sdmmc_host` | `SD_MMC.begin()` | — |
| `esp_wifi.h` 全套 | `WiFi.h` | 200 行 → 30 行 |
| `esp_restart()` | `ESP.restart()` | — |
| `esp_get_free_heap_size()` | `ESP.getFreeHeap()` | — |
| `esp_psram_get_size()` | `ESP.getPsramSize()` | — |
| `esp_clk_cpu_freq()` | `getCpuFrequencyMhz()` | — |
| `temperature_sensor_get_celsius()` | `temperatureRead()` | — |
| `heap_caps_malloc(n, MALLOC_CAP_SPIRAM)` | `ps_malloc(n)` | **LVGL buffer 和摄像头 buffer 必须用此** |
| `vTaskDelay(pdMS_TO_TICKS(n))` | `delay(n)` 或 FreeRTOS 均可 | — |
| `xTaskCreatePinnedToCore()` | 同名，Arduino ESP32 直接支持 | — |
| `ESP_LOGI(tag, fmt, ...)` | `log_i(fmt, ...)` 或 `Serial.printf()` | — |
| `esp_lcd_panel_draw_bitmap()` | `Display::setAddrWindow` + `pushColors` | — |
| **`lvgl_port_lock(ms)`** | **`LVGL_LOCK(ms)`** | **全文搜索替换，必须 100% 覆盖** |
| **`lvgl_port_unlock()`** | **`LVGL_UNLOCK()`** | **同上** |
| `i2s_new_channel()` / `i2s_channel_init_std_mode()` | 同名，`#include <driver/i2s_std.h>` | **Arduino ESP32 内置，可直接调用** |
| `esp_camera_init()` | 同名 | **零改动** |
| `esp_camera_fb_get()` | 同名 | **零改动** |
| `esp_camera_fb_return()` | 同名 | **零改动** |
| `ESP_ERROR_CHECK(x)` | `if(x != ESP_OK) { Serial.printf(...); }` 或 assert | — |

---

## 第七部分：迁移执行顺序

> Agent 严格按此顺序执行，每阶段结束后验证再继续。

### Phase 1 — 项目骨架（约 0.5 天）
1. 创建 `boards/nm_display_28.json`（自定义板级文件，放入项目 `boards/` 目录）
2. 创建 `platformio.ini`（見第五部分）
3. 创建 `partitions/nm_display_28.csv`（分区表放入独立子目录，样式与 IDF 版内容一致）
4. 创建 `src/main.cpp`（见 4.1，极简）
5. 创建 `src/bsp/nm_display_28/config.h`（见 4.2，所有引脚/feature 定义）
6. 创建 `src/app/task_config.h`（见 4.6）
7. 创建 `src/drivers/devices/hal.h`、`hal.cpp`（新增 camera/audio 字段）
8. **验证**：`platformio run --target clean` 无报错

### Phase 2 — 显示 + 触控（约 3 天）
8. 从 NMMiner 复制 `src/drivers/display/hal.h`、`spiport.h`
9. 新建 `src/drivers/display/lvgl_port.h/cpp`（见 4.5，替代 esp_lvgl_port）
10. 新建 `src/drivers/display/lv_conf.h`（对照 IDF sdkconfig LVGL 项转换）
11. 新建 `src/drivers/touch/ft6336_port.h/cpp`
12. 新建 `src/bsp/nm_display_28/nm_display_28.cpp`（见 4.3）
13. 新建 `src/app/application.h/cpp`（见 4.7，暂时只 init 显示部分）
14. **验证**：LCD 点亮，LVGL hello world 显示，触控有响应

### Phase 3 — 传感器/PMU/RTC（约 1 天）
15. 迁移 `axp2101_port`（修改 init 签名，其余不变）
16. 迁移 `qmi8658_port`（同上）
17. 迁移 `pcf85063_port`（同上）
18. **验证**：三个传感器串口输出正常

### Phase 4 — 外设（约 1.5 天）
19. 新建 `sdcard_port`（见 4.11，约 20 行）
20. 新建 `wifi_port`（见 4.11，约 50 行）
21. 从 NMMiner 复制 `button/hal.h`、`onebutton_port.h`
22. **验证**：SD 卡挂载，WiFi 扫描串口输出正常，按键响应

### Phase 5 — 摄像头（约 0.5 天）
23. 新建 `src/drivers/camera/esp32cam_port.h/cpp`（见 4.8，改动 <20 行）
24. 迁移 `src/ui/tiles/camera_tile.cpp`（替换 `lvgl_port_lock` → `LVGL_LOCK`）
25. 在 `Application::begin()` 启动 `camera_task`（Core1）
26. **验证**：camera tile 显示摄像头画面，无崩溃

### Phase 6 — LVGL UI 整合（约 1.5 天）
27. 迁移所有 tileview → `src/ui/tiles/`（修改 include，替换 lvgl_port_lock）
28. 新建 `src/ui/ui_manager.h/cpp`（替代 lvgl_ui.cpp）
29. 在 `Application::begin()` 末尾调用 `UIManager::instance().init()`
30. **验证**：所有 tile 可以滑动切换，触控正常

### Phase 7 — 音频（约 3～5 天）
31. 新建 `src/drivers/audio/es8311_port.h/cpp`（见 4.10）
32. I2S 初始化（`#include <driver/i2s_std.h>`，代码几乎与 IDF 版相同）
33. ES8311 寄存器初始化（参考 esp_codec_dev 源码）
34. **验证**：录音/回放功能

---

## 第八部分：风险与特别注意

### 8.1 PSRAM 强制要求（违反会导致 StackOverflow 崩溃）

| 用途 | 大小 | 正确分配方式 |
|------|------|-------------|
| LVGL 帧缓冲 buf1 | 320×240÷4×2 = 38.4 KB | `ps_malloc()` |
| LVGL 帧缓冲 buf2 | 38.4 KB | `ps_malloc()` |
| 摄像头帧缓冲 ×2 | 320×480×2×2 ≈ 614 KB | 设置 `CAMERA_FB_IN_PSRAM`，库自动处理 |

### 8.2 LEDC Channel 冲突（必须检查）

| 外设 | LEDC Timer | LEDC Channel | GPIO |
|------|-----------|-------------|------|
| 摄像头 XCLK | TIMER_0 | **CH_0**（独占） | GPIO38 |
| LCD 背光 | TIMER_1 | **CH_1**（不得用 CH_0！） | GPIO6 |

### 8.3 迁移完成后的验证 grep 检查（Agent 执行）

```
# 以下字符串在 src/ 目录中应为 0 结果：
grep -r "lvgl_port_lock"        src/    # 必须为 0
grep -r "lvgl_port_unlock"      src/    # 必须为 0
grep -r "i2c_master_bus_handle" src/    # 必须为 0
grep -r "ESP_ERROR_CHECK"       src/    # 必须为 0
grep -r "nvs_flash_init"        src/    # 必须为 0
grep -r "idf_component.yml"     src/    # 必须为 0
grep -r "freertos/semphr.h"     src/    # 可出现，但只应在 lvgl_port.h 中
```

### 8.4 Arduino 框架可直接调用的 IDF API（无需包装）

```cpp
// 以下在 Arduino ESP32 框架中直接可用（底层即为 ESP-IDF）
#include <driver/i2s_std.h>          // ES8311 I2S（几乎零改动）
#include <driver/temperature_sensor.h>  // 片内温度传感器
#include <esp_camera.h>              // 摄像头（完全兼容）
#include <esp_psram.h>               // PSRAM 状态查询
#include <freertos/FreeRTOS.h>       // FreeRTOS 全套
#include <esp_task_wdt.h>            // 看门狗
#include <driver/ledc.h>             // LEDC（也可用 ledcSetup 高层 API）
```

---

*文档版本 2.0 · 基于 NMMiner 分层架构 · 2026-05-12*
