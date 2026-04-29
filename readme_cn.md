# NM-Display-2.8inch 全功能验证固件

本项目是针对 **NM-Display 2.8 寸开发板**（ESP32-S3）的全板硬件功能覆盖测试固件，用于出厂验证和功能联调。

---

## 目录

- [硬件概述](#硬件概述)
- [引脚图](#引脚图)
- [IO 引脚分配表](#io-引脚分配表)
- [依赖环境](#依赖环境)
- [工程结构](#工程结构)
- [工作流程](#工作流程)
- [UI 界面说明](#ui-界面说明)
- [移植指南](#移植指南)

---

## 硬件概述

| 项目       | 规格                         |
| ---------- | ---------------------------- |
| 主控       | ESP32-S3                     |
| 显示屏     | 2.8 寸 ST7789，320×240       |
| 触摸       | FT6336（I2C 电容触摸）       |
| PMU        | AXP2101（电源管理）          |
| 音频编解码 | ES8311（I2S 音频 Codec）     |
| IMU        | QMI8658（6 轴加速度/陀螺仪） |
| RTC        | PCF85063（实时时钟）         |
| IO 扩展    | TCA9554（I2C GPIO 扩展）     |
| 摄像头     | DVP 接口（OV 系列）          |
| SD 卡      | SDMMC 1-bit 模式             |
| Wi-Fi      | ESP32-S3 内置                |
| 背光       | LEDC PWM 调光                |

---

## 引脚图

![NM-Display-2.8 引脚图](image/nm-display-28.png)

---

## IO 引脚分配表

### I2C 总线（共享）

| GPIO | 功能    |
| ---- | ------- |
| IO7  | I2C SCL |
| IO8  | I2C SDA |

> 以下外设均挂载在同一 I2C 总线（I2C_NUM_0）上：TCA9554、FT6336、AXP2101、ES8311（控制）、QMI8658、PCF85063。

---

### LCD 显示屏（ST7789，SPI2）

| GPIO | 功能           |
| ---- | -------------- |
| IO1  | SPI MOSI       |
| IO5  | SPI SCLK       |
| IO3  | LCD DC         |
| IO6  | 背光 BL（PWM） |
| NC   | SPI CS         |
| NC   | LCD RESET      |

- 接口：SPI，时钟频率 80 MHz，SPI Mode 3
- 驱动：ST7789
- 分辨率：320×240（旋转 270° 时逻辑宽高为 320×240）

---

### 触摸屏（FT6336，I2C）

| GPIO | 功能     |
| ---- | -------- |
| IO7  | I2C SCL  |
| IO8  | I2C SDA  |
| NC   | TP INT   |
| NC   | TP RESET |

- 通过 TCA9554 IO 扩展器 PIN_1 控制触摸模组复位

---

### IO 扩展器（TCA9554，I2C）

| GPIO | 功能                       |
| ---- | -------------------------- |
| IO7  | I2C SCL                    |
| IO8  | I2C SDA                    |
| —    | I2C 地址：TCA9554_ADDR_000 |

- PIN_1：LCD/触摸模组复位（上电时先拉低再拉高）

---

### 音频 Codec（ES8311，I2C 控制 + I2S 数据）

| GPIO | 功能             |
| ---- | ---------------- |
| IO7  | I2C SCL（控制）  |
| IO8  | I2C SDA（控制）  |
| IO12 | I2S MCLK         |
| IO13 | I2S BCLK         |
| IO15 | I2S LRCK         |
| IO14 | I2S DIN（录音）  |
| IO16 | I2S DOUT（播放） |

- 采样率：16 kHz，16 bit，立体声
- 工作模式：全双工（录音 + 播放）

---

### IMU（QMI8658，I2C）

| GPIO | 功能    |
| ---- | ------- |
| IO7  | I2C SCL |
| IO8  | I2C SDA |

- 加速度量程：±4G，采样率 1000 Hz
- 陀螺仪量程：±512 DPS，采样率 1000 Hz
- I2C 地址：QMI8658_L_SLAVE_ADDRESS

---

### RTC（PCF85063，I2C）

| GPIO | 功能    |
| ---- | ------- |
| IO7  | I2C SCL |
| IO8  | I2C SDA |

- I2C 地址：PCF85063_SLAVE_ADDRESS
- 上电自动检查时间合法性，若年份早于 2025 则重置为 2025-01-01 12:00:00

---

### PMU（AXP2101，I2C）

| GPIO | 功能    |
| ---- | ------- |
| IO7  | I2C SCL |
| IO8  | I2C SDA |

- I2C 地址：0x34
- 监测：充电状态、电池电压、VBUS 电压、系统电压、各路 DC/LDO 电压

---

### 摄像头（DVP 接口）

| GPIO | 功能                      |
| ---- | ------------------------- |
| IO38 | XCLK                      |
| IO17 | VSYNC                     |
| IO18 | HREF                      |
| IO41 | PCLK                      |
| IO45 | D0 (Y2)                   |
| IO47 | D1 (Y3)                   |
| IO48 | D2 (Y4)                   |
| IO46 | D3 (Y5)                   |
| IO42 | D4 (Y6)                   |
| IO40 | D5 (Y7)                   |
| IO39 | D6 (Y8)                   |
| IO21 | D7 (Y9)                   |
| IO8  | SCCB SDA（复用 I2C 总线） |
| IO7  | SCCB SCL（复用 I2C 总线） |

- XCLK 频率：20 MHz
- 帧格式：RGB565，320×480

---

### SD 卡（SDMMC 1-bit）

| GPIO | 功能      |
| ---- | --------- |
| IO9  | SDMMC D0  |
| IO10 | SDMMC CMD |
| IO11 | SDMMC CLK |

- 挂载点：`/sdcard`
- 模式：1-bit SDMMC

---

### 按键

| GPIO | 功能               |
| ---- | ------------------ |
| IO0  | BOOT 按键（低有效）|

---

## 依赖环境

| 工具 / 框架              | 版本要求                               |
| ------------------------ | -------------------------------------- |
| ESP-IDF                  | v5.4.3（推荐）                         |
| LVGL                     | v8.x（通过 managed_components）        |
| esp-lvgl-port            | Espressif 官方组件                     |
| esp_io_expander_tca9554  | Espressif 官方组件                     |
| esp_lcd_touch（FT6336）  | Espressif 官方组件                     |
| iot_button               | Espressif 官方组件                     |
| esp_codec_dev            | Espressif 官方组件                     |
| XPowersLib               | 本地 components 目录                   |
| sensorlib                | 本地 components 目录（QMI8658、PCF85063）|
| esp32-camera             | 本地 components 目录                   |

### 配置 ESP-IDF 环境

```bash
# Windows (PowerShell)
. $env:IDF_PATH\export.ps1

# Linux / macOS
. $IDF_PATH/export.sh
```

---

## 工程结构

```
NM-Display-28inch/
├── main/
│   ├── main.cpp                      # 主入口，硬件初始化流程 + 触摸测试
│   └── Kconfig.projbuild             # PMU 类型等 menuconfig 选项
├── components/
│   ├── esp_port/                     # 各外设驱动封装层
│   │   ├── esp_3inch5_lcd_port.cpp   # LCD SPI + 背光 PWM
│   │   ├── esp_axp2101_port.cpp      # PMU
│   │   ├── esp_camera_port.cpp       # 摄像头
│   │   ├── esp_es8311_port.cpp       # 音频 Codec
│   │   ├── esp_pcf85063_port.cpp     # RTC
│   │   ├── esp_qmi8658_port.cpp      # IMU
│   │   ├── esp_sdcard_port.cpp       # SD 卡
│   │   └── esp_wifi_port.cpp         # Wi-Fi
│   ├── lvgl_ui/                      # LVGL UI 主界面
│   │   └── tileview/                 # 各功能测试 tile 页面
│   ├── esp_lcd_st7789/               # ST7789 驱动
│   ├── esp_lcd_st7796/               # ST7796 驱动（备用）
│   ├── esp_lcd_touch_ft6336/         # FT6336 触摸驱动
│   ├── esp32-camera/                 # ESP32 摄像头驱动
│   ├── XPowersLib/                   # AXP2101 PMU 库
│   └── sensorlib/                    # QMI8658、PCF85063 等传感器库
├── managed_components/               # 通过 idf_component_manager 管理的组件
├── partitions.csv                    # 分区表
├── sdkconfig.defaults                # 默认配置（PSRAM、Flash 等）
└── CMakeLists.txt
```

---

## 工作流程

上电后 `app_main` 按以下顺序执行：

```
1. NVS Flash 初始化
        ↓
2. I2C 总线初始化（IO7/IO8，I2C_NUM_0）
        ↓
3. IO 扩展器 TCA9554 初始化
   └─ PIN_1 拉低 → 延时 100ms → 拉高（复位 LCD/Touch 模组）
        ↓
4. LCD 显示屏初始化（ST7789，SPI2，80MHz）
        ↓
5. 触摸屏初始化（FT6336，I2C）
        ↓
6. PMU 初始化（AXP2101，I2C 地址 0x34）
        ↓
7. 音频 Codec 初始化（ES8311，I2S + I2C）
        ↓
8. IMU 初始化（QMI8658，I2C）
   └─ 自检加速度计和陀螺仪
        ↓
9. RTC 初始化（PCF85063，I2C）
   └─ 若时间非法则重置为 2025-01-01 12:00:00
        ↓
10. SD 卡初始化（SDMMC 1-bit，挂载至 /sdcard）
        ↓
11. 摄像头初始化（DVP，RGB565，320×480）
        ↓
12. Wi-Fi 初始化（Station 模式，连接指定 AP）
        ↓
13. 背光初始化并设置亮度 80%
        ↓
14. LVGL Port 初始化（注册 LCD + Touch 设备）
        ↓
15. 按键初始化（IO0，BOOT 键，单击退出触摸测试）
        ↓
16. 触摸测试模式
    ├─ 屏幕显示提示文字
    ├─ 循环读取触摸坐标，在触点位置绘制红色方块
    └─ 按下 BOOT 键退出触摸测试
        ↓
17. 启动 LVGL UI（多页 TileView 功能展示）
```

---

## UI 界面说明

LVGL UI 采用横向滑动的 **TileView** 结构，共 6 个页面，左右滑动切换：

| 页面序号 | 名称     | 功能说明                                                                                                         |
| -------- | -------- | ---------------------------------------------------------------------------------------------------------------- |
| 0        | RGB 测试 | 屏幕背景色每秒在红/绿/蓝之间循环切换，验证 LCD 显示正常                                                          |
| 1        | 系统信息 | 显示 Flash 大小、PSRAM 大小、芯片温度、主频、SD 卡容量、RTC 时间日期；提供背光亮度滑块；ES8311 录播测试按钮       |
| 2        | AXP2101  | 实时显示 PMU 状态：充电状态、电池连接、VBUS 输入、电池/VBUS/系统电压、各路 DC/LDO 输出电压                       |
| 3        | QMI8658  | 实时显示 IMU 数据：三轴加速度（mg）、三轴陀螺仪（dps）及 IMU 温度                                                |
| 4        | 摄像头   | 实时预览摄像头画面（RGB565，320×480），帧数据通过 LVGL 图像控件渲染                                               |
| 5        | Wi-Fi    | 显示当前 IP 地址；提供 Wi-Fi 开关和 Scan 按钮，扫描并列出附近 AP                                                  |

---

## 移植指南

### 1. 修改 Wi-Fi 连接信息

在 `main/main.cpp` 中修改 SSID 和密码：

```cpp
esp_wifi_port_init("你的SSID", "你的密码");
```

### 2. 修改显示旋转方向

在 `main/main.cpp` 顶部修改：

```cpp
#define EXAMPLE_DISPLAY_ROTATION 270  // 可选：0 / 90 / 180 / 270
```

`lv_port_init()` 中已针对 4 个方向预设好 swap_xy / mirror_x / mirror_y 参数，无需额外修改。

### 3. 适配其他同类板子

| 需修改内容      | 文件                                          |
| --------------- | --------------------------------------------- |
| LCD 引脚 / 驱动 | `components/esp_port/esp_3inch5_lcd_port.cpp` |
| 触摸引脚 / 驱动 | `components/esp_port/esp_3inch5_lcd_port.cpp` |
| 摄像头引脚      | `components/esp_port/esp_camera_port.cpp`     |
| SD 卡引脚       | `components/esp_port/esp_sdcard_port.cpp`     |
| 音频 I2S 引脚   | `components/esp_port/esp_es8311_port.cpp`     |
| I2C 引脚        | `main/main.cpp`（`EXAMPLE_PIN_I2C_SDA/SCL`） |

### 4. 编译与烧录

```bash
# 配置（可选，修改 PMU 类型等）
idf.py menuconfig

# 编译
idf.py build

# 烧录并监视串口（替换为实际串口号）
idf.py -p COM3 flash monitor
```

### 5. 分区表

项目使用自定义 `partitions.csv`，OTA 和 NVS 分区已预留，烧录时会自动使用该分区表。

---

## 注意事项

- **AXP2101 Chip ID**：本板 AXP2101 报告的 Chip ID 为 `0x47`（非标准 `0x4A`），已在 `esp_axp2101_port.cpp` 中通过宏覆盖处理，无需修改。
- **触摸测试**：上电后必须先完成触摸测试（按 BOOT 键退出）才会进入 LVGL UI 主界面。
- **摄像头**：预览页采用独立 FreeRTOS Task 拉取帧，通过带超时的 LVGL lock 避免阻塞 UI 渲染。
- **SD 卡**：挂载失败不阻塞启动，系统信息页显示容量为 0 即表示未挂载或未插卡。