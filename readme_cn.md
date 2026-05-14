**中文 | [English](README.md)**

# NM-Display-2.8inch 工厂测试固件（Arduino 版）

本项目是 NM-Display 2.8 寸开发板（ESP32-S3）工厂测试固件的 **PlatformIO / Arduino 框架**移植版本，  
提供基于 LVGL 8.x 触控 UI 的 10 项顺序硬件自检流程。

> **注意：** 原 IDF 版本位于独立仓库，本仓库仅面向 **PlatformIO + Arduino 框架**。  
> 构建系统、库管理方式和驱动 API 均有较大差异，请勿混用。

---

## 目录

- [硬件概述](#硬件概述)
- [IO 引脚分配表](#io-引脚分配表)
- [环境与依赖](#环境与依赖)
- [工程结构](#工程结构)
- [编译与烧录](#编译与烧录)
- [工厂测试流程](#工厂测试流程)
- [测试项说明](#测试项说明)
- [移植指南](#移植指南)
- [注意事项](#注意事项)

---

## 硬件概述

![NM-Display-2.8 寸开发板](image/nm-display-28.png)

| 项目         | 规格                                    |
|--------------|-----------------------------------------|
| 主控         | ESP32-S3，240 MHz，双核                  |
| Flash        | 16 MB QIO                               |
| PSRAM        | Octal PSRAM（OPI 模式）                  |
| 显示屏       | 2.8 寸 ST7789，320×240，SPI 80 MHz      |
| 触摸         | FT6336（I2C 电容触摸）                   |
| 音频编解码   | ES8311（I2C 控制 + I2S 数据，16 kHz）   |
| IMU          | QMI8658（6 轴加速度/陀螺仪，I2C）        |
| PMU          | AXP2101（电源管理，I2C 0x34；本板 Chip ID = **0x47**，标准值为 0x4A）|
| RTC          | PCF85063（实时时钟，I2C）                |
| IO 扩展器    | TCA9554（I2C，地址 0x20）               |
| 摄像头       | DVP 接口（OV 系列），RGB565 320×480      |
| SD 卡        | SDMMC 1-bit 模式                        |
| Wi-Fi        | ESP32-S3 内置                           |
| 背光         | LEDC PWM（10-bit，5 kHz，CH1）          |
| 按钮         | BOOT 按钮，GPIO0，低电平有效             |

---

## IO 引脚分配表

### I2C 总线（共享）

| GPIO | 功能     |
|------|----------|
| IO7  | I2C SCL  |
| IO8  | I2C SDA  |

> 所有外设共用同一条 I2C 总线（400 kHz）：TCA9554（0x20）、FT6336（0x38）、AXP2101（0x34）、ES8311（0x18）、QMI8658（0x6A）、PCF85063（0x51）。

---

### LCD 显示屏（ST7789，SPI）

| GPIO | 功能           |
|------|----------------|
| IO1  | SPI MOSI       |
| IO5  | SPI SCLK       |
| IO3  | LCD DC         |
| IO6  | 背光（PWM）    |
| NC   | SPI CS（板载已接地） |
| NC   | LCD RST（由 TCA9554 IO1 控制） |

- SPI 时钟：80 MHz，Mode 3
- 旋转 90° 后逻辑分辨率：320×240

---

### 触摸屏（FT6336，I2C）

| GPIO | 功能    |
|------|---------|
| IO7  | I2C SCL |
| IO8  | I2C SDA |

- 复位：由 TCA9554 IO1 控制

---

### IO 扩展器（TCA9554，I2C 0x20）

| TCA9554 引脚 | 功能                          |
|--------------|-------------------------------|
| IO1          | LCD + 触摸模组复位（输出）     |
| IO7          | PA_CTRL — NS4150B 功放使能（输出）|

- 配置寄存器 0x03 = 0x7D（IO1 和 IO7 为输出，其余为输入）
- 输出寄存器 0x01：`0x82` = PA 开，`0x02` = PA 关

---

### 音频编解码（ES8311，I2C 0x18 + I2S）

| GPIO | 功能                  |
|------|-----------------------|
| IO7  | I2C SCL（控制）       |
| IO8  | I2C SDA（控制）       |
| IO12 | I2S MCLK（256 × SR） |
| IO13 | I2S BCLK              |
| IO15 | I2S LRCK              |
| IO16 | I2S DOUT（播放）      |
| IO14 | I2S DIN（录音）       |

- 采样率：16 000 Hz，16-bit 立体声，Philips/I2S 格式
- MCLK = 256 × 16 000 = 4 096 000 Hz
- 功放（NS4150B）由 TCA9554 IO7 控制

---

### IMU（QMI8658，I2C 0x6A）

| GPIO | 功能    |
|------|---------|
| IO7  | I2C SCL |
| IO8  | I2C SDA |

- 加速度计：±4G，ODR 1000 Hz
- 陀螺仪：±512 DPS，ODR 1000 Hz
- Mahony AHRS 融合算法（四元数输出）

---

### RTC（PCF85063，I2C 0x51）

- 若存储时间年份 < 2025，上电时自动重置为 2025-01-01 12:00:00

---

### PMU（AXP2101，I2C 0x34）

- 监测电池、VBUS、系统电压及各路 DC/LDO 输出
- 本板 Chip ID 上报为 `0x47`（驱动已处理，无需手动干预）

---

### 摄像头（DVP）

| GPIO      | 功能                         |
|-----------|------------------------------|
| IO38      | XCLK（20 MHz，LEDC CH0）    |
| IO17      | VSYNC                        |
| IO18      | HREF                         |
| IO41      | PCLK                         |
| IO45~IO21 | D0~D7                        |
| IO8       | SCCB SDA                     |
| IO7       | SCCB SCL                     |

---

### SD 卡（SDMMC 1-bit）

| GPIO | 功能       |
|------|------------|
| IO9  | SDMMC D0   |
| IO10 | SDMMC CMD  |
| IO11 | SDMMC CLK  |

---

### 按钮

| GPIO | 功能                  |
|------|-----------------------|
| IO0  | BOOT 按钮（低电平有效）|

---

## 环境与依赖

### 必要工具

| 工具            | 版本 / 说明                        |
|-----------------|------------------------------------|
| PlatformIO IDE  | VS Code 扩展或 CLI                 |
| Python          | 3.9 – 3.12（PlatformIO 依赖）      |
| Platform        | `espressif32 @ 6.6.0`              |
| 框架            | Arduino（通过 PlatformIO）         |

### 库依赖（PlatformIO 自动解析）

| 库          | 版本    | 说明                                        |
|-------------|---------|---------------------------------------------|
| lvgl/lvgl   | ^8.4.0  | 在 `platformio.ini` 的 `lib_deps` 中声明    |

> 其余所有驱动（ST7789、FT6336、ES8311、QMI8658、AXP2101、PCF85063、TCA9554、Camera）均以**源文件形式**内嵌在本仓库的 `src/drivers/` 和 `src/bsp/` 目录下，无需单独安装。

### 与 IDF 版本的主要差异

| 方面              | IDF 版本                            | 本版本（Arduino）                             |
|-------------------|-------------------------------------|-----------------------------------------------|
| 构建系统          | CMake + idf.py                      | PlatformIO（`pio run`）                       |
| 框架              | ESP-IDF v5.4.x                      | Arduino-ESP32（espressif32 @ 6.6.0）          |
| I2S 驱动 API      | `i2s_std`（IDF 5.x 新 API）         | 旧版 `driver/i2s.h`（IDF 4.4 风格）           |
| LVGL 移植层       | esp-lvgl-port 组件                  | 自定义 `lvgl_port.cpp`（位于 `src/drivers/display/`）|
| 组件管理          | idf_component_manager               | PlatformIO `lib_deps` + 本地源文件            |
| PSRAM 配置        | sdkconfig（menuconfig）             | `board_build.arduino.memory_type = qio_opi`   |
| 分区表            | `partitions.csv`（CMake 引用）      | `partitions/nm_display_28.csv`（platformio.ini 引用）|
| 摄像头驱动        | esp32-camera 组件                   | 源文件内嵌于仓库                              |

---

## 工程结构

```
NM-Display-28inch/
├── platformio.ini              # PlatformIO 项目配置（板卡、库、编译标志）
├── boards/
│   └── nm_display_28.json      # 自定义板卡定义（ESP32-S3 + OPI PSRAM）
├── partitions/
│   └── nm_display_28.csv       # 16 MB 分区表
│       # nvs 20KB | otadata 8KB | app0 4MB | app1 4MB | spiffs 6MB
├── data/                       # SPIFFS 资源（boot.gif、字体等）
├── scripts/
│   └── upload_all.py           # 编译后脚本：自动烧录固件 + SPIFFS
├── src/
│   ├── main.cpp                # Arduino setup() / loop()，初始化序列
│   ├── version.h               # APP_VERSION_STR "0.1.0"
│   ├── alg/
│   │   └── quaternion/         # Mahony AHRS、IMU 偏差、四元数数学
│   ├── app/
│   │   ├── application.cpp/.h  # 顶层应用：任务创建、外设初始化
│   │   └── task_config.h       # 核心分配、任务优先级与栈大小
│   ├── bsp/
│   │   └── nm_display_28/
│   │       ├── config.h        # 全部 GPIO、I2C 地址、SPI 频率定义
│   │       └── nm_display_28.cpp  # 板级初始化（I2C、SPI、I2S、SD、摄像头）
│   ├── drivers/
│   │   ├── devices/            # hal.cpp — 外设设备驱动
│   │   ├── display/
│   │   │   ├── lv_conf.h       # LVGL 配置（深度=16，swap=0，字体…）
│   │   │   ├── lvgl_port.cpp/.h  # LVGL Tick 任务 + flush 回调
│   │   │   └── spiport.h       # ST7789 SPI 写入工具函数
│   │   └── touch/
│   │       ├── hal.h           # 触摸 HAL 接口
│   │       └── iicport.h       # FT6336 I2C 读取
│   └── ui/
│       ├── boot_anim/          # 开机动画（GIF 播放）
│       └── factory_test/
│           ├── factory_test.h/.cpp  # 10 项顺序工厂测试 UI
│           └── fonts/          # Inconsolata_16、Inconsolata_20 点阵字体
```

---

## 编译与烧录

### 1. 安装 PlatformIO

```bash
# 通过 pip 安装（若未使用 VS Code 扩展）
pip install platformio
```

### 2. 编译固件

```bash
cd NM-Display-28inch
pio run -e nm-display-28
```

### 3. 一键烧录固件 + SPIFFS

项目包含编译后脚本（`scripts/upload_all.py`），固件烧录完成后自动构建并上传 SPIFFS 文件系统镜像。

```bash
pio run -e nm-display-28 --target upload --upload-port COMx
```

将 `COMx` 替换为实际串口（Windows 如 `COM48`，Linux 如 `/dev/ttyACM0`）。

> **提示：** 在 VS Code PlatformIO 扩展中点击「Upload」按钮，两步自动完成。

### 4. 分开烧录固件和 SPIFFS

```bash
# 仅烧录固件
pio run -e nm-display-28 --target upload --upload-port COMx

# 仅烧录 SPIFFS 文件系统
pio run -e nm-display-28 --target uploadfs --upload-port COMx
```

### 5. 串口监视器

```bash
pio device monitor -e nm-display-28 --port COMx
```

波特率：**115 200**。监视器过滤器包含 `esp32_exception_decoder` 和 `time` 前缀。

### 6. 分区表

| 分区     | 类型   | 偏移      | 大小   |
|----------|--------|-----------|--------|
| nvs      | data   | 0x009000  | 20 KB  |
| otadata  | data   | 0x00E000  | 8 KB   |
| app0     | ota_0  | 0x010000  | 4 MB   |
| app1     | ota_1  | 0x410000  | 4 MB   |
| spiffs   | spiffs | 0x810000  | 6 MB   |

---

## 工厂测试流程

上电后，设备播放开机动画（GIF），然后自动进入工厂测试序列。

```
上电
    ↓
开机动画（SPIFFS GIF）
    ↓
FactoryTest::run()
    ├─ T01  Display（显示）  — 颜色填充（R/G/B/W/K）+ PASS/FAIL
    ├─ T02  Touch（触摸）    — 绘制触摸轨迹，点击 OK 确认
    ├─ T03  SD Card（SD 卡） — 挂载 + 读取容量
    ├─ T04  Wi-Fi           — 扫描 + 连接（凭证在配置文件中）
    ├─ T05  IMU             — QMI8658 WHO_AM_I + 加速度/陀螺仪实时数据
    ├─ T06  PMU             — AXP2101 Chip ID + 各路电压
    ├─ T07  RTC             — PCF85063 探测 + 时间读写
    ├─ T08  Camera（摄像头） — DVP 采集 + 实时预览
    ├─ T09  Codec（编解码）  — ES8311 ID 校验 + 扫频（500~4 kHz）+ 欢乐颂
    └─ T10  Mic（麦克风）    — ES8311 ADC 录音 5 s + 自动回放
    ↓
概览页（每项 P/F/S）+ Reboot 按钮
```

每项测试顶部均有 **Failed / Ok** 标题栏按钮，测试员可随时手动覆盖自动判断结果。

---

## 测试项说明

| #  | 名称    | 通过判据                                                    |
|----|---------|-------------------------------------------------------------|
| 1  | Display | 5 种颜色（R/G/B/W/K）显示正常，测试员确认                   |
| 2  | Touch   | 触摸轨迹在屏幕上可见，测试员确认                             |
| 3  | SD Card | 挂载成功，容量 > 0                                          |
| 4  | Wi-Fi   | 成功连接 AP 并获取 IP 地址                                  |
| 5  | IMU     | WHO_AM_I = 0x05，加速度/陀螺仪数值合理（非零）              |
| 6  | PMU     | Chip ID = 0x47，各路电压在预期范围内                        |
| 7  | RTC     | I2C 探测成功，时间可写入并回读                              |
| 8  | Camera  | 成功采集帧并显示（非全黑图像）                              |
| 9  | Codec   | ID[FD]=0x83 + ID[FE]=0x11；扫频音和旋律可听                |
| 10 | Mic     | 通过模拟 MIC（ES8311 ADC）录音 5 s，回放声音可听            |

---

## 移植指南

### 修改引脚定义

所有 GPIO 编号集中定义在 `src/bsp/nm_display_28/config.h`，硬件变更只需编辑该文件。

### 修改 Wi-Fi 凭证

编辑 `src/app/application.cpp`（或 `wifi_init()` 所在位置）：

```cpp
wifi_connect("YOUR_SSID", "YOUR_PASSWORD");
```

### 添加新板型（BSP）

1. 创建 `src/bsp/<board_name>/config.h`，填写新的引脚定义。
2. 创建 `src/bsp/<board_name>/<board_name>.cpp`，实现板级初始化。
3. 在 `platformio.ini` 中新增 `[env:<board_name>]` 段。
4. 在新环境的 `build_src_filter` 中包含新 BSP，排除旧 BSP。

### LVGL 配置

编辑 `src/drivers/display/lv_conf.h`，本板关键配置：

```c
#define LV_COLOR_DEPTH     16
#define LV_COLOR_16_SWAP   0   // 本板 ST7789 不需要字节交换
#define LV_HOR_RES_MAX     320
#define LV_VER_RES_MAX     240
```

---

## 注意事项

- **AXP2101 Chip ID**：AXP2101 标准 Chip ID 为 `0x4A`，本板上报为 `0x47`。驱动层已同时兼容两个值，无需手动干预。
- **PSRAM**：本板使用 Octal PSRAM，`platformio.ini` 中 `board_build.arduino.memory_type = qio_opi` 为**必填项**，删除后会导致启动崩溃。
- **I2S 驱动**：本项目使用**旧版** `driver/i2s.h` API（IDF 4.4 风格），而非 IDF 5.x 新增的 `i2s_std` / `i2s_pdm` API。这是为了与 `platformio.ini` 中锁定的 Arduino-ESP32 框架版本保持兼容，属于有意为之。
- **LEDC 通道冲突**：摄像头 XCLK 使用 `LEDC_TIMER_0 / LEDC_CHANNEL_0`；背光 PWM 使用 `LEDC_CHANNEL_1`。不可共用通道。
- **TCA9554 IO7（PA_CTRL）**：功放（NS4150B）**不直接**连接 ESP32 GPIO，而是通过 TCA9554 IO7 控制。播放音频前必须向 TCA9554 输出寄存器写入使能值，否则无声。
- **ES8311 寄存器读取**：ES8311 **不支持**寄存器地址自动递增。Chip ID 寄存器 0xFD 和 0xFE 必须分两次单独 `requestFrom()` 读取，合并读取会导致 0xFE 返回 0xFF。
- **开机动画**：GIF 文件须上传至 SPIFFS 分区。若 SPIFFS 为空，开机动画跳过，直接进入工厂测试。
- **固件版本**：在 `src/version.h` 中定义，当前为 `APP_VERSION_STR "0.1.0"`。
