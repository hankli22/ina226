# INA226 Power Monitor Sensor Driver

ESP-IDF 组件 · INA226 功率监测传感器驱动

[![License](https://img.shields.io/badge/license-Apache%202.0-blue)](LICENSE)
[![Platform](https://img.shields.io/badge/platform-ESP32--C6%20%7C%20ESP32-green)](https://www.espressif.com/)

## 简介 / Overview

INA226 是 TI 公司的高精度数字功率监测芯片，通过 I2C 接口测量分流电阻压降，计算总线电压、电流和功率。本驱动封装了芯片的全部寄存器操作，支持连续/触发/低功耗模式、可配置 Alert 引脚、采样平均和转换时间调节。所有公开 API 均提供中英双语注释。

The INA226 from Texas Instruments is a high-accuracy digital power/current monitor.  It measures the voltage drop across a shunt resistor via I2C and derives bus voltage, current and power.  This driver wraps every register — continuous, triggered and low-power modes, configurable Alert pin, averaging and conversion-time tuning are all exposed.  Every public API is documented with Chinese and English comments.

## 硬件连接 / Wiring

```
    ESP32                     INA226
   ┌────────┐            ┌──────────┐
   │   SDA  ├────────────┤ SDA      │
   │   SCL  ├────────────┤ SCL      │
   │   3.3V ├────────────┤ VCC       │
   │   GND  ├────────────┤ GND      │
   │  (GPIO)├────────────┤ ALERT     │ (可选 / optional)
   │        │   ┌─Rshunt─┤ VIN+     │
   │        │   │        │ VIN-     │
   └────────┘   └────────┴──────────┘
```

| INA226 引脚 | 连接 |
|---|---|
| VCC | 3.3V |
| GND | GND |
| SDA | GPIO (由调用方指定) |
| SCL | GPIO (由调用方指定) |
| A0, A1 | GND → 地址 0x40 (可组合出 0x41, 0x44, 0x45) |
| ALERT | 可选, 任意 GPIO |
| VIN+ / VIN- | 分流电阻两端 (高侧或低侧均可) |

## 安装 / Installation

### ESP-IDF 项目

将本仓库克隆到项目的 `components/` 目录，或通过 `EXTRA_COMPONENT_DIRS` 引用：

```cmake
# 根 CMakeLists.txt / root CMakeLists.txt
set(EXTRA_COMPONENT_DIRS ${CMAKE_SOURCE_DIR}/../ina226)
```

```cmake
# src/CMakeLists.txt 或 main/CMakeLists.txt
idf_component_register(
    SRCS "main.cpp"
    REQUIRES driver ina226
)
```

```cpp
#include "ina226.h"
```

### PlatformIO

```ini
lib_deps =
    https://github.com/hankli22/ina226
```

## 快速开始 / Quick Start

```cpp
#include "ina226.h"

void app_main() {
    ina226_t sensor;

    // 初始化: I2C端口0, SDA=6, SCL=7, 地址=0x40
    // 分流电阻 10 mΩ, 预期最大电流 5 A
    int ret = ina226_init(&sensor, 0, 6, 7, 0x40, 0.01f, 5.0f);
    if (ret != 0) return;  // 检查错误 / check error

    // 验证芯片型号
    uint16_t mfr = ina226_read_mfr_id(&sensor);  // → 0x5449 "TI"
    uint16_t die = ina226_read_die_id(&sensor);  // → 0x2260

    while (1) {
        float bus_v = ina226_read_bus_voltage(&sensor, INA226_UNIT_V);
        float cur_a = ina226_read_current(&sensor, INA226_UNIT_A);
        float pow_w = ina226_read_power(&sensor, INA226_UNIT_W);

        printf("Bus=%.3fV  Current=%.3fA  Power=%.3fW\n", bus_v, cur_a, pow_w);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

## API 参考 / API Reference

### 初始化与复位

| 函数 | 说明 |
|---|---|
| `ina226_init()` | 初始化传感器 + 配置 I2C 总线 + 写入校准值 |
| `ina226_reset()` | 软件复位 (恢复上电默认值) |

### 配置寄存器

| 函数 | 说明 |
|---|---|
| `ina226_read_config()` | 读回完整配置寄存器 |
| `ina226_write_config()` | 批量写入配置寄存器 |
| `ina226_set_mode()` | 工作模式: 连续 / 触发 / 低功耗 |
| `ina226_set_averaging()` | 采样平均: 1 ~ 1024 次 |
| `ina226_set_bus_ct()` | 总线 ADC 转换时间: 140 µs ~ 8.2 ms |
| `ina226_set_shunt_ct()` | 分流 ADC 转换时间: 140 µs ~ 8.2 ms |

### Alert 告警

| 函数 | 说明 |
|---|---|
| `ina226_set_alert()` | 配置 ALERT 引脚触发条件与行为 |
| `ina226_read_alert()` | 读回屏蔽/使能寄存器 |
| `ina226_set_alert_limit()` | 写入告警阈值 (原始寄存器值) |
| `ina226_set_alert_limit_amps()` | 写入告警阈值 (物理安培, 自动换算) |
| `ina226_alert_asserted()` | 查询 ALERT 引脚是否已断言 |
| `ina226_conversion_ready()` | 查询触发模式下的单次转换是否完成 |

### 读取测量值

| 函数 | 说明 |
|---|---|
| `ina226_read_bus_voltage()` | 总线电压 (可选 mV / V) |
| `ina226_read_shunt_voltage()` | 分流电压 (可选 mV / V) |
| `ina226_read_current()` | 电流 (可选 µA / mA / A) |
| `ina226_read_power()` | 功率 (可选 mW / W) |
| `ina226_read_*_raw()` | 原始 ADC 寄存器值 (调试用) |

### 芯片识别

| 函数 | 预期值 |
|---|---|
| `ina226_read_mfr_id()` | `0x5449` |
| `ina226_read_die_id()` | `0x2260` |

### 单位枚举

| 枚举 | 说明 |
|---|---|
| `INA226_UNIT_MV` | 毫伏 (×10³) |
| `INA226_UNIT_V` | 伏特 |
| `INA226_UNIT_UA` | 微安 (×10⁶) |
| `INA226_UNIT_MA` | 毫安 (×10³) |
| `INA226_UNIT_A` | 安培 |
| `INA226_UNIT_MW` | 毫瓦 (×10³) |
| `INA226_UNIT_W` | 瓦特 |

### Alert 标志位

| 宏 | 说明 |
|---|---|
| `INA226_ALERT_SOL` | 分流过压告警 |
| `INA226_ALERT_SUL` | 分流欠压告警 |
| `INA226_ALERT_BOL` | 总线过压告警 |
| `INA226_ALERT_BUL` | 总线欠压告警 |
| `INA226_ALERT_POL` | 功率超限告警 |
| `INA226_ALERT_CNVR` | 转换就绪 |
| `INA226_ALERT_LATCH` | 锁存模式 |
| `INA226_ALERT_POL_INVERT` | Alert 极性反转 |

## 低功耗示例 / Low-Power Example

```cpp
// 进入低功耗待机
ina226_set_mode(&sensor, INA226_MODE_POWER_DOWN);

// 需要测量时触发一次转换
ina226_set_mode(&sensor, INA226_MODE_SHUNT_BUS_TRIG);
while (!ina226_conversion_ready(&sensor)) {
    vTaskDelay(pdMS_TO_TICKS(1));
}
// 读取数据...
float bus_v = ina226_read_bus_voltage(&sensor, INA226_UNIT_V);

// 读取后自动回到 POWER_DOWN — 也可显式切回
ina226_set_mode(&sensor, INA226_MODE_POWER_DOWN);
```

## 告警示例 / Alert Example

```cpp
// 电流超过 1.5A 时 ALERT 拉低 (锁存)
ina226_set_alert(&sensor, INA226_ALERT_SOL | INA226_ALERT_LATCH);
ina226_set_alert_limit_amps(&sensor, 1.5f);

// 轮询
if (ina226_alert_asserted(&sensor)) {
    printf("Over-current!\n");
    ina226_read_alert(&sensor);  // 读寄存器清除锁存
}
```

## 编译选项 / Build Flags

可修改 I2C 时钟频率:

```ini
build_flags = -DINA226_I2C_FREQ=400000
```

默认 100 kHz。INA226 支持 Standard (100k)、Fast (400k) 和 High-Speed (2.94M)。

## 许可 / License

Apache 2.0 © 2026 hankli22

---

See also: [TI INA226 Datasheet (SBOS547F)](https://www.ti.com/lit/ds/symlink/ina226.pdf)
