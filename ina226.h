/*
 * Copyright (c) 2026 hankli22
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * ╔══════════════════════════════════════════════════════════════════════════════╗
 * ║                    INA226 功率监测传感器驱动 · 公共接口                       ║
 * ║                 INA226 Power Monitor Sensor Driver · Public API              ║
 * ╚══════════════════════════════════════════════════════════════════════════════╝
 *
 * 【概述 / Overview】
 *   INA226 是 TI 公司的高精度数字功率监测芯片，通过 I2C 接口读取分流电阻上
 *   的电压降，进而计算总线电压、电流和功率。本驱动封装了芯片全部寄存器操作，
 *   支持连续/触发/低功耗模式、可配置 Alert 引脚、采样平均和转换时间调节。
 *
 *   INA226 is a high-accuracy digital power monitor from TI.  It measures the
 *   voltage drop across a shunt resistor via I2C and derives bus voltage,
 *   current and power.  This driver wraps every register — continuous,
 *   triggered and low-power modes, configurable Alert pin, averaging and
 *   conversion-time tuning are all exposed.
 *
 * 【硬件连接 / Wiring】
 *       ESP32-C6                INA226
 *      ┌────────┐            ┌──────────┐
 *      │   SDA  ├────────────┤ SDA      │
 *      │   SCL  ├────────────┤ SCL      │
 *      │   3.3V ├────────────┤ VCC       │
 *      │   GND  ├────────────┤ GND      │
 *      │  3.3V  ├─┬──────────┤ A0, A1   │  地址 = 0x40 (两脚均接 GND)
 *      │        │ └─10kΩ─GND │          │  可通过 A0/A1 接 VCC 修改为
 *      │        │            │          │  0x41, 0x44, 0x45
 *      │        │   ┌─Rshunt─┤ VIN+     │  负载高侧 / 低侧均可
 *      │        │   │        │ VIN-     │  (High- or low-side sensing)
 *      │        │   │        │ ALERT ───│→ GPIO (可选 / optional)
 *      └────────┘   └────────┴──────────┘
 *
 * 【典型用法 / Typical Usage】
 *
 *   ina226_t sensor;
 *   int ret = ina226_init(&sensor, 0, SDA, SCL, 0x40, 0.01f, 5.0f);
 *   if (ret < 0) { /* handle error */ }
 *
 *   // 设置告警: 电流超过 1A 时 ALERT 脚拉低 (锁存)
 *   // Set alert: latch ALERT pin low when current exceeds 1 A
 *   ina226_set_alert(&sensor,
 *       INA226_ALERT_SOL | INA226_ALERT_LATCH);
 *   ina226_set_alert_limit_amps(&sensor, 1.0f);
 *
 *   // 低功耗待机, 需要时触发一次转换
 *   // Low-power standby, trigger a one-shot conversion when needed
 *   ina226_set_mode(&sensor, INA226_MODE_POWER_DOWN);
 *   // ... later ...
 *   ina226_set_mode(&sensor, INA226_MODE_SHUNT_BUS_TRIG);
 *   while (!ina226_conversion_ready(&sensor)) { vTaskDelay(1); }
 *
 *   float bus_v = ina226_read_bus_voltage(&sensor, INA226_UNIT_V);
 *   float cur_a = ina226_read_current(&sensor, INA226_UNIT_A);
 *   float pow_w = ina226_read_power(&sensor, INA226_UNIT_W);
 *
 *   // 芯片识别 / Chip identification
 *   uint16_t mfr = ina226_read_mfr_id(&sensor);   // → 0x5449 "TI"
 *   uint16_t die = ina226_read_die_id(&sensor);   // → 0x2260
 */

#ifndef INA226_H
#define INA226_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ══════════════════════════════════════════════════════════════════════════════
 *  单位枚举 / Unit enumeration
 *
 *  所有物理量读取函数均接受此枚举决定输出量纲。
 *  Every physical reading function accepts this enum to choose the output unit.
 * ══════════════════════════════════════════════════════════════════════════ */

typedef enum {
    INA226_UNIT_MV,    // 毫伏     millivolts   (×10³)
    INA226_UNIT_V,     // 伏       volts        (SI base)
    INA226_UNIT_UA,    // 微安     microamps    (×10⁶)
    INA226_UNIT_MA,    // 毫安     milliamps    (×10³)
    INA226_UNIT_A,     // 安培     amps         (SI base)
    INA226_UNIT_MW,    // 毫瓦     milliwatts   (×10³)
    INA226_UNIT_W      // 瓦       watts        (SI base)
} ina226_unit_t;

/* ══════════════════════════════════════════════════════════════════════════════
 *  工作模式 / Operating mode — 对应于配置寄存器 bit [2:0]
 *  Corresponds to Configuration register bits 2–0.
 *
 *  ┌──────────────────────────┬─────┬────────────────────────────────┐
 *  │ 模式 Mode                │ 值  │ 说明 Description               │
 *  ├──────────────────────────┼─────┼────────────────────────────────┤
 *  │ POWER_DOWN               │ 0x0 │ 低功耗待机, 内部电路断电       │
 *  │ SHUNT_TRIG               │ 0x1 │ 触发一次分流电压转换后待机     │
 *  │ BUS_TRIG                 │ 0x2 │ 触发一次总线电压转换后待机     │
 *  │ SHUNT_BUS_TRIG           │ 0x3 │ 触发分流+总线各一次后待机      │
 *  │ ADC_OFF                  │ 0x4 │ ADC 关闭 (保留配置)            │
 *  │ SHUNT_CONT               │ 0x5 │ 连续测量分流电压               │
 *  │ BUS_CONT                 │ 0x6 │ 连续测量总线电压               │
 *  │ SHUNT_BUS_CONT (默认)    │ 0x7 │ 连续测量分流+总线 (推荐)       │
 *  └──────────────────────────┴─────┴────────────────────────────────┘
 * ══════════════════════════════════════════════════════════════════════════ */

typedef enum {
    INA226_MODE_POWER_DOWN     = 0x0,
    INA226_MODE_SHUNT_TRIG     = 0x1,
    INA226_MODE_BUS_TRIG       = 0x2,
    INA226_MODE_SHUNT_BUS_TRIG = 0x3,
    INA226_MODE_ADC_OFF        = 0x4,
    INA226_MODE_SHUNT_CONT     = 0x5,
    INA226_MODE_BUS_CONT       = 0x6,
    INA226_MODE_SHUNT_BUS_CONT = 0x7
} ina226_mode_t;

/* ══════════════════════════════════════════════════════════════════════════════
 *  采样平均次数 / Averaging — 对应于配置寄存器 bit [14:12]
 *  Corresponds to Configuration register bits 14–12.
 *
 *  更多平均次数降低噪声但延长有效更新间隔。
 *  More averages reduce noise at the cost of longer effective update interval.
 * ══════════════════════════════════════════════════════════════════════════ */

typedef enum {
    INA226_AVG_1    = 0x0,    //   1 次 (无平均 / no averaging)
    INA226_AVG_4    = 0x1,    //   4 次
    INA226_AVG_16   = 0x2,    //  16 次
    INA226_AVG_64   = 0x3,    //  64 次
    INA226_AVG_128  = 0x4,    // 128 次
    INA226_AVG_256  = 0x5,    // 256 次
    INA226_AVG_512  = 0x6,    // 512 次
    INA226_AVG_1024 = 0x7     // 1024 次
} ina226_avg_t;

/* ══════════════════════════════════════════════════════════════════════════════
 *  转换时间 / Conversion time — 配置寄存器 bit [11:9] (Bus), bit [8:6] (Shunt)
 *  Config register bits 11–9 for bus voltage, bits 8–6 for shunt voltage.
 *
 *  更长时间 = 更高精度 + 更低噪声, 但单次转换更慢。
 *  Longer time = higher precision + lower noise, but slower per conversion.
 * ══════════════════════════════════════════════════════════════════════════ */

typedef enum {
    INA226_CT_140US  = 0x0,   //  140 µs
    INA226_CT_204US  = 0x1,   //  204 µs
    INA226_CT_332US  = 0x2,   //  332 µs
    INA226_CT_588US  = 0x3,   //  588 µs
    INA226_CT_1100US = 0x4,   // 1100 µs (1.1 ms, 默认 / default)
    INA226_CT_2116US = 0x5,   // 2116 µs
    INA226_CT_4156US = 0x6,   // 4156 µs
    INA226_CT_8244US = 0x7    // 8244 µs
} ina226_ct_t;

/* ══════════════════════════════════════════════════════════════════════════════
 *  Alert 标志位 / Alert flags — 对应于 屏蔽/使能寄存器 (0x06)
 *  Corresponds to Mask/Enable register (0x06).
 *
 *  多个标志位通过按位或 (|) 组合后传入 ina226_set_alert()。
 *  Combine flags with bitwise-OR and pass to ina226_set_alert().
 *
 *  ┌─────────────────────┬──────┬────────────────────────────────────────┐
 *  │ 宏 Macro             │ 位   │ 说明 Description                       │
 *  ├─────────────────────┼──────┼────────────────────────────────────────┤
 *  │ SOL                 │ bit15│ 分流过压报警 Shunt over-limit          │
 *  │ SUL                 │ bit14│ 分流欠压报警 Shunt under-limit         │
 *  │ BOL                 │ bit13│ 总线过压报警 Bus over-limit            │
 *  │ BUL                 │ bit12│ 总线欠压报警 Bus under-limit           │
 *  │ POL                 │ bit11│ 功率超限报警 Power over-limit          │
 *  │ CNVR                │ bit10│ 转换就绪标志 Conversion ready          │
 *  │ AFF                 │ bit4 │ Alert 有效标志 (只读) Alert function flag│
 *  │ MATH_OVF            │ bit3 │ 算术溢出 (只读) Math overflow flag     │
 *  │ LATCH               │ bit1 │ 锁存模式: 断言后保持直到读取 Mask 寄存器│
 *  │ POL_INVERT          │ bit0 │ 极性反转: ALERT 脚高电平有效 (默认低)  │
 *  └─────────────────────┴──────┴────────────────────────────────────────┘
 * ══════════════════════════════════════════════════════════════════════════ */

#define INA226_ALERT_SOL         (1u << 15)
#define INA226_ALERT_SUL         (1u << 14)
#define INA226_ALERT_BOL         (1u << 13)
#define INA226_ALERT_BUL         (1u << 12)
#define INA226_ALERT_POL         (1u << 11)
#define INA226_ALERT_CNVR        (1u << 10)
#define INA226_ALERT_AFF         (1u << 4)
#define INA226_ALERT_MATH_OVF    (1u << 3)
#define INA226_ALERT_LATCH       (1u << 1)
#define INA226_ALERT_POL_INVERT  (1u << 0)

/* ══════════════════════════════════════════════════════════════════════════════
 *  设备句柄 / Device handle
 *
 *  调用方分配此结构体并通过 ina226_init() 初始化。
 *  Caller allocates this struct and initializes it via ina226_init().
 *  ─────────────────────────────────────────────────────────────────────────
 *  字段 Field       │ 说明 Description
 *  ─────────────────┼──────────────────────────────────────────────────────
 *   i2c_port        │ I2C 外设编号 (0 或 1) / I2C peripheral index
 *   addr            │ 7-bit I2C 地址 (0x40–0x4F) / 7-bit I2C address
 *   shunt_res       │ 分流电阻值, 单位 Ω (例如 0.01 = 10 mΩ)
 *   max_current     │ 预期最大电流, 单位 A (用于计算 LSB)
 *   current_lsb     │ 计算得出的电流 LSB, 单位 A/bit (由 init 自动填充)
 *   initialized     │ 初始化成功标志 / true after successful init
 *  ─────────────────┴──────────────────────────────────────────────────────
 * ══════════════════════════════════════════════════════════════════════════ */

typedef struct {
    int     i2c_port;
    uint8_t addr;
    float   shunt_res;
    float   max_current;
    float   current_lsb;
    bool    initialized;
} ina226_t;

/*
 * I2C 总线时钟频率, 可在编译时通过 build_flags = -DINA226_I2C_FREQ=400000 覆盖。
 * I2C bus clock frequency; override at build time if needed.
 * INA226 支持 Standard (100k), Fast (400k) 和 High-Speed (2.94M) 模式。
 */
#ifndef INA226_I2C_FREQ
#define INA226_I2C_FREQ 100000
#endif

/* ══════════════════════════════════════════════════════════════════════════════
 *  初始化 / 复位 — Init / Reset
 * ══════════════════════════════════════════════════════════════════════════ */

/*
 * 初始化 INA226 传感器并自动配置底层 I2C 总线。
 * Initialize the INA226 sensor and auto-configure the underlying I2C bus.
 *
 * 参数 Parameters:
 *   dev         — 设备句柄指针 / pointer to device handle
 *   i2c_port    — I2C 外设编号, 0 或 1 / I2C peripheral number
 *   sda, scl    — GPIO 引脚号 / GPIO pin numbers
 *   addr        — 7-bit I2C 地址 (默认 0x40, A0=A1=GND)
 *   shunt_res   — 分流电阻值, 单位 Ω (例如 0.01f = 10 mΩ)
 *   max_current — 预期最大电流, 单位 A (用于计算校准值)
 *
 * 返回值 Return:
 *    0 — 成功 / success
 *   -1 — dev 为 NULL / dev is NULL
 *   -2 — shunt_res 或 max_current 无效 (≤ 0)
 *   -3 — I2C 通信失败, 芯片未响应 (检查接线)
 *
 * 副作用 Side effects:
 *   首次调用时初始化对应 I2C 总线 (仅一次)。
 *   First call on a given port initializes the I2C bus (once only).
 *   写入校准寄存器并设置模式为 SHUNT_BUS_CONT。
 *   Writes calibration register and sets mode to SHUNT_BUS_CONT.
 */
int  ina226_init(ina226_t *dev, int i2c_port, int sda, int scl,
                 uint8_t addr, float shunt_res, float max_current);

/*
 * 软件复位 — 等同于上电复位, 所有寄存器恢复默认值。
 * Software reset — equivalent to power-on reset.  All registers restored to defaults.
 * 复位后需重新调用 ina226_init() 配置校准值。
 * Must re-call ina226_init() after reset to restore calibration.
 */
void ina226_reset(ina226_t *dev);

/* ══════════════════════════════════════════════════════════════════════════════
 *  配置寄存器 / Configuration register (0x00)
 * ══════════════════════════════════════════════════════════════════════════ */

/*
 * 读回完整的 16-bit 配置寄存器原始值 (调试用)。
 * Read back the full 16-bit configuration register value (for debugging).
 * 失败返回 0 (同时 I2C 出错时读值也接近 0, 建议配合 init 返回值判断)。
 */
uint16_t ina226_read_config(ina226_t *dev);

/*
 * 批量写入配置寄存器。调用方必须自行按数据手册拼装位域。
 * Bulk-write configuration register.  Caller must assemble bitfields per datasheet.
 * 多数场景下推荐使用下面的独立 setter 函数。
 * In most cases prefer the individual setter functions below.
 */
void     ina226_write_config(ina226_t *dev, uint16_t cfg);

/*
 * 设置工作模式 (连续/触发/低功耗)。
 * Set operating mode (continuous / triggered / power-down).
 * 常用技巧: 触发模式下先切到 SHUNT_BUS_TRIG,
 *         然后轮询 ina226_conversion_ready() 等待完成再读取。
 * Tip: in triggered mode switch to SHUNT_BUS_TRIG, then poll
 *      ina226_conversion_ready() before reading.
 */
void     ina226_set_mode(ina226_t *dev, ina226_mode_t mode);

/*
 * 设置采样平均窗口。
 * Set averaging window.
 * 例 Example: ina226_set_averaging(&dev, INA226_AVG_16);  // 16次平均
 */
void     ina226_set_averaging(ina226_t *dev, ina226_avg_t avg);

/*
 * 设置总线电压 ADC 转换时间。
 * Set bus-voltage ADC conversion time.
 */
void     ina226_set_bus_ct(ina226_t *dev, ina226_ct_t ct);

/*
 * 设置分流电压 ADC 转换时间。
 * Set shunt-voltage ADC conversion time.
 */
void     ina226_set_shunt_ct(ina226_t *dev, ina226_ct_t ct);

/* ══════════════════════════════════════════════════════════════════════════════
 *  Alert / 屏蔽-使能寄存器 (0x06) — Alert / Mask-Enable register
 * ══════════════════════════════════════════════════════════════════════════ */

/*
 * 配置 ALERT 引脚的触发条件和行为。
 * Configure ALERT pin trigger conditions and behaviour.
 *
 * mask — 通过按位或组合 INA226_ALERT_* 宏。
 *        Bitwise-OR of INA226_ALERT_* macros.
 *
 * 例 Example (触发过流告警, 锁存输出, 低电平有效):
 *   ina226_set_alert(&dev, INA226_ALERT_SOL | INA226_ALERT_LATCH);
 *   ina226_set_alert_limit_amps(&dev, 2.5f);
 */
void     ina226_set_alert(ina226_t *dev, uint16_t mask);

/*
 * 读回屏蔽/使能寄存器当前值 (含只读状态位 AFF / MATH_OVF)。
 * Read back mask/enable register (includes read-only status bits AFF / MATH_OVF).
 */
uint16_t ina226_read_alert(ina226_t *dev);

/*
 * 检测 ALERT 引脚是否已断言 (读 AFF 位)。
 * Check whether the ALERT pin is currently asserted (reads AFF bit).
 * 注意: 非锁存模式下, 条件消失后本函数也返回 false。
 * Note: in non-latched mode this returns false once the condition clears.
 */
bool     ina226_alert_asserted(ina226_t *dev);

/*
 * 检测触发模式下的单次转换是否完成 (读 CNVR 位)。
 * Check whether a triggered one-shot conversion has finished (reads CNVR bit).
 * 连续模式下 CNVR 位行为未定义, 此函数只在触发模式有意义。
 * CNVR bit is undefined in continuous mode; only meaningful in triggered mode.
 */
bool     ina226_conversion_ready(ina226_t *dev);

/* ══════════════════════════════════════════════════════════════════════════════
 *  告警阈值寄存器 (0x07) — Alert Limit register
 * ══════════════════════════════════════════════════════════════════════════ */

/*
 * 写入告警阈值 (原始寄存器值, 单位 = current_lsb)。
 * Write alert limit as raw register value (unit = current_lsb).
 * 阈值含义取决于 Alert 配置: SOL/SUL 为分流电压, POL 为功率, BOL/BUL 为总线电压。
 * Meaning depends on alert config: SOL/SUL = shunt, POL = power, BOL/BUL = bus.
 */
void     ina226_set_alert_limit(ina226_t *dev, int16_t limit);

/*
 * 读回告警阈值 (原始寄存器值)。
 * Read back alert limit (raw register value).
 */
int16_t  ina226_read_alert_limit(ina226_t *dev);

/*
 * 用物理电流值 (安培) 设置告警阈值 — 自动根据 current_lsb 换算。
 * Set alert limit as physical current in amps — auto-converts using current_lsb.
 *
 * 适用于 SOL / SUL 告警。对于 POL 告警阈值需使用 ina226_set_alert_limit()。
 * Suitable for SOL / SUL alerts.  For POL threshold use ina226_set_alert_limit().
 */
void     ina226_set_alert_limit_amps(ina226_t *dev, float amps);

/* ══════════════════════════════════════════════════════════════════════════════
 *  原始 ADC 读数 / Raw ADC readings
 *
 *  返回寄存器原始值 (带符号), 主要用于调试和校准。
 *  Return raw register values (signed), mainly for debugging and calibration.
 *  返回值: 0 = 成功, -1 = I2C 错误
 *  Return: 0 = success, -1 = I2C error
 * ══════════════════════════════════════════════════════════════════════════ */

int ina226_read_shunt_raw(ina226_t *dev, int16_t *raw);    // 分流电压 Shunt voltage
int ina226_read_bus_raw(ina226_t *dev, int16_t *raw);      // 总线电压 Bus voltage
int ina226_read_current_raw(ina226_t *dev, int16_t *raw);  // 电流 Current
int ina226_read_power_raw(ina226_t *dev, int16_t *raw);    // 功率 Power

/* ══════════════════════════════════════════════════════════════════════════════
 *  物理量读取 / Converted readings
 *
 *  返回转换后的浮点值, 单位由 unit 参数指定。
 *  Return converted float value; unit selected by 'unit' parameter.
 *  I2C 通信失败时返回 NAN (Not A Number)。
 *  Returns NAN on I2C communication failure.
 * ══════════════════════════════════════════════════════════════════════════ */

/*
 * 总线电压 / Bus voltage (loading-side voltage).
 * 原始 LSB = 1.25 mV, 范围 0–36 V (实际受限于 PCB 布线)。
 * Raw LSB = 1.25 mV, range 0–36 V (PCB traces may limit).
 */
float ina226_read_bus_voltage(ina226_t *dev, ina226_unit_t unit);

/*
 * 分流电压 / Shunt voltage (voltage drop across Rshunt).
 * 原始 LSB = 2.5 µV, 符号表示电流方向。
 * Raw LSB = 2.5 µV; sign indicates current direction.
 * 提示: 分流电压本身是微小值, 推荐使用 INA226_UNIT_MV。
 * Hint: shunt voltage is tiny; INA226_UNIT_MV is recommended.
 */
float ina226_read_shunt_voltage(ina226_t *dev, ina226_unit_t unit);

/*
 * 电流 / Current (Vshunt / Rshunt, 符号表示方向)。
 * Current = Vshunt / Rshunt; sign indicates direction.
 */
float ina226_read_current(ina226_t *dev, ina226_unit_t unit);

/*
 * 功率 / Power (Vbus × Current)。
 * Power = Vbus × Current.
 */
float ina226_read_power(ina226_t *dev, ina226_unit_t unit);

/* ══════════════════════════════════════════════════════════════════════════════
 *  芯片识别 / Chip identification
 *
 *  用于在运行时确认 I2C 总线上连接的是真正的 INA226。
 *  Useful to verify at runtime that a genuine INA226 is on the bus.
 *
 *  制造商 ID / Manufacturer ID: 0x5449 → ASCII "TI"
 *  芯片 ID   / Die ID:          0x2260
 * ══════════════════════════════════════════════════════════════════════════ */

uint16_t ina226_read_mfr_id(ina226_t *dev);
uint16_t ina226_read_die_id(ina226_t *dev);

#ifdef __cplusplus
}
#endif

#endif  /* INA226_H */
