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
 * ║                    INA226 功率监测传感器驱动 · 实现                          ║
 * ║                 INA226 Power Monitor Sensor Driver · Implementation          ║
 * ╚══════════════════════════════════════════════════════════════════════════════╝
 *
 * 【数据手册参考 / Datasheet Reference】
 *   TI INA226 — High-Side or Low-Side Measurement,
 *   Bi-Directional Current and Power Monitor with I2C Interface
 *   (文献编号 / Literature Number: SBOS547F)
 *
 * 【寄存器映射 / Register Map】
 *   ┌──────┬────────────────────────────────────────────────┐
 *   │ 地址 │ 寄存器名                                       │
 *   │ Addr │ Register                                       │
 *   ├──────┼────────────────────────────────────────────────┤
 *   │ 0x00 │ 配置       Configuration (R/W)                 │
 *   │ 0x01 │ 分流电压   Shunt Voltage (R, signed int16)     │
 *   │ 0x02 │ 总线电压   Bus Voltage (R, unsigned)           │
 *   │ 0x03 │ 功率       Power (R, unsigned)                 │
 *   │ 0x04 │ 电流       Current (R, signed int16)           │
 *   │ 0x05 │ 校准       Calibration (R/W)                   │
 *   │ 0x06 │ 屏蔽/使能  Mask/Enable (R/W)                   │
 *   │ 0x07 │ 告警阈值   Alert Limit (R/W, signed int16)     │
 *   │ 0xFE │ 制造商 ID  Manufacturer ID (R) = 0x5449        │
 *   │ 0xFF │ 芯片 ID    Die ID (R) = 0x2260                 │
 *   └──────┴────────────────────────────────────────────────┘
 *
 * 【校准公式 / Calibration Formula】
 *   Current_LSB = Max_Expected_Current / 2^15
 *   Calibration = trunc(0.00512 / (Current_LSB × Rshunt))
 *
 *   其中 0.00512 = 允许的最大分流电压 81.92 mV 除以 2^4 的内部因子。
 *   Where 0.00512 = max allowed shunt voltage 81.92 mV divided by internal 2^4 factor.
 *
 * 【物理量 LSB / Measurement LSBs】
 *   Bus Voltage LSB    = 1.25 mV  (固定 / fixed)
 *   Shunt Voltage LSB  = 2.5 µV   (固定 / fixed)
 *   Current LSB        = Max_Expected_Current / 32768  (由校准决定)
 *   Power LSB          = 25 × Current_LSB               (由校准决定)
 */

#include "ina226.h"
#include "hal/i2c_hal.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "ina226";

/* ── 寄存器地址 / Register addresses ────────────────────────────── */

#define INA226_REG_CONFIG      0x00
#define INA226_REG_SHUNT_VOLT  0x01
#define INA226_REG_BUS_VOLT    0x02
#define INA226_REG_POWER       0x03
#define INA226_REG_CURRENT     0x04
#define INA226_REG_CALIBRATION 0x05
#define INA226_REG_MASK_ENABLE 0x06
#define INA226_REG_ALERT_LIMIT 0x07
#define INA226_REG_MFR_ID      0xFE
#define INA226_REG_DIE_ID      0xFF

/* ── 配置寄存器位域 / Config register bit fields ──────────────────
 *
 *   Bit 15    : RST  — 复位 (写 1 触发, 自清零) / Reset (self-clearing)
 *   Bits 14–12: AVG  — 采样平均次数 / Averaging mode
 *   Bits 11–9 : VBUSCT — 总线转换时间 / Bus conversion time
 *   Bits 8–6  : VSHCT  — 分流转换时间 / Shunt conversion time
 *   Bits 5–3  : MODE  — 工作模式 / Operating mode
 *   Bits 2–0  : (同 MODE 低位, 兼容) / (same as MODE LSBs, for compatibility)
 * ───────────────────────────────────────────────────────────────── */

#define CFG_RESET              (1u << 15)
#define CFG_AVG_SHIFT          9
#define CFG_BUS_CT_SHIFT       6
#define CFG_SHUNT_CT_SHIFT     3
#define CFG_MODE_SHIFT         0

/* ── 内部 I2C 总线管理 / Internal I2C bus management ──────────────
 *
 *   最多两个 I2C 外设 (esp32-c6 有 2 个 I2C 控制器)。
 *   Up to two I2C peripherals (ESP32-C6 has 2 I2C controllers).
 *   首次使用时惰性初始化, 同端口后续请求复用。
 *   Lazy-init on first use; subsequent requests on same port reuse the handle.
 * ───────────────────────────────────────────────────────────────── */

static i2c_hal_t i2c_bus[2];
static bool      i2c_done[2];

static i2c_hal_t *get_i2c(int port) {
    if (port < 0 || port > 1) return NULL;
    return &i2c_bus[port];
}

/*
 * 向传感器寄存器写入 16-bit 大端值。
 * Write a 16-bit big-endian value to a sensor register.
 * 返回: 0 = 成功, -1 = I2C 错误
 */
static int write_reg(ina226_t *dev, uint8_t reg, uint16_t val) {
    i2c_hal_t *i2c = get_i2c(dev->i2c_port);
    return i2c ? i2c_hal_write_reg16(i2c, dev->addr, reg, val) : -1;
}

/*
 * 从传感器寄存器读取 16-bit 大端值并转为主机字节序。
 * Read a 16-bit big-endian value from a sensor register, convert to host order.
 * 返回: 0 = 成功, -1 = I2C 错误
 */
static int read_reg(ina226_t *dev, uint8_t reg, uint16_t *val) {
    i2c_hal_t *i2c = get_i2c(dev->i2c_port);
    return i2c ? i2c_hal_read_reg16(i2c, dev->addr, reg, val) : -1;
}

/*
 * 将 SI 基本单位值按枚举缩放为对应单位。
 * Scale an SI-base value to the unit selected by the enum.
 *
 *   INA226_UNIT_MV:  V → mV   (× 10³)
 *   INA226_UNIT_V:   V → V    (× 1)
 *   INA226_UNIT_UA:  A → µA   (× 10⁶)
 *   INA226_UNIT_MA:  A → mA   (× 10³)
 *   INA226_UNIT_A:   A → A    (× 1)
 *   INA226_UNIT_MW:  W → mW   (× 10³)
 *   INA226_UNIT_W:   W → W    (× 1)
 */
static float scale_to(float si, ina226_unit_t unit) {
    switch (unit) {
        case INA226_UNIT_MV: return si * 1e3f;
        case INA226_UNIT_V:  return si;
        case INA226_UNIT_UA: return si * 1e6f;
        case INA226_UNIT_MA: return si * 1e3f;
        case INA226_UNIT_A:  return si;
        case INA226_UNIT_MW: return si * 1e3f;
        case INA226_UNIT_W:  return si;
        default:             return si;
    }
}

/* ══════════════════════════════════════════════════════════════════════════════
 *  初始化 / Initialization
 * ══════════════════════════════════════════════════════════════════════════ */

int ina226_init(ina226_t *dev, int i2c_port, int sda, int scl,
                uint8_t addr, float shunt_res, float max_current) {
    if (!dev) return -1;
    if (shunt_res <= 0.0f || max_current <= 0.0f) return -2;

    /*
     * 惰性初始化 I2C 总线 (每个 port 仅一次)。
     * Lazy-init I2C bus — once per port.
     */
    if (!i2c_done[i2c_port]) {
        i2c_hal_init(&i2c_bus[i2c_port], i2c_port, sda, scl, INA226_I2C_FREQ);
        i2c_done[i2c_port] = true;
    }

    dev->i2c_port    = i2c_port;
    dev->addr        = addr;
    dev->shunt_res   = shunt_res;
    dev->max_current = max_current;

    /*
     * 计算校准寄存器值。
     *   Current_LSB = Expected_Max_Current / 2^15
     *   Cal         = 0.00512 / (Current_LSB × Rshunt)
     *
     * 0.00512 的来源: 最大可测量分流电压为 ±81.92 mV。
     * 内部右移 4 位 (÷16) 后存入 16-bit 寄存器, 故因子为 81.92mV / 16 = 5.12mV。
     * 其中 0.00512 的单位是 V, 与 Rshunt (Ω) 和 Current_LSB (A) 匹配。
     *
     * Origin of 0.00512: max measurable shunt voltage is ±81.92 mV.
     * The value is internally right-shifted by 4 (÷16) before entering the
     * 16-bit register, giving a factor of 81.92mV / 16 = 5.12mV.
     * 0.00512 has unit V, matching Rshunt (Ω) and Current_LSB (A).
     */
    float lsb = max_current / 32768.0f;
    dev->current_lsb = lsb;
    uint16_t cal = (uint16_t)(0.00512f / (lsb * shunt_res));

    ESP_LOGI(TAG, "addr=0x%02X Rshunt=%.4fΩ Imax=%.2fA LSB=%.6fA Cal=%u",
             addr, shunt_res, max_current, lsb, cal);

    /* 写入校准值 / Write calibration */
    write_reg(dev, INA226_REG_CALIBRATION, cal);

    /*
     * 验证芯片存在: 尝试读出配置寄存器 (上电默认值 0x4127)。
     * 然后切换到连续分流+总线测量模式。
     *
     * Verify chip presence by reading the config register (POR default 0x4127),
     * then switch to continuous shunt+bus mode.
     */
    uint16_t cfg;
    if (read_reg(dev, INA226_REG_CONFIG, &cfg) != 0) {
        ESP_LOGE(TAG, "I2C read failed at addr 0x%02X — check SDA/SCL wiring", addr);
        dev->initialized = false;
        return -3;
    }
    cfg = (cfg & 0x07FF) | INA226_MODE_SHUNT_BUS_CONT;
    write_reg(dev, INA226_REG_CONFIG, cfg);

    dev->initialized = true;
    ESP_LOGI(TAG, "ready");
    return 0;
}

/* ══════════════════════════════════════════════════════════════════════════════
 *  复位 / Reset
 * ══════════════════════════════════════════════════════════════════════════ */

void ina226_reset(ina226_t *dev) {
    /*
     * 写 bit 15 触发上电复位。该位自动清零, 无需回写。
     * Writing bit 15 triggers a power-on reset.  Self-clearing, no write-back needed.
     * 警告: 复位后校准寄存器归零, 必须重新调用 ina226_init()。
     * Warning: calibration is cleared; you MUST re-call ina226_init().
     */
    write_reg(dev, INA226_REG_CONFIG, CFG_RESET);
    ESP_LOGI(TAG, "reset");
}

/* ══════════════════════════════════════════════════════════════════════════════
 *  配置寄存器 / Configuration register
 * ══════════════════════════════════════════════════════════════════════════ */

uint16_t ina226_read_config(ina226_t *dev) {
    uint16_t val;
    if (read_reg(dev, INA226_REG_CONFIG, &val) != 0) return 0;
    return val;
}

void ina226_write_config(ina226_t *dev, uint16_t cfg) {
    write_reg(dev, INA226_REG_CONFIG, cfg);
}

/*
 * 模式设置在配置寄存器低 3 位。
 * Mode field occupies bits [2:0] of the config register.
 */
void ina226_set_mode(ina226_t *dev, ina226_mode_t mode) {
    uint16_t cfg;
    if (read_reg(dev, INA226_REG_CONFIG, &cfg) != 0) return;
    cfg = (cfg & ~0x0007u) | (mode & 0x07);
    write_reg(dev, INA226_REG_CONFIG, cfg);
}

/*
 * 平均次数占据 bit [14:12]。
 * Averaging field occupies bits [14:12].
 */
void ina226_set_averaging(ina226_t *dev, ina226_avg_t avg) {
    uint16_t cfg;
    if (read_reg(dev, INA226_REG_CONFIG, &cfg) != 0) return;
    cfg = (cfg & ~(0x7u << CFG_AVG_SHIFT)) | ((avg & 0x07) << CFG_AVG_SHIFT);
    write_reg(dev, INA226_REG_CONFIG, cfg);
}

/*
 * 总线转换时间占据 bit [11:9]。
 * Bus conversion-time field occupies bits [11:9].
 */
void ina226_set_bus_ct(ina226_t *dev, ina226_ct_t ct) {
    uint16_t cfg;
    if (read_reg(dev, INA226_REG_CONFIG, &cfg) != 0) return;
    cfg = (cfg & ~(0x7u << CFG_BUS_CT_SHIFT)) | ((ct & 0x07) << CFG_BUS_CT_SHIFT);
    write_reg(dev, INA226_REG_CONFIG, cfg);
}

/*
 * 分流转换时间占据 bit [8:6]。
 * Shunt conversion-time field occupies bits [8:6].
 */
void ina226_set_shunt_ct(ina226_t *dev, ina226_ct_t ct) {
    uint16_t cfg;
    if (read_reg(dev, INA226_REG_CONFIG, &cfg) != 0) return;
    cfg = (cfg & ~(0x7u << CFG_SHUNT_CT_SHIFT)) | ((ct & 0x07) << CFG_SHUNT_CT_SHIFT);
    write_reg(dev, INA226_REG_CONFIG, cfg);
}

/* ══════════════════════════════════════════════════════════════════════════════
 *  Alert / 屏蔽-使能寄存器 — Mask/Enable register (0x06)
 *
 *  寄存器位布局:
 *  ┌──15──┬──14──┬──13──┬──12──┬──11──┬──10──┬─9..5─┬───4──┬───3──┬──2──┬──1──┬──0──┐
 *  │ SOL  │ SUL  │ BOL  │ BUL  │ POL  │ CNVR │ (0)  │ AFF  │ MOVF │ (0) │LATCH│POL_I│
 *  └──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┘
 *   bit15–10: 可读写 / R/W   — 告警使能 / Alert enable
 *   bit4:     只读   / RO    — 告警功能标志 (ALERT 脚当前是否断言)
 *   bit3:     只读   / RO    — 算术溢出标志 (计算值超 16-bit 范围)
 *   bit1:     可读写 / R/W   — 锁存使能
 *   bit0:     可读写 / R/W   — 极性反转
 * ══════════════════════════════════════════════════════════════════════════ */

void ina226_set_alert(ina226_t *dev, uint16_t mask) {
    write_reg(dev, INA226_REG_MASK_ENABLE, mask);
}

uint16_t ina226_read_alert(ina226_t *dev) {
    uint16_t val;
    if (read_reg(dev, INA226_REG_MASK_ENABLE, &val) != 0) return 0;
    return val;
}

bool ina226_alert_asserted(ina226_t *dev) {
    uint16_t val;
    if (read_reg(dev, INA226_REG_MASK_ENABLE, &val) != 0) return false;
    /*
     * AFF (Alert Function Flag) — 硬件置位, 读 Mask 寄存器后自动清零 (锁存模式)。
     * AFF is set by hardware; in latched mode it auto-clears after reading the Mask register.
     */
    return (val & INA226_ALERT_AFF) != 0;
}

bool ina226_conversion_ready(ina226_t *dev) {
    uint16_t val;
    if (read_reg(dev, INA226_REG_MASK_ENABLE, &val) != 0) return false;
    /*
     * CNVR (Conversion Ready) — 触发模式下本次转换完成时置位。
     * 连续模式下 CNVR 行为未定义, 但通常也反映最新转换状态。
     * CNVR is set when the triggered conversion completes.
     * In continuous mode the behaviour is undefined, but it typically
     * tracks whether fresh data is available.
     */
    return (val & INA226_ALERT_CNVR) != 0;
}

/* ══════════════════════════════════════════════════════════════════════════════
 *  告警阈值 / Alert Limit register (0x07)
 *
 *  16-bit 有符号值, 单位视告警类型而定:
 *  - SOL / SUL: 分流电压 (LSB = 2.5 µV)
 *  - BOL / BUL: 总线电压 (LSB = 1.25 mV)
 *  - POL:       功率       (LSB = 25 × Current_LSB)
 *
 * 16-bit signed value; LSB depends on alert type:
 *  - SOL / SUL: shunt voltage (LSB = 2.5 µV)
 *  - BOL / BUL: bus voltage   (LSB = 1.25 mV)
 *  - POL:        power        (LSB = 25 × Current_LSB)
 * ══════════════════════════════════════════════════════════════════════════ */

void ina226_set_alert_limit(ina226_t *dev, int16_t limit) {
    write_reg(dev, INA226_REG_ALERT_LIMIT, (uint16_t)limit);
}

int16_t ina226_read_alert_limit(ina226_t *dev) {
    uint16_t val;
    if (read_reg(dev, INA226_REG_ALERT_LIMIT, &val) != 0) return 0;
    return (int16_t)val;
}

void ina226_set_alert_limit_amps(ina226_t *dev, float amps) {
    /*
     * 物理电流 → 寄存器原始值: raw = I(A) / Current_LSB
     * Physical current → raw register value.
     * 注意: 告警阈值比较的是分流电压寄存器 (0x01), 因此
     * INA226_ALERT_SOL/SUL 与本函数直接对应。
     * Note: SOL/SUL compare against the current register (0x04),
     * which is already calibrated — so amps→raw conversion is correct.
     */
    int16_t raw = (int16_t)(amps / dev->current_lsb);
    write_reg(dev, INA226_REG_ALERT_LIMIT, (uint16_t)raw);
}

/* ══════════════════════════════════════════════════════════════════════════════
 *  原始 ADC 读数 / Raw ADC readings
 *
 *  直接将寄存器值以带符号形式返回, 不做任何换算。
 *  Return register values as-is (signed), no conversion applied.
 * ══════════════════════════════════════════════════════════════════════════ */

int ina226_read_shunt_raw(ina226_t *dev, int16_t *raw) {
    uint16_t val;
    if (read_reg(dev, INA226_REG_SHUNT_VOLT, &val) != 0) return -1;
    *raw = (int16_t)val;
    return 0;
}

int ina226_read_bus_raw(ina226_t *dev, int16_t *raw) {
    uint16_t val;
    if (read_reg(dev, INA226_REG_BUS_VOLT, &val) != 0) return -1;
    /* 总线电压寄存器 bit 0–2 不是数据位, 无需去除 — 直接返回原始 16-bit */
    /* Bus voltage register bits 0–2 are not data; return raw 16-bit as-is. */
    *raw = (int16_t)val;
    return 0;
}

int ina226_read_current_raw(ina226_t *dev, int16_t *raw) {
    uint16_t val;
    if (read_reg(dev, INA226_REG_CURRENT, &val) != 0) return -1;
    *raw = (int16_t)val;
    return 0;
}

int ina226_read_power_raw(ina226_t *dev, int16_t *raw) {
    uint16_t val;
    if (read_reg(dev, INA226_REG_POWER, &val) != 0) return -1;
    *raw = (int16_t)val;  /* 功率为无符号量, 但为统一接口保留 signed 类型 */
    return 0;
}

/* ══════════════════════════════════════════════════════════════════════════════
 *  物理量换算 / Converted readings
 *
 *  每个函数: 读寄存器 → 乘 LSB 得到 SI 单位值 → scale_to() 缩放到目标单位。
 *  Each function: read register → multiply by LSB → scale_to() target unit.
 *  I2C 错误时返回 NAN (math.h)。
 *  Returns NAN on I2C error.
 * ══════════════════════════════════════════════════════════════════════════ */

float ina226_read_bus_voltage(ina226_t *dev, ina226_unit_t unit) {
    uint16_t raw;
    if (read_reg(dev, INA226_REG_BUS_VOLT, &raw) != 0) return NAN;

    /*
     * 总线电压寄存器格式: bit 15–3 为数据, bit 2–0 固定为零 (未使用)。
     * 整体右移 3 位等价于 × 1.25mV LSB; 也可以不位移直接 × 1.25mV:
     * raw × 1.25mV = (raw >> 3) × 10mV? 不对...
     * 实际上 raw × 1.25mV 直接就对了, 因为 bit 2–0 为 0 时不影响。
     * Bus voltage LSB = 1.25 mV.  Bits [2:0] are zero, so multiplying the
     * raw 16-bit word by 1.25 mV already yields the correct voltage.
     */
    return scale_to(raw * 0.00125f, unit);
}

float ina226_read_shunt_voltage(ina226_t *dev, ina226_unit_t unit) {
    uint16_t raw;
    if (read_reg(dev, INA226_REG_SHUNT_VOLT, &raw) != 0) return NAN;

    /*
     * 分流电压为有符号 16-bit, LSB = 2.5 µV。
     * Shunt voltage is signed 16-bit, LSB = 2.5 µV.
     * 分流电压通常在几十 mV 量级, 默认返回 mV。
     * Shunt voltage is typically on the order of tens of mV; default to mV.
     */
    float volts = (int16_t)raw * 2.5e-6f;
    return (unit == INA226_UNIT_V) ? volts : volts * 1e3f;
}

float ina226_read_current(ina226_t *dev, ina226_unit_t unit) {
    uint16_t raw;
    if (read_reg(dev, INA226_REG_CURRENT, &raw) != 0) return NAN;

    /*
     * 电流为有符号 16-bit, LSB = Current_LSB (由校准决定)。
     * Current is signed 16-bit, LSB = Current_LSB (determined by calibration).
     */
    return scale_to((int16_t)raw * dev->current_lsb, unit);
}

float ina226_read_power(ina226_t *dev, ina226_unit_t unit) {
    uint16_t raw;
    if (read_reg(dev, INA226_REG_POWER, &raw) != 0) return NAN;

    /*
     * 功率为无符号 16-bit (INA226 只报告正功率, 方向由电流符号体现)。
     * LSB = 25 × Current_LSB (固定内部因子)。
     * Power is unsigned 16-bit (INA226 reports only positive power;
     * direction is reflected in current sign).
     * LSB = 25 × Current_LSB (fixed internal multiplier).
     *
     * 25 的来源: 功率寄存器通过内部累加器直接从分流和总线电压算出,
     * 硬件设计使得其 LSB 恰好为 25 × Current_LSB。
     * The factor 25 comes from the hardware accumulator design;
     * the LSB is defined as 25 × Current_LSB per the datasheet.
     */
    return scale_to(raw * 25.0f * dev->current_lsb, unit);
}

/* ══════════════════════════════════════════════════════════════════════════════
 *  芯片识别 / Chip identification — 只读寄存器, 无参数依赖
 *  Read-only registers; no calibration or config needed.
 *  用于在运行时验证芯片型号, 建议在 init 成功后调用。
 *  Use at runtime to verify chip model; recommend calling right after init.
 * ══════════════════════════════════════════════════════════════════════════ */

uint16_t ina226_read_mfr_id(ina226_t *dev) {
    uint16_t val;
    if (read_reg(dev, INA226_REG_MFR_ID, &val) != 0) return 0;
    return val;  /* 预期 / expected: 0x5449 */
}

uint16_t ina226_read_die_id(ina226_t *dev) {
    uint16_t val;
    if (read_reg(dev, INA226_REG_DIE_ID, &val) != 0) return 0;
    return val;  /* 预期 / expected: 0x2260 */
}
