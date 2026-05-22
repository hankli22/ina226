/*
 * INA226 基本使用示例 / Basic usage example
 *
 * 硬件连接 / Wiring:
 *   ESP32-C6  SDA (GPIO6) → INA226 SDA
 *   ESP32-C6  SCL (GPIO7) → INA226 SCL
 *   INA226 VCC → 3.3V,  GND → GND
 *   INA226 A0, A1 → GND  (I2C addr = 0x40)
 *
 *   ┌─Rshunt─┐
 *   │ 10 mΩ  │  (VIN+ → 负载正极 / Load+,  VIN- → 电源正极 / Supply+)
 *   └────────┘
 */

#include "ina226.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "example";

// 引脚定义 / Pin definitions
#define I2C_PORT    0
#define PIN_SDA     6
#define PIN_SCL     7
#define INA226_ADDR 0x40

extern "C" void app_main() {
    ina226_t sensor;

    /*
     * 初始化: I2C端口0, SDA=6, SCL=7, 地址=0x40
     * 分流电阻 10 mΩ, 预期最大电流 5 A
     *
     * Init: I2C port 0, SDA=6, SCL=7, addr=0x40
     * Shunt resistor 10 mΩ, expected max current 5 A
     */
    int ret = ina226_init(&sensor, I2C_PORT, PIN_SDA, PIN_SCL,
                          INA226_ADDR, 0.01f, 5.0f);
    if (ret != 0) {
        ESP_LOGE(TAG, "INA226 init failed (%d)", ret);
        return;
    }

    // 验证芯片型号 / Verify chip identity
    uint16_t mfr = ina226_read_mfr_id(&sensor);
    uint16_t die = ina226_read_die_id(&sensor);
    ESP_LOGI(TAG, "MFR=0x%04X (expect 0x5449), DIE=0x%04X (expect 0x2260)", mfr, die);

    // 配置: 64次平均, 1.1ms转换时间 / Config: 64-sample avg, 1.1ms conv time
    ina226_set_averaging(&sensor, INA226_AVG_64);
    ina226_set_bus_ct(&sensor, INA226_CT_1100US);
    ina226_set_shunt_ct(&sensor, INA226_CT_1100US);

    /*
     * 配置告警: 电流 > 2A 时 ALERT 脚拉低 (锁存模式)
     * Configure alert: latch ALERT low when current exceeds 2 A
     */
    ina226_set_alert(&sensor, INA226_ALERT_SOL | INA226_ALERT_LATCH);
    ina226_set_alert_limit_amps(&sensor, 2.0f);

    for (;;) {
        /*
         * 读总线电压 (V), 电流 (mA), 功率 (mW)
         * Read bus voltage (V), current (mA), power (mW)
         */
        float bus_v = ina226_read_bus_voltage(&sensor, INA226_UNIT_V);
        float cur_ma = ina226_read_current(&sensor, INA226_UNIT_MA);
        float pow_mw = ina226_read_power(&sensor, INA226_UNIT_MW);
        float shunt_mv = ina226_read_shunt_voltage(&sensor, INA226_UNIT_MV);

        ESP_LOGI(TAG, "Bus=%.3fV  Shunt=%.3fmV  Current=%.1fmA  Power=%.1fmW",
                 bus_v, shunt_mv, cur_ma, pow_mw);

        // 检查告警 / Check alert
        if (ina226_alert_asserted(&sensor)) {
            ESP_LOGW(TAG, "ALERT! Current limit exceeded");
            ina226_read_alert(&sensor);  // 读寄存器以清除锁存
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
