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

#include "i2c_hal.h"
#include "driver/i2c.h"
#include "esp_log.h"

static const char *TAG = "i2c_hal";

void i2c_hal_init(i2c_hal_t *i2c, int port, int sda, int scl, uint32_t freq_hz) {
    if (i2c->initialized) return;

    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = scl,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = freq_hz,
        .clk_flags = 0,
    };
    i2c_param_config((i2c_port_t)port, &cfg);
    i2c_driver_install((i2c_port_t)port, I2C_MODE_MASTER, 0, 0, 0);

    i2c->port = port;
    i2c->sda = sda;
    i2c->scl = scl;
    i2c->initialized = 1;
    ESP_LOGI(TAG, "I2C%d init ok (SDA=%d, SCL=%d, %lu Hz)", port, sda, scl, freq_hz);
}

int i2c_hal_write_reg16(i2c_hal_t *i2c, uint8_t dev_addr, uint8_t reg, uint16_t val) {
    uint8_t buf[3] = { reg, (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) };
    esp_err_t ret = i2c_master_write_to_device((i2c_port_t)i2c->port, dev_addr, buf, 3,
                                                pdMS_TO_TICKS(100));
    return (ret == ESP_OK) ? 0 : -1;
}

int i2c_hal_read_reg16(i2c_hal_t *i2c, uint8_t dev_addr, uint8_t reg, uint16_t *val) {
    esp_err_t ret = i2c_master_write_read_device((i2c_port_t)i2c->port, dev_addr,
                                                  &reg, 1, (uint8_t *)val, 2,
                                                  pdMS_TO_TICKS(100));
    if (ret == ESP_OK) {
        *val = (*val >> 8) | (*val << 8);  // big-endian to host
        return 0;
    }
    return -1;
}
