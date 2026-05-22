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

#ifndef I2C_HAL_H
#define I2C_HAL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int port;            // I2C port number (0 or 1)
    int sda;
    int scl;
    int initialized;
} i2c_hal_t;

void i2c_hal_init(i2c_hal_t *i2c, int port, int sda, int scl, uint32_t freq_hz);
int  i2c_hal_write_reg16(i2c_hal_t *i2c, uint8_t dev_addr, uint8_t reg, uint16_t val);
int  i2c_hal_read_reg16(i2c_hal_t *i2c, uint8_t dev_addr, uint8_t reg, uint16_t *val);

#ifdef __cplusplus
}
#endif

#endif
