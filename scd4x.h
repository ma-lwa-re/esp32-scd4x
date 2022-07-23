/* MIT License
*
* Copyright (c) 2022 ma-lwa-re
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/
#include <stdio.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c.h"

#define FAHRENHEIT(celcius)         (((celcius * 9.0) / 5.0) + 32.0)
#define KELVIN(celcius)             (celcius + 273.15)
#define SCALE_CELCIUS               ('C')
#define SCALE_FAHRENHEIT            ('F')
#define SCALE_KELVIN                ('K')

#define I2C_MASTER_SDA              (GPIO_NUM_6)
#define I2C_MASTER_SCL              (GPIO_NUM_7)
#define I2C_MASTER_RX_BUF_DISABLE   (0)
#define I2C_MASTER_TX_BUF_DISABLE   (0)
#define I2C_MASTER_FREQ_HZ          (100000)
#define I2C_MASTER_TIMEOUT_MS       (1000)
#define I2C_MASTER_NUM              (0)
#define I2C_ACK_CHECK_DIS           (0x00)
#define I2C_ACK_CHECK_EN            (0x01)
#define I2C_ACK_VAL                 (0x00)
#define I2C_NACK_VAL                (0x01)

#define SCD41_SENSOR_ADDR           (0x62)
#define SCD41_READ_ERROR            (0xFFFF)
#define SCD41_HEX_CODE_SIZE         (0x02)
#define SLEEP_DELAY                 (1000 * 60 * 15)
#define UPDATE_DELAY                (1000 * 15)
#define INIT_DELAY                  (5000)

#define CRC8_POLYNOMIAL             (0x31)
#define CRC8_INIT                   (0xFF)

#ifndef SCD4X_H
#define SCD4X_H
typedef struct sensors_values {
    uint16_t co2;
    float temperature;
    float humidity;
} sensors_values_t;
#endif

uint8_t scd41_generate_crc(const uint8_t* data, uint16_t count);

esp_err_t scd41_send_command(uint8_t *command);

esp_err_t scd41_read(uint8_t *hex_code, uint8_t *measurements, uint8_t size);

esp_err_t scd41_write(uint8_t *hex_code, uint8_t *measurements, uint8_t size);

esp_err_t start_periodic_measurement();

esp_err_t read_measurement(sensors_values_t *sensors_values);

esp_err_t stop_periodic_measurement();

esp_err_t set_temperature_offset(float temperature);

float get_temperature_offset();

esp_err_t set_sensor_altitude(float altitude);

uint16_t get_sensor_altitude();

esp_err_t set_ambient_pressure(uint32_t pressure);

uint16_t perform_forced_recalibration(u_int16_t  co2_concentration);

esp_err_t set_automatic_self_calibration_enabled(bool asc_enabled);

bool get_automatic_self_calibration_enabled();

esp_err_t start_low_power_periodic_measurement();

bool get_data_ready_status();

esp_err_t persist_settings();

uint64_t get_serial_number();

bool perform_self_test();

esp_err_t perfom_factory_reset();

esp_err_t reinit();

esp_err_t measure_single_shot();

esp_err_t measure_single_shot_rht_only();

esp_err_t power_down();

esp_err_t wake_up();
