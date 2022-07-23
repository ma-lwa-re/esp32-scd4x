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
#include "scd4x.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define UART_STACK_SIZE             (4096)
#define TEMPERATURE_OFFSET          (4.0)
#define SENSOR_ALTITUDE             (0)

static const char *MAIN_TAG = "main";
static const char *SENSORS_TAG = "sensors";

char scale = SCALE_CELCIUS;
float temperature = 0.0;
float humidity = 0.0;
float co2_level = 0.0;

void sensors_task(void *arg) {
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &i2c_config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, i2c_config.mode,
                    I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0));

    esp_log_level_set(SENSORS_TAG, ESP_LOG_INFO);

    #if defined(SENSORS_SCALE_F)
    scale = SCALE_FAHRENHEIT;
    #elif defined(SENSORS_SCALE_K)
    scale = SCALE_KELVIN;
    #endif

    vTaskDelay(INIT_DELAY / portTICK_PERIOD_MS);
    ESP_LOGI(SENSORS_TAG, "Sensor serial number 0x%012llX", get_serial_number());

    vTaskDelay(INIT_DELAY / portTICK_PERIOD_MS);
    float temperature_offset = get_temperature_offset();

    vTaskDelay(INIT_DELAY / portTICK_PERIOD_MS);
    uint16_t sensor_altitude = get_sensor_altitude();

    if(temperature_offset != SCD41_READ_ERROR && sensor_altitude != SCD41_READ_ERROR) {

        if(temperature_offset != TEMPERATURE_OFFSET) {
            ESP_LOGW(SENSORS_TAG, "Temperature offset calibration from %.1f °%c to %.1f °%c",
                     temperature_offset, scale, TEMPERATURE_OFFSET, scale);

            vTaskDelay(INIT_DELAY / portTICK_PERIOD_MS);
            ESP_ERROR_CHECK_WITHOUT_ABORT(set_temperature_offset(TEMPERATURE_OFFSET));

            vTaskDelay(INIT_DELAY / portTICK_PERIOD_MS);
            ESP_ERROR_CHECK_WITHOUT_ABORT(persist_settings());

            vTaskDelay(INIT_DELAY / portTICK_PERIOD_MS);
            temperature_offset = get_temperature_offset();
        }

        if(sensor_altitude != SENSOR_ALTITUDE) {
            ESP_LOGW(SENSORS_TAG, "Sensor altitude calibration from %d m to %d m",
                     sensor_altitude, SENSOR_ALTITUDE);

            vTaskDelay(INIT_DELAY / portTICK_PERIOD_MS);
            ESP_ERROR_CHECK_WITHOUT_ABORT(set_sensor_altitude(SENSOR_ALTITUDE));

            vTaskDelay(INIT_DELAY / portTICK_PERIOD_MS);
            ESP_ERROR_CHECK_WITHOUT_ABORT(persist_settings());

            vTaskDelay(INIT_DELAY / portTICK_PERIOD_MS);
            sensor_altitude = get_sensor_altitude();
        }
        ESP_LOGI(SENSORS_TAG, "Temperature offset %.1f °%c - Sensor altitude %d %s",
                 temperature_offset, scale, sensor_altitude, scale == SCALE_CELCIUS ? "m" : "ft");
    } else {
        ESP_LOGE(SENSORS_TAG, "Sensor offset/altitude read error!");
    }

    vTaskDelay(INIT_DELAY / portTICK_PERIOD_MS);

    for(;;) {
        start_periodic_measurement();

        sensors_values_t sensors_values = {
            .co2 = 0x00,
            .temperature = 0x00,
            .humidity = 0x00
        };
        vTaskDelay(INIT_DELAY / portTICK_PERIOD_MS);

        if(read_measurement(&sensors_values) != ESP_OK) {
            ESP_LOGE(SENSORS_TAG, "Sensors read measurement error!");
        }
        vTaskDelay(INIT_DELAY / portTICK_PERIOD_MS);

        float co2_level = sensors_values.co2;
        float temperature = sensors_values.temperature;
        float humidity = sensors_values.humidity;

        #if defined(SENSORS_SCALE_F)
        temperature = FAHRENHEIT(temperature);
        #elif defined(SENSORS_SCALE_K)
        temperature = KELVIN(temperature);
        #endif

        ESP_LOG_BUFFER_HEX_LEVEL(SENSORS_TAG, &sensors_values, sizeof(sensors_values), ESP_LOG_DEBUG);

        stop_periodic_measurement();

        ESP_LOGI(SENSORS_TAG, "CO₂ %4.0f ppm - Temperature %2.1f °%c - Humidity %2.1f%%",
                 co2_level, temperature, scale, humidity);
        vTaskDelay(SLEEP_DELAY / portTICK_PERIOD_MS);
    }
}

void app_main() {
    esp_log_level_set(MAIN_TAG, ESP_LOG_INFO);
    ESP_LOGI(MAIN_TAG, "Hello there!");

    xTaskCreate(sensors_task, "sensors_task", UART_STACK_SIZE, NULL, configMAX_PRIORITIES-9, NULL);
}
