# ESP32 SCD4x Library
Implementation of the Sensirion SCD4x carbon dioxide, temperature, and relative humidity sensor. It enables developers to communicate with the SCD4x sensor on the ESP32 platform by only adapting the I2C communication channel.

[<center><img src="images/SCD4x.png" width="300px"></center>](images/SCD4x.png)

## Config
### Select Features
Edit the [`scd4x.h`](scd4x.h) header to select the GPIO used for the I2C interface.
```
#define I2C_MASTER_SDA      (GPIO_NUM_6)
#define I2C_MASTER_SCL      (GPIO_NUM_7)
```

#### Example
The [`main.c`](main.c) file is an example of how you could use the SCD4x sensor family with an ESP32 microcontroller.

Configure the I2C interface by setting the SDA and SCL GPIO, as well as buffer size and other optional settings.

```
i2c_config_t i2c_config = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = I2C_MASTER_SDA,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_io_num = I2C_MASTER_SCL,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = I2C_MASTER_FREQ_HZ
};

ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &i2c_config));
ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, i2c_config.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0));
```

The typical communication sequence between the I2C master (e.g., an ESP32 microcontroller) and the SCD4x sensor is as follows:
1. The sensor is powered up
2. The I2C master sends a start_periodic_measurement command. Signal update interval is 5 seconds.
3. The I2C master periodically reads out data with the read measurement sequence.
4. To put the sensor back to idle mode, the I2C master sends a stop periodic measurement command.

```
scd4x_start_periodic_measurement();

scd4x_sensors_values_t sensors_values = {
    .co2 = 0x00,
    .temperature = 0x00,
    .humidity = 0x00
};

if(scd4x_read_measurement(&sensors_values) != ESP_OK) {
    ESP_LOGE(SENSORS_TAG, "Sensors read measurement error!");
}

float co2_level = sensors_values.co2;
float temperature = sensors_values.temperature;
float humidity = sensors_values.humidity;

scd4x_stop_periodic_measurement();
```
