#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "lsm6ds3.h"

static const char *TAG = "MAIN";

#define I2C_SDA_PIN     GPIO_NUM_21
#define I2C_SCL_PIN     GPIO_NUM_22
#define I2C_FREQ_HZ     400000
#define LSM6DS3_ADDR    0x6A

void app_main(void)
{
    // --- I2C Bus Setup ---
    i2c_master_bus_handle_t bus_handle;
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));
    ESP_LOGI(TAG, "I2C bus initialized");

    // --- Add LSM6DS3 Device ---
    i2c_master_dev_handle_t dev_handle;
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = LSM6DS3_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle));
    ESP_LOGI(TAG, "LSM6DS3 device added");

    // --- Init LSM6DS3 ---
    lsm6ds3_handle_t imu;
    lsm6ds3_config_t imu_config = {
        .dev_handle = dev_handle,
    };

    if (lsm6ds3_init(&imu, &imu_config) != ESP_OK) {
        ESP_LOGE(TAG, "LSM6DS3 init failed, halting");
        return;
    }

    // --- Calibrate ---
    ESP_LOGI(TAG, "Starting calibration, keep sensor still and flat...");
    vTaskDelay(pdMS_TO_TICKS(2000));  // give yourself time to set the sensor down

    if (lsm6ds3_calibrate_gyro(&imu, 200) != ESP_OK) {
        ESP_LOGE(TAG, "Gyro calibration failed");
        return;
    }

    if (lsm6ds3_calibrate_accel(&imu, 200) != ESP_OK) {
        ESP_LOGE(TAG, "Accel calibration failed");
        return;
    }

    ESP_LOGI(TAG, "Calibration complete, starting readings...");

    // --- Read Loop ---
    float accel[3], gyro[3];

    while (1) {
        if (lsm6ds3_read_accel(&imu, accel) == ESP_OK) {
            ESP_LOGI(TAG, "Accel (mg) X=%.2f Y=%.2f Z=%.2f", accel[0], accel[1], accel[2]);
        } else {
            ESP_LOGE(TAG, "Accel read failed");
        }

        if (lsm6ds3_read_gyro(&imu, gyro) == ESP_OK) {
            ESP_LOGI(TAG, "Gyro (mdps) X=%.2f Y=%.2f Z=%.2f", gyro[0], gyro[1], gyro[2]);
        } else {
            ESP_LOGE(TAG, "Gyro read failed");
        }

        vTaskDelay(pdMS_TO_TICKS(100));  // 10Hz print rate
    }
}