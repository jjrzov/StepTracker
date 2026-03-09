#ifndef LSM6DS3_H
#define LSM6DS3_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Register addresses
#define LSM6DS3_WHO_AM_I        0x0F
#define LSM6DS3_ID              0x69
#define LSM6DS3_CTRL1_XL        0x10
#define LSM6DS3_CTRL2_G         0x11
#define LSM6DS3_CTRL3_C         0x12
#define LSM6DS3_STATUS_REG      0x1E
#define LSM6DS3_OUTX_L_G        0x22
#define LSM6DS3_OUTX_L_XL       0x28

typedef struct {
    void *handle;  // your I2C device handle
    int32_t (*write_reg)(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
    int32_t (*read_reg)(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
} lsm6ds3_ctx_t;

typedef struct {
    i2c_master_dev_handle_t dev_handle;
} lsm6ds3_config_t;

typedef struct {
    lsm6ds3_ctx_t ctx;
    bool accel_calibrated;
    bool gyro_calibrated;
    float accel_offset[3];
    float gyro_offset[3];
} lsm6ds3_handle_t;



esp_err_t lsm6ds3_init(lsm6ds3_handle_t *handle, const lsm6ds3_config_t *config);
esp_err_t lsm6ds3_read_accel(lsm6ds3_handle_t *handle, float accel_mg[3]);
esp_err_t lsm6ds3_read_gyro(lsm6ds3_handle_t *handle, float gyro_mdps[3]);
esp_err_t lsm6ds3_calibrate_gyro(lsm6ds3_handle_t *handle, uint16_t num_samples);
esp_err_t lsm6ds3_calibrate_accel(lsm6ds3_handle_t *handle, uint16_t num_samples);

#endif