#include "lsm6ds3.h"
#include "esp_log.h"

static const char *TAG = "LSM6DS3";


static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) 
{
    i2c_master_dev_handle_t dev = (i2c_master_dev_handle_t)handle;

    if (bufp == NULL && len > 0) {
        ESP_LOGE(TAG, "I2C write buffer pointer is NULL");
        return -1;
    }

    // Pack reg address and data into buffer
    uint8_t write_buf[len + 1];
    write_buf[0] = reg;
    memcpy(&write_buf[1], bufp, len);

    esp_err_t ret = i2c_master_transmit(dev, write_buf, len + 1, -1);
    
    return (ret == ESP_OK) ? 0 : -1;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    i2c_master_dev_handle_t dev = (i2c_master_dev_handle_t)handle;
    
    esp_err_t ret = i2c_master_transmit_receive(dev, &reg, 1, bufp, len, -1);
    
    return (ret == ESP_OK) ? 0 : -1;
}

static void platform_delay(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

esp_err_t lsm6ds3_init(lsm6ds3_handle_t *handle, const lsm6ds3_config_t *config)
{
    if (handle == NULL || config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memset(handle, 0, sizeof(lsm6ds3_handle_t));
    handle->accel_calibrated = false;
    handle->gyro_calibrated = false;

    handle->ctx.handle = (void *)config->dev_handle;
    handle->ctx.write_reg = platform_write;
    handle->ctx.read_reg = platform_read;
    
    vTaskDelay(pdMS_TO_TICKS(100));

    uint8_t whoamI;
    if (handle->ctx.read_reg(handle->ctx.handle, LSM6DS3_WHO_AM_I, &whoamI, 1) != 0) {
        ESP_LOGE(TAG, "Failed to read device ID");
        return ESP_ERR_NOT_FOUND;
    }
    
    if (whoamI != LSM6DS3_ID) {
        ESP_LOGE(TAG, "Invalid device ID: 0x%02X (expected 0x%02X)", whoamI, LSM6DS3_ID);
        return ESP_ERR_NOT_FOUND;
    }

    // Enable block data update, auto-increment
    uint8_t ctrl3 = 0x44;
    handle->ctx.write_reg(handle->ctx.handle, LSM6DS3_CTRL3_C, &ctrl3, 1);

    // Accel: 104Hz, ±2g
    uint8_t ctrl1 = 0x40;
    handle->ctx.write_reg(handle->ctx.handle, LSM6DS3_CTRL1_XL, &ctrl1, 1);

    // Gyro: 104Hz, 250dps
    uint8_t ctrl2 = 0x40;
    handle->ctx.write_reg(handle->ctx.handle, LSM6DS3_CTRL2_G, &ctrl2, 1);

    vTaskDelay(pdMS_TO_TICKS(100));  // wait for gyro to start up

    ESP_LOGI(TAG, "LSM6DS3 initialized successfully (ID: 0x%02X)", whoamI);
    return ESP_OK;
}

esp_err_t lsm6ds3_read_accel(lsm6ds3_handle_t *handle, float accel_mg[3])
{
    if (handle == NULL || accel_mg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buf[6];
    if (handle->ctx.read_reg(handle->ctx.handle, LSM6DS3_OUTX_L_XL, buf, 6) != 0) {
        return ESP_FAIL;
    }

    int16_t raw[3];
    raw[0] = (int16_t)(buf[1] << 8 | buf[0]);
    raw[1] = (int16_t)(buf[3] << 8 | buf[2]);
    raw[2] = (int16_t)(buf[5] << 8 | buf[4]);

    // ±2g sensitivity: 0.061 mg/LSB
    accel_mg[0] = raw[0] * 0.061f;
    accel_mg[1] = raw[1] * 0.061f;
    accel_mg[2] = raw[2] * 0.061f;

    if (handle->accel_calibrated) {
        accel_mg[0] -= handle->accel_offset[0];
        accel_mg[1] -= handle->accel_offset[1];
        accel_mg[2] -= handle->accel_offset[2];
    }

    return ESP_OK;
}

esp_err_t lsm6ds3_read_gyro(lsm6ds3_handle_t *handle, float gyro_mdps[3])
{
    if (handle == NULL || gyro_mdps == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buf[6];
    if (handle->ctx.read_reg(handle->ctx.handle, LSM6DS3_OUTX_L_G, buf, 6) != 0) {
        return ESP_FAIL;
    }

    int16_t raw[3];
    raw[0] = (int16_t)(buf[1] << 8 | buf[0]);
    raw[1] = (int16_t)(buf[3] << 8 | buf[2]);
    raw[2] = (int16_t)(buf[5] << 8 | buf[4]);

    // 250dps sensitivity: 8.75 mdps/LSB
    gyro_mdps[0] = raw[0] * 8.75f;
    gyro_mdps[1] = raw[1] * 8.75f;
    gyro_mdps[2] = raw[2] * 8.75f;

    if (handle->gyro_calibrated) {
        gyro_mdps[0] -= handle->gyro_offset[0];
        gyro_mdps[1] -= handle->gyro_offset[1];
        gyro_mdps[2] -= handle->gyro_offset[2];
    }

    return ESP_OK;
}

esp_err_t lsm6ds3_calibrate_gyro(lsm6ds3_handle_t *handle, uint16_t num_samples)
{
    if (handle == NULL) return ESP_ERR_INVALID_ARG;

    float sum[3] = {0, 0, 0};
    float sample[3];

    ESP_LOGI(TAG, "Calibrating gyro, keep sensor still...");

    for (int i = 0; i < num_samples; i++) {
        if (lsm6ds3_read_gyro(handle, sample) != ESP_OK) {
            return ESP_FAIL;
        }
        sum[0] += sample[0];
        sum[1] += sample[1];
        sum[2] += sample[2];
        platform_delay(10);
    }

    handle->gyro_offset[0] = sum[0] / num_samples;
    handle->gyro_offset[1] = sum[1] / num_samples;
    handle->gyro_offset[2] = sum[2] / num_samples;
    handle->gyro_calibrated = true;

    ESP_LOGI(TAG, "Gyro offsets: X=%.2f Y=%.2f Z=%.2f mdps",
             handle->gyro_offset[0],
             handle->gyro_offset[1],
             handle->gyro_offset[2]);

    return ESP_OK;
}

esp_err_t lsm6ds3_calibrate_accel(lsm6ds3_handle_t *handle, uint16_t num_samples)
{
    if (handle == NULL) return ESP_ERR_INVALID_ARG;

    float sum[3] = {0, 0, 0};
    float sample[3];

    ESP_LOGI(TAG, "Calibrating accel, keep sensor flat and still...");

    for (int i = 0; i < num_samples; i++) {
        if (lsm6ds3_read_accel(handle, sample) != ESP_OK) {
            return ESP_FAIL;
        }
        sum[0] += sample[0];
        sum[1] += sample[1];
        sum[2] += sample[2];
        platform_delay(10);
    }

    // Average the samples
    handle->accel_offset[0] = sum[0] / num_samples;
    handle->accel_offset[1] = sum[1] / num_samples;
    handle->accel_offset[2] = sum[2] / num_samples;

    // Remove expected gravity from whichever axis is vertical
    // Assumes Z is pointing up — adjust if your mounting is different
    handle->accel_offset[2] -= 1000.0f;

    handle->accel_calibrated = true;

    ESP_LOGI(TAG, "Accel offsets: X=%.2f Y=%.2f Z=%.2f mg",
             handle->accel_offset[0],
             handle->accel_offset[1],
             handle->accel_offset[2]);

    return ESP_OK;
}