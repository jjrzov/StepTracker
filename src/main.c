#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "lsm6ds3.h"
#include "lsm6ds3_fusion.h"

static const char *TAG = "MAIN";

// Hardware
#define I2C_SDA_PIN     GPIO_NUM_21
#define I2C_SCL_PIN     GPIO_NUM_22
#define I2C_FREQ_HZ     400000
#define LSM6DS3_ADDR    0x6A

// Task priorities
#define IMU_TASK_PRIORITY       5
#define FUSION_TASK_PRIORITY    4
#define OUTPUT_TASK_PRIORITY    3

// Sample rate
#define SAMPLE_RATE_HZ          104
#define SAMPLE_PERIOD_MS        (1000 / SAMPLE_RATE_HZ)

// Raw IMU data passed between tasks via queue
typedef struct {
    float accel_mg[3];
    float gyro_mdps[3];
} imu_data_t;

// Fused output shared between fusion and output tasks
typedef struct {
    lsm6ds3_euler_angles_t euler;
    float angle_from_vertical;
    uint32_t step_count;
    float distance_m;
} fusion_result_t;

// Shared resources
static QueueHandle_t imu_queue;
static SemaphoreHandle_t fusion_mutex;
static fusion_result_t shared_result;
static lsm6ds3_handle_t imu;

// --- IMU Task ---
static void imu_task(void *arg)
{
    imu_data_t data;

    while (1) {
        if (lsm6ds3_read_accel(&imu, data.accel_mg) != ESP_OK) {
            ESP_LOGE(TAG, "Accel read failed");
        }
        if (lsm6ds3_read_gyro(&imu, data.gyro_mdps) != ESP_OK) {
            ESP_LOGE(TAG, "Gyro read failed");
        }

        // Drop oldest if queue full — never block the sensor read
        if (xQueueSend(imu_queue, &data, 0) != pdTRUE) {
            ESP_LOGW(TAG, "IMU queue full, dropping sample");
        }

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}

// --- Fusion Task ---
static void fusion_task(void *arg)
{
    lsm6ds3_madgwick_filter_t filter;
    lsm6ds3_madgwick_init(&filter, 0.01f, SAMPLE_RATE_HZ);

    imu_data_t data;
    const float dt = 1.0f / SAMPLE_RATE_HZ;
    float gyro_dps[3];

    // Step detection state
    float filtered_mag = 1000.0f;
    bool above_threshold = false;
    TickType_t last_step_time = xTaskGetTickCount();
    uint32_t step_count = 0;

    // Tunable parameters
    const float STEP_THRESHOLD = 1150.0f;
    const float LOW_PASS_ALPHA = 0.2f;
    const uint32_t MIN_STEP_MS = 500;
    const uint32_t MAX_STEP_MS = 2000;
    const float MIN_RANGE = 150.0f;

    // Stride estimation state
    float step_max_mag = 0.0f;
    float step_min_mag = 2000.0f;
    float total_distance_m = 0.0f;

    while (1) {
        if (xQueueReceive(imu_queue, &data, portMAX_DELAY) == pdTRUE) {

            // --- Sensor Fusion ---
            gyro_dps[0] = data.gyro_mdps[0] / 1000.0f;
            gyro_dps[1] = data.gyro_mdps[1] / 1000.0f;
            gyro_dps[2] = data.gyro_mdps[2] / 1000.0f;

            lsm6ds3_madgwick_update(&filter, data.accel_mg, gyro_dps, dt);

            lsm6ds3_euler_angles_t euler;
            lsm6ds3_madgwick_get_euler(&filter, &euler);
            float angle_from_vertical = lsm6ds3_calculate_angle_from_vertical(
                euler.roll, euler.pitch);

            // --- Step Detection ---
            float mag = sqrtf(
                data.accel_mg[0] * data.accel_mg[0] +
                data.accel_mg[1] * data.accel_mg[1] +
                data.accel_mg[2] * data.accel_mg[2]
            );

            filtered_mag = LOW_PASS_ALPHA * mag + (1.0f - LOW_PASS_ALPHA) * filtered_mag;

            TickType_t now = xTaskGetTickCount();
            TickType_t ticks_since_last = now - last_step_time;
            uint32_t ms_since_last = pdTICKS_TO_MS(ticks_since_last);

            // Track min/max for stride estimation
            if (filtered_mag > step_max_mag) step_max_mag = filtered_mag;
            if (filtered_mag < step_min_mag) step_min_mag = filtered_mag;

            // Add separate above_threshold timer
            TickType_t above_threshold_time = xTaskGetTickCount();

            // Replace the MAX_STEP_MS reset block with this
            if (above_threshold && (pdTICKS_TO_MS(now - above_threshold_time) > MAX_STEP_MS)) {
                above_threshold = false;
                step_max_mag = 0.0f;
                step_min_mag = 2000.0f;
            }

            // Update above_threshold_time when rising edge fires
            if (!above_threshold && filtered_mag > STEP_THRESHOLD) {
                above_threshold = true;
                above_threshold_time = now;
                ESP_LOGI("STEP", "rising edge %.1f", filtered_mag);
            } else if (above_threshold && filtered_mag < STEP_THRESHOLD) {
                above_threshold = false;
                float range = step_max_mag - step_min_mag;
                ESP_LOGI("STEP", "falling edge ms=%lu range=%.1f", ms_since_last, range);

                if (ms_since_last > MIN_STEP_MS && range > MIN_RANGE) {
                    step_count++;
                    last_step_time = now;
                    float step_length = 0.75f * (500.0f / (float)ms_since_last);
                    step_length = fmaxf(0.4f, fminf(1.2f, step_length));
                    total_distance_m += step_length;
                }

                step_max_mag = 0.0f;
                step_min_mag = 2000.0f;
            }

            // --- Update shared result ---
            if (xSemaphoreTake(fusion_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                shared_result.euler = euler;
                shared_result.angle_from_vertical = angle_from_vertical;
                shared_result.step_count = step_count;
                shared_result.distance_m = total_distance_m;
                xSemaphoreGive(fusion_mutex);
            }
        }
    }
}

// --- Output Task ---
static void output_task(void *arg)
{
    fusion_result_t result;

    while (1) {
        if (xSemaphoreTake(fusion_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            memcpy(&result, &shared_result, sizeof(fusion_result_t));
            xSemaphoreGive(fusion_mutex);
        }

        ESP_LOGI(TAG, "Roll=%.1f  Pitch=%.1f  Steps=%lu  Dist=%.2fm",
            result.euler.roll,
            result.euler.pitch,
            result.step_count,
            result.distance_m);

        vTaskDelay(pdMS_TO_TICKS(200));  // print at 5Hz
    }
}

void app_main(void)
{
    // --- I2C Setup ---
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

    i2c_master_dev_handle_t dev_handle;
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = LSM6DS3_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle));

    // --- IMU Init ---
    lsm6ds3_config_t imu_config = { .dev_handle = dev_handle };
    if (lsm6ds3_init(&imu, &imu_config) != ESP_OK) {
        ESP_LOGE(TAG, "IMU init failed, halting");
        return;
    }

    // --- Calibrate ---
    float test_gyro[3];
    lsm6ds3_read_gyro(&imu, test_gyro);
    ESP_LOGI(TAG, "Calibrating in 2 seconds, keep sensor still and flat...");
    vTaskDelay(pdMS_TO_TICKS(5000));
    lsm6ds3_calibrate_gyro(&imu, 200);
    lsm6ds3_calibrate_accel(&imu, 200);
    ESP_LOGI(TAG, "Calibration done, starting fusion...");

    // --- Create FreeRTOS objects ---
    imu_queue    = xQueueCreate(50, sizeof(imu_data_t));
    fusion_mutex = xSemaphoreCreateMutex();

    // --- Start Tasks ---
    xTaskCreate(imu_task,    "imu_task",    4096, NULL, IMU_TASK_PRIORITY,    NULL);
    xTaskCreate(fusion_task, "fusion_task", 4096, NULL, FUSION_TASK_PRIORITY, NULL);
    xTaskCreate(output_task, "output_task", 4096, NULL, OUTPUT_TASK_PRIORITY, NULL);
}