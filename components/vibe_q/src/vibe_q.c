/**
 * @file vibe_q.c
 * @brief Vibe Q / MPU6050 direct-I2C vibration sensor implementation.
 */

#include "vibe_q.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "log_config.h"
#undef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL LOG_LEVEL_VIB
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

static const char *TAG = "vibe_q";

#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1A
#define MPU6050_GYRO_CONFIG  0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_PWR_MGMT_1   0x6B

#define VIBE_Q_TASK_STACK_SIZE 4096
#define VIBE_Q_TASK_PRIORITY   (tskIDLE_PRIORITY + 2)
#define VIBE_Q_TIMEOUT_MS      100

#define ALPHA_LPF          0.40f
#define ALPHA_DC           0.01f
#define NOISE_THRESHOLD_G  0.018f
#define GRAVITY_MS2        9.80665f
#define PI_VAL             3.14159265358979323846f
#define SAMPLE_WINDOW_MS   250
#define DEBOUNCE_MICROS    4000
#define STATIC_CLEAR_LIMIT  3

static const float HYSTERESIS_MS2 = 0.020f * GRAVITY_MS2;

typedef struct {
    double sum_vel_rms;
    double sum_accel_rms;
    double sum_disp_x;
    double sum_disp_y;
    double sum_disp_z;
    double sum_freq_x;
    double sum_freq_y;
    double sum_freq_z;
    double sum_dom_freq;
    float max_vel_rms;
    float max_accel_rms;
    float last_temp;
    uint32_t sample_count;
    uint32_t error_count;
} vibe_q_stats_accum_t;

static i2c_master_bus_handle_t s_bus = NULL;
static i2c_master_dev_handle_t s_dev = NULL;
static SemaphoreHandle_t s_mutex = NULL;
static TaskHandle_t s_task = NULL;
static volatile bool s_running = false;
static volatile vibe_q_status_t s_status = VIBE_Q_STATUS_STARTING;
static wtvb01_data_t s_data = {0};
static vibe_q_stats_accum_t s_accum = {0};

static void stats_reset_locked(void)
{
    memset(&s_accum, 0, sizeof(s_accum));
}

static void stats_accumulate_locked(const wtvb01_data_t *d)
{
    s_accum.sum_vel_rms += d->velocity_rms;
    s_accum.sum_accel_rms += d->accel_rms;
    s_accum.sum_disp_x += d->displacement_x;
    s_accum.sum_disp_y += d->displacement_y;
    s_accum.sum_disp_z += d->displacement_z;
    s_accum.sum_freq_x += d->frequency_x;
    s_accum.sum_freq_y += d->frequency_y;
    s_accum.sum_freq_z += d->frequency_z;
    s_accum.sum_dom_freq += d->dominant_frequency;
    if (d->velocity_rms > s_accum.max_vel_rms) {
        s_accum.max_vel_rms = d->velocity_rms;
    }
    if (d->accel_rms > s_accum.max_accel_rms) {
        s_accum.max_accel_rms = d->accel_rms;
    }
    s_accum.last_temp = d->temperature;
    s_accum.sample_count++;
}

static esp_err_t write_reg(uint8_t reg_addr, uint8_t value)
{
    uint8_t write_buf[2] = {reg_addr, value};
    return i2c_master_transmit(s_dev, write_buf, sizeof(write_buf),
                               pdMS_TO_TICKS(VIBE_Q_TIMEOUT_MS));
}

static esp_err_t mpu6050_configure(void)
{
    esp_err_t ret = write_reg(MPU6050_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) {
        return ret;
    }
    (void)write_reg(MPU6050_SMPLRT_DIV, 0x00);
    (void)write_reg(MPU6050_CONFIG, 0x00);
    (void)write_reg(MPU6050_GYRO_CONFIG, 0x08);
    return write_reg(MPU6050_ACCEL_CONFIG, 0x10);
}

static esp_err_t mpu6050_read_data(float *acc_x, float *acc_y, float *acc_z,
                                   float *temp)
{
    if (!acc_x || !acc_y || !acc_z || !temp) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t reg = MPU6050_ACCEL_XOUT_H;
    uint8_t data[14] = {0};
    esp_err_t ret = i2c_master_transmit_receive(
        s_dev, &reg, 1, data, sizeof(data), pdMS_TO_TICKS(VIBE_Q_TIMEOUT_MS));
    if (ret != ESP_OK) {
        return ret;
    }

    int16_t raw_acc_x = (int16_t)((data[0] << 8) | data[1]);
    int16_t raw_acc_y = (int16_t)((data[2] << 8) | data[3]);
    int16_t raw_acc_z = (int16_t)((data[4] << 8) | data[5]);
    int16_t raw_temp = (int16_t)((data[6] << 8) | data[7]);

    *acc_x = ((float)raw_acc_x) / 4096.0f;
    *acc_y = ((float)raw_acc_y) / 4096.0f;
    *acc_z = ((float)raw_acc_z) / 4096.0f;
    *temp = (((float)raw_temp) / 340.0f) + 36.53f;
    return ESP_OK;
}

static float max3f(float a, float b, float c)
{
    float m = (a > b) ? a : b;
    return (m > c) ? m : c;
}

static void publish_sample(const wtvb01_data_t *sample, bool accumulate_stats,
                           bool reset_stats)
{
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return;
    }

    if (reset_stats) {
        stats_reset_locked();
    }
    s_data = *sample;
    if (accumulate_stats) {
        stats_accumulate_locked(sample);
    }
    s_status = VIBE_Q_STATUS_OK;

    xSemaphoreGive(s_mutex);
}

static void publish_error(void)
{
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(20)) != pdTRUE) {
        return;
    }
    s_data.error_count++;
    s_accum.error_count++;
    xSemaphoreGive(s_mutex);
}

static void publish_starting_zero(float temperature, uint32_t error_count)
{
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return;
    }

    stats_reset_locked();
    s_accum.error_count = error_count;
    wtvb01_data_t sample = {
        .temperature = temperature,
        .data_valid = true,
        .error_count = error_count,
    };
    s_data = sample;
    stats_accumulate_locked(&sample);
    s_status = VIBE_Q_STATUS_STARTING;

    xSemaphoreGive(s_mutex);
}

static void publish_i2c_fault(void)
{
    if (s_mutex != NULL &&
        xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        uint32_t error_count = s_accum.error_count;
        stats_reset_locked();
        s_accum.error_count = error_count;
        s_data.data_valid = false;
        s_data.error_count = error_count;
        s_status = VIBE_Q_STATUS_I2C_COMM_FAULT;
        xSemaphoreGive(s_mutex);
    } else {
        s_status = VIBE_Q_STATUS_I2C_COMM_FAULT;
    }
}

static void vibe_q_task(void *arg)
{
    (void)arg;

    float filt_x = 0.0f;
    float filt_y = 0.0f;
    float filt_z = 0.0f;
    float dc_x = 0.0f;
    float dc_y = 0.0f;
    float dc_z = 0.0f;
    uint32_t read_count = 0;
    uint32_t total_errors = 0;
    int static_counter = 0;
    bool static_latched = false;
    bool sensor_online = false;

    while (s_running) {
        if (!sensor_online) {
            esp_err_t init_ret = mpu6050_configure();
            if (init_ret == ESP_OK) {
                float init_x = 0.0f;
                float init_y = 0.0f;
                float init_z = 0.0f;
                float init_temp = 0.0f;
                if (mpu6050_read_data(&init_x, &init_y, &init_z, &init_temp) == ESP_OK) {
                    filt_x = dc_x = init_x * GRAVITY_MS2;
                    filt_y = dc_y = init_y * GRAVITY_MS2;
                    filt_z = dc_z = init_z * GRAVITY_MS2;
                    sensor_online = true;
                    publish_starting_zero(init_temp, total_errors);
                    ESP_LOGI(TAG, "Vibe Q MPU6050 ready on I2C%d SDA=%d SCL=%d",
                             VIBE_Q_I2C_PORT, VIBE_Q_SDA_PIN, VIBE_Q_SCL_PIN);
                }
            }
            if (!sensor_online) {
                total_errors++;
                publish_error();
                publish_i2c_fault();
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
        }

        int64_t window_start_us = esp_timer_get_time();
        int sample_count = 0;
        int window_errors = 0;
        float sum_sq_x = 0.0f;
        float sum_sq_y = 0.0f;
        float sum_sq_z = 0.0f;
        bool state_x = false;
        bool state_y = false;
        bool state_z = false;
        int cycles_x = 0;
        int cycles_y = 0;
        int cycles_z = 0;
        int64_t first_edge_x = 0;
        int64_t first_edge_y = 0;
        int64_t first_edge_z = 0;
        int64_t last_edge_x = 0;
        int64_t last_edge_y = 0;
        int64_t last_edge_z = 0;
        float last_temp = 0.0f;

        while (s_running &&
               (esp_timer_get_time() - window_start_us) < (SAMPLE_WINDOW_MS * 1000LL)) {
            float raw_x_g = 0.0f;
            float raw_y_g = 0.0f;
            float raw_z_g = 0.0f;
            float temp_c = 0.0f;

            esp_err_t read_ret = mpu6050_read_data(&raw_x_g, &raw_y_g, &raw_z_g, &temp_c);
            if (read_ret == ESP_OK) {
                window_errors = 0;
                last_temp = temp_c;

                float acc_x_ms2 = raw_x_g * GRAVITY_MS2;
                float acc_y_ms2 = raw_y_g * GRAVITY_MS2;
                float acc_z_ms2 = raw_z_g * GRAVITY_MS2;

                filt_x = (ALPHA_LPF * acc_x_ms2) + ((1.0f - ALPHA_LPF) * filt_x);
                filt_y = (ALPHA_LPF * acc_y_ms2) + ((1.0f - ALPHA_LPF) * filt_y);
                filt_z = (ALPHA_LPF * acc_z_ms2) + ((1.0f - ALPHA_LPF) * filt_z);

                dc_x = (ALPHA_DC * filt_x) + ((1.0f - ALPHA_DC) * dc_x);
                dc_y = (ALPHA_DC * filt_y) + ((1.0f - ALPHA_DC) * dc_y);
                dc_z = (ALPHA_DC * filt_z) + ((1.0f - ALPHA_DC) * dc_z);

                float ac_x = filt_x - dc_x;
                float ac_y = filt_y - dc_y;
                float ac_z = filt_z - dc_z;

                sum_sq_x += ac_x * ac_x;
                sum_sq_y += ac_y * ac_y;
                sum_sq_z += ac_z * ac_z;
                sample_count++;

                int64_t now_us = esp_timer_get_time();
                if (!state_x && ac_x > HYSTERESIS_MS2) {
                    if ((now_us - last_edge_x) > DEBOUNCE_MICROS) {
                        state_x = true;
                        cycles_x++;
                        if (cycles_x == 1) {
                            first_edge_x = now_us;
                        }
                        last_edge_x = now_us;
                    }
                } else if (state_x && ac_x < -HYSTERESIS_MS2) {
                    state_x = false;
                }

                if (!state_y && ac_y > HYSTERESIS_MS2) {
                    if ((now_us - last_edge_y) > DEBOUNCE_MICROS) {
                        state_y = true;
                        cycles_y++;
                        if (cycles_y == 1) {
                            first_edge_y = now_us;
                        }
                        last_edge_y = now_us;
                    }
                } else if (state_y && ac_y < -HYSTERESIS_MS2) {
                    state_y = false;
                }

                if (!state_z && ac_z > HYSTERESIS_MS2) {
                    if ((now_us - last_edge_z) > DEBOUNCE_MICROS) {
                        state_z = true;
                        cycles_z++;
                        if (cycles_z == 1) {
                            first_edge_z = now_us;
                        }
                        last_edge_z = now_us;
                    }
                } else if (state_z && ac_z < -HYSTERESIS_MS2) {
                    state_z = false;
                }
            } else {
                total_errors++;
                publish_error();
                if (++window_errors > 5) {
                    sensor_online = false;
                    publish_i2c_fault();
                    break;
                }
            }
            taskYIELD();
        }

        if (!s_running) {
            break;
        }
        if (!sensor_online || sample_count == 0) {
            continue;
        }

        float rms_x = sqrtf(sum_sq_x / (float)sample_count) / GRAVITY_MS2;
        float rms_y = sqrtf(sum_sq_y / (float)sample_count) / GRAVITY_MS2;
        float rms_z = sqrtf(sum_sq_z / (float)sample_count) / GRAVITY_MS2;
        float amp_acc_x = rms_x * 1.4142f;
        float amp_acc_y = rms_y * 1.4142f;
        float amp_acc_z = rms_z * 1.4142f;

        float freq_x = 0.0f;
        float freq_y = 0.0f;
        float freq_z = 0.0f;
        if (cycles_x > 1 && last_edge_x > first_edge_x) {
            freq_x = ((float)(cycles_x - 1) * 1000000.0f) /
                     (float)(last_edge_x - first_edge_x);
        }
        if (cycles_y > 1 && last_edge_y > first_edge_y) {
            freq_y = ((float)(cycles_y - 1) * 1000000.0f) /
                     (float)(last_edge_y - first_edge_y);
        }
        if (cycles_z > 1 && last_edge_z > first_edge_z) {
            freq_z = ((float)(cycles_z - 1) * 1000000.0f) /
                     (float)(last_edge_z - first_edge_z);
        }

        float amp_vel_x = 0.0f;
        float amp_vel_y = 0.0f;
        float amp_vel_z = 0.0f;
        float amp_disp_x = 0.0f;
        float amp_disp_y = 0.0f;
        float amp_disp_z = 0.0f;

        if (amp_acc_x >= NOISE_THRESHOLD_G && freq_x > 10.0f) {
            float omega = 2.0f * PI_VAL * freq_x;
            float acc_ms2 = amp_acc_x * GRAVITY_MS2;
            amp_vel_x = (acc_ms2 / omega) * 1000.0f;
            amp_disp_x = (acc_ms2 / (omega * omega)) * 1000000.0f;
        } else {
            freq_x = 0.0f;
        }

        if (amp_acc_y >= NOISE_THRESHOLD_G && freq_y > 10.0f) {
            float omega = 2.0f * PI_VAL * freq_y;
            float acc_ms2 = amp_acc_y * GRAVITY_MS2;
            amp_vel_y = (acc_ms2 / omega) * 1000.0f;
            amp_disp_y = (acc_ms2 / (omega * omega)) * 1000000.0f;
        } else {
            freq_y = 0.0f;
        }

        if (amp_acc_z >= NOISE_THRESHOLD_G && freq_z > 10.0f) {
            float omega = 2.0f * PI_VAL * freq_z;
            float acc_ms2 = amp_acc_z * GRAVITY_MS2;
            amp_vel_z = (acc_ms2 / omega) * 1000.0f;
            amp_disp_z = (acc_ms2 / (omega * omega)) * 1000000.0f;
        } else {
            freq_z = 0.0f;
        }

        float accel_rms = sqrtf((amp_acc_x * amp_acc_x) +
                                (amp_acc_y * amp_acc_y) +
                                (amp_acc_z * amp_acc_z)) / 1.4142f;
        float velocity_rms = sqrtf((amp_vel_x * amp_vel_x) +
                                   (amp_vel_y * amp_vel_y) +
                                   (amp_vel_z * amp_vel_z)) / 1.4142f;

        bool reset_stats = false;
        if (velocity_rms < 0.05f) {
            if (static_counter < STATIC_CLEAR_LIMIT) {
                static_counter++;
            }
            if (static_counter >= STATIC_CLEAR_LIMIT) {
                /* Sustained stillness is a valid zero-vibration reading.
                 * Clear earlier motion once when entering the static state,
                 * then keep accumulating zero samples so livedata never
                 * mistakes a healthy stationary sensor for missing data. */
                reset_stats = !static_latched;
                static_latched = true;
                static_counter = STATIC_CLEAR_LIMIT;

                amp_vel_x = 0.0f;
                amp_vel_y = 0.0f;
                amp_vel_z = 0.0f;
                velocity_rms = 0.0f;
                amp_disp_x = 0.0f;
                amp_disp_y = 0.0f;
                amp_disp_z = 0.0f;
            }
        } else {
            static_counter = 0;
            static_latched = false;
        }

        wtvb01_data_t sample = {
            .velocity_x = amp_vel_x,
            .velocity_y = amp_vel_y,
            .velocity_z = amp_vel_z,
            .velocity_rms = velocity_rms,
            .displacement_x = amp_disp_x,
            .displacement_y = amp_disp_y,
            .displacement_z = amp_disp_z,
            .accel_x = amp_acc_x,
            .accel_y = amp_acc_y,
            .accel_z = amp_acc_z,
            .accel_rms = accel_rms,
            .frequency_x = freq_x,
            .frequency_y = freq_y,
            .frequency_z = freq_z,
            .dominant_frequency = max3f(freq_x, freq_y, freq_z),
            .temperature = last_temp,
            .data_valid = true,
            .read_count = ++read_count,
            .error_count = total_errors,
        };
        publish_sample(&sample, true, reset_stats);
        vTaskDelay(pdMS_TO_TICKS(750));
    }

    s_task = NULL;
    vTaskDelete(NULL);
}

esp_err_t vibe_q_init(void)
{
    if (s_running) {
        return ESP_OK;
    }

    s_status = VIBE_Q_STATUS_STARTING;

    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) {
        s_status = VIBE_Q_STATUS_I2C_COMM_FAULT;
        return ESP_ERR_NO_MEM;
    }

    memset(&s_data, 0, sizeof(s_data));
    memset(&s_accum, 0, sizeof(s_accum));

    i2c_master_bus_config_t bus_config = {
        .i2c_port = VIBE_Q_I2C_PORT,
        .sda_io_num = (gpio_num_t)VIBE_Q_SDA_PIN,
        .scl_io_num = (gpio_num_t)VIBE_Q_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = true,
        },
    };

    esp_err_t ret = i2c_new_master_bus(&bus_config, &s_bus);
    if (ret != ESP_OK) {
        s_status = VIBE_Q_STATUS_I2C_COMM_FAULT;
        vSemaphoreDelete(s_mutex);
        s_mutex = NULL;
        ESP_LOGE(TAG, "Failed to create Vibe Q I2C%d bus: %s",
                 VIBE_Q_I2C_PORT, esp_err_to_name(ret));
        return ret;
    }

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = VIBE_Q_MPU6050_ADDR,
        .scl_speed_hz = VIBE_Q_I2C_FREQ_HZ,
        .scl_wait_us = 20000,
    };

    ret = i2c_master_bus_add_device(s_bus, &dev_config, &s_dev);
    if (ret != ESP_OK) {
        s_status = VIBE_Q_STATUS_I2C_COMM_FAULT;
        ESP_LOGE(TAG, "Failed to add Vibe Q MPU6050 @0x%02X: %s",
                 VIBE_Q_MPU6050_ADDR, esp_err_to_name(ret));
        i2c_del_master_bus(s_bus);
        s_bus = NULL;
        vSemaphoreDelete(s_mutex);
        s_mutex = NULL;
        return ret;
    }

    ret = mpu6050_configure();
    if (ret != ESP_OK) {
        s_status = VIBE_Q_STATUS_I2C_COMM_FAULT;
        ESP_LOGW(TAG, "Vibe Q MPU6050 not detected yet: %s; task will retry",
                 esp_err_to_name(ret));
    } else {
        /* The sensor is present and configured, but the background task may
         * not have completed its first measurement window yet.  Seed one
         * valid zero sample so livedata reports 0 instead of null during
         * that short startup interval. */
        wtvb01_data_t initial_sample = {
            .data_valid = true,
        };
        s_data = initial_sample;
        stats_accumulate_locked(&initial_sample);
        s_status = VIBE_Q_STATUS_STARTING;
    }

    s_running = true;
    BaseType_t task_ret = xTaskCreate(vibe_q_task, "vibe_q",
                                      VIBE_Q_TASK_STACK_SIZE, NULL,
                                      VIBE_Q_TASK_PRIORITY, &s_task);
    if (task_ret != pdPASS) {
        s_running = false;
        s_status = VIBE_Q_STATUS_I2C_COMM_FAULT;
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
        i2c_del_master_bus(s_bus);
        s_bus = NULL;
        vSemaphoreDelete(s_mutex);
        s_mutex = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Vibe Q driver started on I2C%d SDA=%d SCL=%d freq=%d",
             VIBE_Q_I2C_PORT, VIBE_Q_SDA_PIN, VIBE_Q_SCL_PIN,
             VIBE_Q_I2C_FREQ_HZ);
    return ESP_OK;
}

esp_err_t vibe_q_deinit(void)
{
    if (!s_running && s_bus == NULL && s_dev == NULL) {
        return ESP_OK;
    }

    s_running = false;
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(1500);
    while (s_task != NULL && xTaskGetTickCount() < deadline) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (s_task != NULL) {
        ESP_LOGW(TAG, "Vibe Q task did not stop before timeout");
        vTaskDelete(s_task);
        s_task = NULL;
    }

    if (s_dev != NULL) {
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
    }
    if (s_bus != NULL) {
        i2c_del_master_bus(s_bus);
        s_bus = NULL;
    }
    if (s_mutex != NULL) {
        vSemaphoreDelete(s_mutex);
        s_mutex = NULL;
    }

    ESP_LOGI(TAG, "Vibe Q driver stopped");
    return ESP_OK;
}

esp_err_t vibe_q_read_all(wtvb01_data_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_running || s_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    *data = s_data;
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t vibe_q_get_stats(wtvb01_stats_t *stats, bool reset)
{
    if (stats == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_running || s_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    uint32_t n = s_accum.sample_count;
    if (n == 0) {
        memset(stats, 0, sizeof(*stats));
        stats->error_count = s_accum.error_count;
        xSemaphoreGive(s_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    float inv_n = 1.0f / (float)n;
    stats->velocity_rms_avg = (float)(s_accum.sum_vel_rms * inv_n);
    stats->velocity_rms_max = s_accum.max_vel_rms;
    stats->accel_rms_avg = (float)(s_accum.sum_accel_rms * inv_n);
    stats->accel_rms_max = s_accum.max_accel_rms;
    stats->displacement_x_avg = (float)(s_accum.sum_disp_x * inv_n);
    stats->displacement_y_avg = (float)(s_accum.sum_disp_y * inv_n);
    stats->displacement_z_avg = (float)(s_accum.sum_disp_z * inv_n);
    stats->frequency_x_avg = (float)(s_accum.sum_freq_x * inv_n);
    stats->frequency_y_avg = (float)(s_accum.sum_freq_y * inv_n);
    stats->frequency_z_avg = (float)(s_accum.sum_freq_z * inv_n);
    stats->dominant_freq_avg = (float)(s_accum.sum_dom_freq * inv_n);
    stats->temperature = s_accum.last_temp;
    stats->sample_count = n;
    stats->error_count = s_accum.error_count;

    if (reset) {
        stats_reset_locked();
    }

    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

vibe_q_status_t vibe_q_get_status(void)
{
    return s_status;
}

const char *vibe_q_status_name(vibe_q_status_t status)
{
    switch (status) {
    case VIBE_Q_STATUS_OK:
        return "OK";
    case VIBE_Q_STATUS_I2C_COMM_FAULT:
        return "I2C_COMM_FAULT";
    case VIBE_Q_STATUS_STARTING:
    default:
        return "STARTING";
    }
}


