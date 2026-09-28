/**
 * @file vibe_q.h
 * @brief Vibe Q / MPU6050 direct-I2C vibration sensor component.
 *
 * Hardware:
 *   - I2C_NUM_1, SDA=GPIO13, SCL=GPIO14
 *   - MPU6050 address 0x68
 *
 * This component intentionally uses a dedicated I2C controller so it does not
 * touch the Datameter TCA9548A mux bus on I2C_NUM_0.
 */

#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "wtvb01_485.h"

#ifdef __cplusplus
extern "C" {
#endif

#define VIBE_Q_I2C_PORT I2C_NUM_1
#define VIBE_Q_SDA_PIN 13
#define VIBE_Q_SCL_PIN 14
#define VIBE_Q_I2C_FREQ_HZ 100000
#define VIBE_Q_MPU6050_ADDR 0x68

typedef enum {
    VIBE_Q_STATUS_STARTING = 0,
    VIBE_Q_STATUS_OK,
    VIBE_Q_STATUS_I2C_COMM_FAULT,
} vibe_q_status_t;

esp_err_t vibe_q_init(void);
esp_err_t vibe_q_deinit(void);
esp_err_t vibe_q_read_all(wtvb01_data_t *data);
esp_err_t vibe_q_get_stats(wtvb01_stats_t *stats, bool reset);
vibe_q_status_t vibe_q_get_status(void);
const char *vibe_q_status_name(vibe_q_status_t status);

#ifdef __cplusplus
}
#endif


