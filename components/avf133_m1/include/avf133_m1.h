#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AVF133_UART_NUM 1
#define AVF133_TX_PIN 17
#define AVF133_RX_PIN 18
#define AVF133_RTS_PIN 19
#define AVF133_SLAVE_ADDR 2
#define AVF133_BAUD_RATE 9600

#define AVH14_SLAVE_ADDR 1
#define AVH14_BAUD_RATE 9600

typedef enum {
  ENERGY_METER_MODEL_AVF_133_M1 = 0,
  ENERGY_METER_MODEL_AVH_14_M1 = 1,
} energy_meter_model_t;

typedef struct {
  float vrn;
  float vyn;
  float vbn;
  float vln_avg;
  float vry;
  float vyb;
  float vbr;
  float vll_avg;
  float ir;
  float iy;
  float ib;
  float i_avg;
  float total_kw;
  float total_kva;
  float power_factor;
  bool measured_power_valid;
  energy_meter_model_t model;
  bool data_valid;
  uint32_t read_count;
  uint32_t error_count;
  uint32_t last_update_ms;
} avf133_data_t;

esp_err_t avf133_init(void);
esp_err_t avf133_deinit(void);
bool avf133_is_running(void);
esp_err_t avf133_read_latest(avf133_data_t *out);
esp_err_t avf133_set_meter_model(energy_meter_model_t model);
energy_meter_model_t avf133_get_meter_model(void);
const char *avf133_meter_model_name(energy_meter_model_t model);
uint8_t avf133_meter_slave_address(energy_meter_model_t model);

#ifdef __cplusplus
}
#endif
