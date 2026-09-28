#include "avf133_m1.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "log_config.h"
#include "rs485_bus.h"

static const char *TAG = "AVF133";

#define AVF133_FUNC_READ_HOLDING 0x03
#define AVF133_START_REG 0
#define AVF133_REG_COUNT 30
#define AVH14_START_REG 12
#define AVH14_REG_COUNT 90
#define AVF133_REQ_LEN 8
#define AVF133_MAX_REG_COUNT AVH14_REG_COUNT
#define AVF133_MAX_RESP_LEN (5 + (AVF133_MAX_REG_COUNT * 2))
#define AVF133_POLL_MS 1000
#define AVF133_TIMEOUT_MS 500

static SemaphoreHandle_t s_mutex = NULL;
static TaskHandle_t s_task = NULL;
static avf133_data_t s_data = {0};
static volatile bool s_running = false;
static energy_meter_model_t s_model = ENERGY_METER_MODEL_AVF_133_M1;

static uint8_t model_slave_address(energy_meter_model_t model) {
  return model == ENERGY_METER_MODEL_AVH_14_M1 ? AVH14_SLAVE_ADDR
                                               : AVF133_SLAVE_ADDR;
}

static uint16_t model_start_register(energy_meter_model_t model) {
  return model == ENERGY_METER_MODEL_AVH_14_M1 ? AVH14_START_REG
                                               : AVF133_START_REG;
}

static uint16_t model_register_count(energy_meter_model_t model) {
  return model == ENERGY_METER_MODEL_AVH_14_M1 ? AVH14_REG_COUNT
                                               : AVF133_REG_COUNT;
}

static uint16_t modbus_crc16(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; bit++) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

static void modbus_add_crc(uint8_t *frame, size_t length) {
  uint16_t crc = modbus_crc16(frame, length);
  frame[length] = (uint8_t)(crc & 0xFF);
  frame[length + 1] = (uint8_t)((crc >> 8) & 0xFF);
}

static bool modbus_crc_ok(const uint8_t *frame, size_t length) {
  if (length < 4) {
    return false;
  }
  uint16_t got = (uint16_t)frame[length - 2] |
                 ((uint16_t)frame[length - 1] << 8);
  return got == modbus_crc16(frame, length - 2);
}

static void build_read_cmd(uint8_t *cmd, energy_meter_model_t model) {
  uint16_t start_reg = model_start_register(model);
  uint16_t reg_count = model_register_count(model);
  cmd[0] = model_slave_address(model);
  cmd[1] = AVF133_FUNC_READ_HOLDING;
  cmd[2] = (uint8_t)((start_reg >> 8) & 0xFF);
  cmd[3] = (uint8_t)(start_reg & 0xFF);
  cmd[4] = (uint8_t)((reg_count >> 8) & 0xFF);
  cmd[5] = (uint8_t)(reg_count & 0xFF);
  modbus_add_crc(cmd, 6);
}

static float regs_float_abcd(const uint16_t *regs, int offset) {
  uint32_t raw = ((uint32_t)regs[offset + 1] << 16) | regs[offset];
  float value;
  memcpy(&value, &raw, sizeof(value));
  return value;
}

static bool sane_float(float value, float min_value, float max_value) {
  return isfinite(value) && value >= min_value && value <= max_value;
}

static esp_err_t parse_response(const uint8_t *resp, int len,
                                energy_meter_model_t model,
                                avf133_data_t *out) {
  uint16_t reg_count = model_register_count(model);
  int expected_len = 5 + (reg_count * 2);
  uint8_t slave_addr = model_slave_address(model);
  if (len != expected_len) {
    ESP_LOGW(TAG, "Bad response length: got %d expected %d", len,
             expected_len);
    return ESP_FAIL;
  }
  if (!modbus_crc_ok(resp, len)) {
    ESP_LOGW(TAG, "CRC mismatch");
    return ESP_FAIL;
  }
  if (resp[0] != slave_addr || resp[1] != AVF133_FUNC_READ_HOLDING ||
      resp[2] != (reg_count * 2)) {
    ESP_LOGW(TAG, "Unexpected header addr=0x%02X fn=0x%02X bytes=%u",
             resp[0], resp[1], resp[2]);
    return ESP_FAIL;
  }

  uint16_t regs[AVF133_MAX_REG_COUNT];
  for (int i = 0; i < reg_count; i++) {
    int off = 3 + (i * 2);
    regs[i] = ((uint16_t)resp[off] << 8) | resp[off + 1];
  }

  memset(out, 0, sizeof(*out));
  out->model = model;
  if (model == ENERGY_METER_MODEL_AVH_14_M1) {
    /* AVH-14-M1 register map, relative to holding register 12. */
    out->vrn = regs_float_abcd(regs, 0);   /* 12 */
    out->vyn = regs_float_abcd(regs, 4);   /* 16 */
    out->vbn = regs_float_abcd(regs, 8);   /* 20 */
    out->vln_avg = regs_float_abcd(regs, 12); /* 24 */
    out->vry = regs_float_abcd(regs, 16);  /* 28 */
    out->vyb = regs_float_abcd(regs, 20);  /* 32 */
    out->vbr = regs_float_abcd(regs, 24);  /* 36 */
    out->vll_avg = regs_float_abcd(regs, 28); /* 40 */
    out->ir = regs_float_abcd(regs, 32);   /* 44 */
    out->iy = regs_float_abcd(regs, 36);   /* 48 */
    out->ib = regs_float_abcd(regs, 40);   /* 52 */
    out->i_avg = regs_float_abcd(regs, 44); /* 56 */
    out->power_factor = regs_float_abcd(regs, 56); /* 68 */
    out->total_kw = regs_float_abcd(regs, 72);     /* 84 */
    out->total_kva = regs_float_abcd(regs, 88);    /* 100 */
    out->measured_power_valid =
        sane_float(out->total_kw, -1000000.0f, 1000000.0f) &&
        sane_float(out->total_kva, 0.0f, 1000000.0f);
  } else {
    out->vrn = regs_float_abcd(regs, 0);
    out->vyn = regs_float_abcd(regs, 2);
    out->vbn = regs_float_abcd(regs, 4);
    out->vln_avg = regs_float_abcd(regs, 6);
    out->vry = regs_float_abcd(regs, 8);
    out->vyb = regs_float_abcd(regs, 10);
    out->vbr = regs_float_abcd(regs, 12);
    out->vll_avg = regs_float_abcd(regs, 14);
    out->ir = regs_float_abcd(regs, 16);
    out->iy = regs_float_abcd(regs, 20);
    out->ib = regs_float_abcd(regs, 24);
    out->i_avg = regs_float_abcd(regs, 28);
  }

  if (!sane_float(out->vrn, 0.0f, 1000.0f) ||
      !sane_float(out->vll_avg, 0.0f, 1000.0f) ||
      !sane_float(out->i_avg, 0.0f, 20000.0f)) {
    ESP_LOGW(TAG, "Parsed values out of range: VRN=%.2f VLLavg=%.2f Iavg=%.2f",
             out->vrn, out->vll_avg, out->i_avg);
    return ESP_FAIL;
  }
  return ESP_OK;
}

static esp_err_t read_once(avf133_data_t *out) {
  uint8_t cmd[AVF133_REQ_LEN];
  uint8_t resp[AVF133_MAX_RESP_LEN];
  energy_meter_model_t model = s_model;
  int expected_len = 5 + (model_register_count(model) * 2);
  build_read_cmd(cmd, model);

  int got = 0;
  esp_err_t ret = rs485_bus_transact(
      AVF133_BAUD_RATE, cmd, AVF133_REQ_LEN, resp, expected_len,
      AVF133_TIMEOUT_MS, 200, 200, 5000, &got);
  if (ret != ESP_OK) {
    return ret;
  }
  return parse_response(resp, got, model, out);
}

static void avf133_task(void *arg) {
  (void)arg;
  ESP_LOGI(TAG, "Task started: model=%s slave=%u baud=%d 8N1",
           avf133_meter_model_name(s_model), model_slave_address(s_model),
           AVF133_BAUD_RATE);

  while (s_running) {
    avf133_data_t local = {0};
    esp_err_t ret = read_once(&local);
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      if (ret == ESP_OK) {
        local.data_valid = true;
        local.read_count = s_data.read_count + 1;
        local.error_count = s_data.error_count;
        local.last_update_ms = now_ms;
        s_data = local;
      } else {
        s_data.error_count++;
      }
      xSemaphoreGive(s_mutex);
    }

    vTaskDelay(pdMS_TO_TICKS(AVF133_POLL_MS));
  }

  s_task = NULL;
  vTaskDelete(NULL);
}

esp_err_t avf133_init(void) {
  if (s_running) {
    return ESP_OK;
  }

  esp_err_t ret = rs485_bus_init(AVF133_BAUD_RATE);
  if (ret != ESP_OK) {
    return ret;
  }

  if (s_mutex == NULL) {
    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) {
      rs485_bus_deinit();
      return ESP_ERR_NO_MEM;
    }
  }

  memset(&s_data, 0, sizeof(s_data));
  s_running = true;
  BaseType_t ok = xTaskCreatePinnedToCore(avf133_task, "avf133_task", 4096,
                                          NULL, tskIDLE_PRIORITY + 2,
                                          &s_task, 1);
  if (ok != pdPASS) {
    s_running = false;
    rs485_bus_deinit();
    return ESP_ERR_NO_MEM;
  }

  ESP_LOGI(TAG, "%s ready on UART%d TX=%d RX=%d RTS=%d slave=%d baud=%d",
           avf133_meter_model_name(s_model),
           AVF133_UART_NUM, AVF133_TX_PIN, AVF133_RX_PIN, AVF133_RTS_PIN,
           model_slave_address(s_model), AVF133_BAUD_RATE);
  return ESP_OK;
}

esp_err_t avf133_deinit(void) {
  if (!s_running) {
    return ESP_OK;
  }
  s_running = false;
  /* A poll can be inside its UART timeout followed by its normal delay.  Wait
   * long enough for that iteration to finish before releasing the shared bus
   * or allowing the caller to select another meter model. */
  for (int i = 0; i < 50 && s_task != NULL; i++) {
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  if (s_task != NULL) {
    ESP_LOGE(TAG, "Timed out waiting for %s task to stop",
             avf133_meter_model_name(s_model));
    return ESP_ERR_TIMEOUT;
  }
  rs485_bus_deinit();
  return ESP_OK;
}

bool avf133_is_running(void) { return s_running; }

esp_err_t avf133_read_latest(avf133_data_t *out) {
  if (out == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!s_running || s_mutex == NULL) {
    return ESP_ERR_INVALID_STATE;
  }
  if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(20)) != pdTRUE) {
    return ESP_ERR_TIMEOUT;
  }
  *out = s_data;
  xSemaphoreGive(s_mutex);
  return out->data_valid ? ESP_OK : ESP_ERR_INVALID_STATE;
}

esp_err_t avf133_set_meter_model(energy_meter_model_t model) {
  if (model != ENERGY_METER_MODEL_AVF_133_M1 &&
      model != ENERGY_METER_MODEL_AVH_14_M1) {
    return ESP_ERR_INVALID_ARG;
  }
  if (s_running) {
    return ESP_ERR_INVALID_STATE;
  }
  s_model = model;
  memset(&s_data, 0, sizeof(s_data));
  return ESP_OK;
}

energy_meter_model_t avf133_get_meter_model(void) { return s_model; }

const char *avf133_meter_model_name(energy_meter_model_t model) {
  return model == ENERGY_METER_MODEL_AVH_14_M1 ? "AVH-14-M1"
                                               : "AVF-133-M1";
}

uint8_t avf133_meter_slave_address(energy_meter_model_t model) {
  return model_slave_address(model);
}
