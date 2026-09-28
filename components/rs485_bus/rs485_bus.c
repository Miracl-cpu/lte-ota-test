#include "rs485_bus.h"

#include "driver/uart.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <stdbool.h>

static SemaphoreHandle_t s_bus_mutex = NULL;
static int s_ref_count = 0;
static int s_current_baud = 0;
static bool s_installed = false;

#define RS485_BUS_RX_BUF_SIZE 512

static esp_err_t rs485_bus_apply_baud_locked(int baud_rate) {
  if (baud_rate <= 0) {
    return ESP_ERR_INVALID_ARG;
  }
  if (s_current_baud == baud_rate) {
    return ESP_OK;
  }
  esp_err_t ret =
      uart_set_baudrate((uart_port_t)RS485_BUS_UART_NUM, baud_rate);
  if (ret == ESP_OK) {
    s_current_baud = baud_rate;
  }
  return ret;
}

esp_err_t rs485_bus_init(int baud_rate) {
  if (baud_rate <= 0) {
    return ESP_ERR_INVALID_ARG;
  }
  if (s_bus_mutex == NULL) {
    s_bus_mutex = xSemaphoreCreateMutex();
    if (s_bus_mutex == NULL) {
      return ESP_ERR_NO_MEM;
    }
  }

  if (xSemaphoreTake(s_bus_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    return ESP_ERR_TIMEOUT;
  }

  esp_err_t ret = ESP_OK;
  if (!s_installed) {
    uart_config_t cfg = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ret = uart_param_config((uart_port_t)RS485_BUS_UART_NUM, &cfg);
    if (ret == ESP_OK) {
      ret = uart_set_pin((uart_port_t)RS485_BUS_UART_NUM, RS485_BUS_TX_PIN,
                         RS485_BUS_RX_PIN, RS485_BUS_RTS_PIN,
                         UART_PIN_NO_CHANGE);
    }
    if (ret == ESP_OK) {
      ret = uart_driver_install((uart_port_t)RS485_BUS_UART_NUM,
                                RS485_BUS_RX_BUF_SIZE, 0, 0, NULL, 0);
    }
    if (ret == ESP_OK) {
      ret = uart_set_mode((uart_port_t)RS485_BUS_UART_NUM,
                          UART_MODE_RS485_HALF_DUPLEX);
    }
    if (ret == ESP_OK) {
      s_installed = true;
      s_current_baud = baud_rate;
    } else {
      uart_driver_delete((uart_port_t)RS485_BUS_UART_NUM);
      s_current_baud = 0;
    }
  } else {
    ret = rs485_bus_apply_baud_locked(baud_rate);
  }

  if (ret == ESP_OK) {
    s_ref_count++;
  }
  xSemaphoreGive(s_bus_mutex);
  return ret;
}

esp_err_t rs485_bus_deinit(void) {
  if (s_bus_mutex == NULL) {
    return ESP_OK;
  }
  if (xSemaphoreTake(s_bus_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    return ESP_ERR_TIMEOUT;
  }

  if (s_ref_count > 0) {
    s_ref_count--;
  }
  if (s_ref_count == 0 && s_installed) {
    uart_driver_delete((uart_port_t)RS485_BUS_UART_NUM);
    s_installed = false;
    s_current_baud = 0;
  }

  xSemaphoreGive(s_bus_mutex);
  return ESP_OK;
}

esp_err_t rs485_bus_set_baudrate(int baud_rate) {
  if (s_bus_mutex == NULL || !s_installed) {
    return ESP_ERR_INVALID_STATE;
  }
  if (xSemaphoreTake(s_bus_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    return ESP_ERR_TIMEOUT;
  }
  esp_err_t ret = rs485_bus_apply_baud_locked(baud_rate);
  if (ret == ESP_OK) {
    uart_flush_input((uart_port_t)RS485_BUS_UART_NUM);
  }
  xSemaphoreGive(s_bus_mutex);
  return ret;
}

esp_err_t rs485_bus_transact(int baud_rate, const uint8_t *cmd, int cmd_len,
                             uint8_t *resp, int resp_len,
                             uint32_t timeout_ms, uint32_t pre_tx_delay_us,
                             uint32_t post_tx_delay_us,
                             uint32_t inter_frame_delay_us,
                             int *received_out) {
  if (received_out != NULL) {
    *received_out = 0;
  }
  if (!cmd || cmd_len <= 0 || !resp || resp_len <= 0) {
    return ESP_ERR_INVALID_ARG;
  }
  if (s_bus_mutex == NULL || !s_installed) {
    return ESP_ERR_INVALID_STATE;
  }
  if (xSemaphoreTake(s_bus_mutex, pdMS_TO_TICKS(timeout_ms + 1000)) != pdTRUE) {
    return ESP_ERR_TIMEOUT;
  }

  esp_err_t ret = rs485_bus_apply_baud_locked(baud_rate);
  int received = 0;
  if (ret == ESP_OK) {
    uart_flush_input((uart_port_t)RS485_BUS_UART_NUM);
    esp_rom_delay_us(pre_tx_delay_us);

    int written = uart_write_bytes((uart_port_t)RS485_BUS_UART_NUM,
                                   (const char *)cmd, cmd_len);
    if (written != cmd_len) {
      ret = ESP_FAIL;
    }
  }
  if (ret == ESP_OK) {
    uart_wait_tx_done((uart_port_t)RS485_BUS_UART_NUM, pdMS_TO_TICKS(25));
    esp_rom_delay_us(post_tx_delay_us);
    received = uart_read_bytes((uart_port_t)RS485_BUS_UART_NUM, resp, resp_len,
                               pdMS_TO_TICKS(timeout_ms));
    if (inter_frame_delay_us > 0) {
      vTaskDelay(pdMS_TO_TICKS((inter_frame_delay_us + 999U) / 1000U));
    }
  }

  if (received_out != NULL) {
    *received_out = received;
  }
  xSemaphoreGive(s_bus_mutex);
  return ret;
}
