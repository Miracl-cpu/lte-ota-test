#pragma once

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RS485_BUS_UART_NUM 1
#define RS485_BUS_TX_PIN 17
#define RS485_BUS_RX_PIN 18
#define RS485_BUS_RTS_PIN 19

esp_err_t rs485_bus_init(int baud_rate);
esp_err_t rs485_bus_deinit(void);
esp_err_t rs485_bus_set_baudrate(int baud_rate);
esp_err_t rs485_bus_transact(int baud_rate, const uint8_t *cmd, int cmd_len,
                             uint8_t *resp, int resp_len,
                             uint32_t timeout_ms, uint32_t pre_tx_delay_us,
                             uint32_t post_tx_delay_us,
                             uint32_t inter_frame_delay_us,
                             int *received_out);

#ifdef __cplusplus
}
#endif
