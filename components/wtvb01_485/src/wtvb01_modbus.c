/**
 * @file wtvb01_modbus.c
 * @brief Modbus RTU protocol implementation for RS-WZ3 / WTVB01-485 sensors
 *
 * Covers:
 *   - CRC-16 calculation (polynomial 0xA001)
 *   - Request frame construction (FC 0x03)
 *   - Response frame validation and register extraction
 */

#include "wtvb01_modbus.h"

#include <string.h>
#include "esp_log.h"

static const char *TAG = "VIB_MODBUS";

/* ============================================================================
 * CRC-16 (Modbus)
 * ============================================================================ */

uint16_t modbus_crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i];

        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}

void modbus_add_crc(uint8_t *frame, size_t length)
{
    uint16_t crc = modbus_crc16(frame, length);
    frame[length]     = (uint8_t)(crc & 0xFF);         /* CRC low byte first */
    frame[length + 1] = (uint8_t)((crc >> 8) & 0xFF);  /* CRC high byte second */
}

bool modbus_check_crc(const uint8_t *frame, size_t length)
{
    if (length < 4) {
        return false;
    }

    uint16_t received_crc   = (uint16_t)frame[length - 2] |
                              ((uint16_t)frame[length - 1] << 8);
    uint16_t calculated_crc = modbus_crc16(frame, length - 2);

    return (received_crc == calculated_crc);
}

/* ============================================================================
 * Frame construction
 * ============================================================================ */

void modbus_build_read_cmd_addr(uint8_t addr, uint8_t *cmd,
                                uint16_t start_reg, uint16_t num_regs)
{
    cmd[0] = addr;                        /* Slave address */
    cmd[1] = MODBUS_FC_READ_REGS;         /* Function code 0x03 */
    cmd[2] = (start_reg >> 8) & 0xFF;     /* Start address high byte */
    cmd[3] = start_reg & 0xFF;            /* Start address low byte */
    cmd[4] = (num_regs >> 8) & 0xFF;      /* Register count high byte */
    cmd[5] = num_regs & 0xFF;             /* Register count low byte */
    modbus_add_crc(cmd, 6);               /* Append 2-byte CRC */
}

void modbus_build_read_cmd(uint8_t *cmd, uint16_t start_reg, uint16_t num_regs)
{
    modbus_build_read_cmd_addr(WTVB01_MODBUS_ADDR, cmd, start_reg, num_regs);
}

/* ============================================================================
 * Response parsing
 * ============================================================================ */

int modbus_parse_response_addr(uint8_t expected_addr, const uint8_t *response,
                               size_t length, uint16_t *regs, int max_regs)
{
    /* Absolute minimum: addr(1) + func(1) + byte_count(1) + crc(2) = 5 bytes */
    if (length < 5) {
        ESP_LOGW(TAG, "Response too short: %d bytes", (int)length);
        return -MODBUS_ERR_LENGTH;
    }

    /* Validate CRC before processing content */
    if (!modbus_check_crc(response, length)) {
        uint16_t received_crc   = (uint16_t)response[length - 2] |
                                  ((uint16_t)response[length - 1] << 8);
        uint16_t calculated_crc = modbus_crc16(response, length - 2);
        ESP_LOGW(TAG, "CRC mismatch: received=0x%04X calculated=0x%04X",
                 received_crc, calculated_crc);
        return -MODBUS_ERR_CRC;
    }

    /* Validate slave address */
    if (response[0] != expected_addr) {
        ESP_LOGW(TAG, "Wrong address: expected 0x%02X, got 0x%02X",
                 expected_addr, response[0]);
        return -MODBUS_ERR_ADDRESS;
    }

    /* Check for exception response (function code with error flag 0x80) */
    if (response[1] == (MODBUS_FC_READ_REGS | 0x80)) {
        ESP_LOGW(TAG, "Sensor exception code: 0x%02X", response[2]);
        return -MODBUS_ERR_EXCEPTION;
    }

    /* Validate function code */
    if (response[1] != MODBUS_FC_READ_REGS) {
        ESP_LOGW(TAG, "Wrong function code: expected 0x%02X, got 0x%02X",
                 MODBUS_FC_READ_REGS, response[1]);
        return -MODBUS_ERR_FUNCTION;
    }

    /* Validate byte count vs actual frame length */
    uint8_t byte_count = response[2];
    int num_regs       = byte_count / 2;

    if (length != (size_t)(5 + byte_count)) {
        ESP_LOGW(TAG, "Length mismatch: byte_count=%d, expected frame=%d, got=%d",
                 byte_count, 5 + byte_count, (int)length);
        return -MODBUS_ERR_LENGTH;
    }

    if (num_regs > max_regs) {
        ESP_LOGW(TAG, "More registers than buffer: %d > %d", num_regs, max_regs);
        return -MODBUS_ERR_LENGTH;
    }

    /* Extract register values (big-endian: high byte first) */
    for (int i = 0; i < num_regs; i++) {
        int offset = 3 + (i * 2);
        regs[i] = ((uint16_t)response[offset] << 8) |
                  ((uint16_t)response[offset + 1]);
    }

    return num_regs;
}

int modbus_parse_response(const uint8_t *response, size_t length,
                          uint16_t *regs, int max_regs)
{
    return modbus_parse_response_addr(WTVB01_MODBUS_ADDR, response, length,
                                      regs, max_regs);
}

/* ============================================================================
 * Register value helpers
 * ============================================================================ */

uint32_t modbus_parse_uint32(const uint16_t *regs, int offset)
{
    uint32_t high = (uint32_t)regs[offset];
    uint32_t low  = (uint32_t)regs[offset + 1];
    return (high << 16) | low;
}

int32_t modbus_parse_int32(const uint16_t *regs, int offset)
{
    return (int32_t)modbus_parse_uint32(regs, offset);
}
