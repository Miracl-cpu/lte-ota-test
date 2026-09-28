/**
 * @file wtvb01_modbus.h
 * @brief Internal Modbus RTU protocol layer for RS-WZ3 / WTVB01-485 sensors
 *
 * Internal header - not part of the public API.
 * Handles CRC-16, frame construction, and response parsing.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* --------------------------------------------------------------------------
 * Modbus communication parameters
 * -------------------------------------------------------------------------- */

/** Sensor Modbus slave addresses */
#define RS_WZ3_MODBUS_ADDR          0x01
#define WTVB01_485_MODBUS_ADDR      0x50
#define WTVB01_MODBUS_ADDR          RS_WZ3_MODBUS_ADDR

/** Function code: Read Holding Registers */
#define MODBUS_FC_READ_REGS         0x03

/** Maximum response buffer size (for 19 registers: 5 + 38 = 43 bytes) */
#define MODBUS_MAX_RESPONSE_LEN     48

/** Request frame length (always 8 bytes for FC 0x03) */
#define MODBUS_REQUEST_LEN          8

/* --------------------------------------------------------------------------
 * Register blocks
 * -------------------------------------------------------------------------- */

/* WTVB01-485 firmware reference: 0x0034 to 0x0046 inclusive = 19 regs */
#define WTVB01_485_REG_DATA_START   0x0034
#define WTVB01_485_REG_DATA_COUNT   19

/* RS-WZ3WZ1-N01-1 firmware reference: 0x0000 to 0x000C inclusive = 13 regs */
#define RS_WZ3_REG_DATA_START       0x0000
#define RS_WZ3_REG_DATA_COUNT       13

#define VIB_MAX_REG_DATA_COUNT      WTVB01_485_REG_DATA_COUNT

/* --------------------------------------------------------------------------
 * Error codes
 * -------------------------------------------------------------------------- */

typedef enum {
    MODBUS_OK            = 0,  /**< Success */
    MODBUS_ERR_TIMEOUT   = 1,  /**< No/incomplete response */
    MODBUS_ERR_CRC       = 2,  /**< CRC mismatch */
    MODBUS_ERR_LENGTH    = 3,  /**< Unexpected frame length */
    MODBUS_ERR_ADDRESS   = 4,  /**< Wrong slave address in response */
    MODBUS_ERR_FUNCTION  = 5,  /**< Wrong function code in response */
    MODBUS_ERR_EXCEPTION = 6,  /**< Sensor returned exception code */
} modbus_err_t;

uint16_t modbus_crc16(const uint8_t *data, size_t length);
void modbus_add_crc(uint8_t *frame, size_t length);
bool modbus_check_crc(const uint8_t *frame, size_t length);
void modbus_build_read_cmd_addr(uint8_t addr, uint8_t *cmd,
                                uint16_t start_reg, uint16_t num_regs);
void modbus_build_read_cmd(uint8_t *cmd, uint16_t start_reg, uint16_t num_regs);
int modbus_parse_response_addr(uint8_t expected_addr, const uint8_t *response,
                               size_t length, uint16_t *regs, int max_regs);
int modbus_parse_response(const uint8_t *response, size_t length,
                          uint16_t *regs, int max_regs);
uint32_t modbus_parse_uint32(const uint16_t *regs, int offset);
int32_t modbus_parse_int32(const uint16_t *regs, int offset);

static inline int modbus_response_len(int num_regs)
{
    return 5 + num_regs * 2;
}
