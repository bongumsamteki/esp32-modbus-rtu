#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "driver/uart.h"
#include "driver/gpio.h"

/* ESP32 CAM defines */
#define UART_NUM   UART_NUM_1

/* end of ESP32CAM defines */

/* memory locations defines here */

#define COILS_BASE 1
#define INPUTS_BASE 10001
#define RESERVED_BASE 20001
#define INPUT_REGISTERS_BASE 30001
#define HOLDING_REGISTER_BASE 40001
#define LAST_REGISTER_LOCATION 50000

/* end of memory location defines */

/* function codes here */

#define READ_COILS 0x01
#define READ_DISCRETE_INPUTS 0x02
#define READ_HOLDING_REGISTERS 0x03
#define READ_INPUT_REGISTERS 0x04
#define WRITE_SINGLE_COIL 0x05
#define WRITE_SINGLE_REGISTER 0x06
#define WRITE_MULTIPLE_COILS 0x0F
#define WRITE_MULTIPLE_REGISTERS 0x10
#define READ_WRITE_MULTIPLE_REGISTERS 0x17

/* end of function codes */

/* modbus rtu error codes */

#define NO_ERROR              0x00
#define ILLEGAL_FUNCTION      0x01
#define ILLEGAL_ADDRESS       0x02
#define ILLEGAL_DATA_SIZE     0x21
#define CRC_ERROR             0x96
#define UNKNOWN_ERROR         0x55

#define READ_COILS_ERR_CODE        0x81
#define READ_DINPUTS_ERR_CODE      0x82
#define READ_HOLDING_ERR_CODE      0x83
#define READ_INPUTS_ERR_CODE       0x84
#define WRITE_COIL_ERR_CODE        0x85
#define WRITE_SINGLE_REG_ERR_CODE  0x86
#define RW_MULTIPLE_ERR_CODE       0x97


/* end of modbus rtu error codes */

/* master structure */

typedef struct {
    uint8_t slave_id; // 1 byte
    uint8_t function_code; // 1 byte
    uint16_t start_address; // 2 bytes
    uint16_t write_start_address; // 2 bytes
    uint16_t quantity; // 2 bytes
    uint16_t write_quantity; // 2 bytes
    uint16_t crc; // 2 bytes
} modbus_rtu_query_frame;

/* slave structure */
typedef struct {
    uint8_t slave_id; // 1 byte
    uint8_t byte_count; // 1 byte
    uint8_t *slave_data; // byte_count*8 byte
    uint16_t crc; // 2 byte
} modbus_rtu_response_frame;

typedef unsigned int error_mod;

/* modbus rtu uart init function */
void modbus_uart_init(void);

/* modbus rtu read function */
uint8_t modbus_rtu_read(modbus_rtu_query_frame *query_ptr, uint8_t *query_response);
uint8_t modbus_rtu_write(modbus_rtu_query_frame *query_ptr, uint8_t *query_value);
uint8_t *modbus_rtu_crc(uint8_t *modbus_rtu_frame, size_t frame_size, uint8_t *crc_buffer);
uint8_t modbus_rtu_read_write_multi(modbus_rtu_query_frame *query_ptr, uint8_t *query_buffer, uint8_t *response_buffer);