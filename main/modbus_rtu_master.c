#include "modbus_rtu_master.h"

void modbus_uart_init(void)
{ 
    const uart_port_t uart_num = UART_NUM_1;
    
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    uart_param_config(uart_num, &uart_config);
    uart_set_pin(uart_num, 1, 3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    const int uart_buffer_size = (1024 * 2);
    QueueHandle_t uart_queue;

    uart_driver_install(uart_num, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0);
}

uint8_t modbus_rtu_read(modbus_rtu_query_frame *query_ptr, uint8_t *query_response)
{
    if(((query_ptr -> function_code) == READ_COILS) || ((query_ptr -> function_code) == READ_DISCRETE_INPUTS) || ((query_ptr -> function_code) == READ_HOLDING_REGISTERS) || (((query_ptr -> function_code) == READ_INPUT_REGISTERS)))
    {
        if((((query_ptr -> start_address) >= COILS_BASE) && ((query_ptr -> start_address) < RESERVED_BASE)) || (((query_ptr -> start_address) >= INPUT_REGISTERS_BASE) && ((query_ptr -> start_address) < LAST_REGISTER_LOCATION)))
        {   
            uint8_t modbus_rtu_frame_query[8];
            size_t length = 0;
            error_mod error_code = NO_ERROR;

            modbus_rtu_frame_query[0] = query_ptr -> slave_id; // slave id - 1 byte
            modbus_rtu_frame_query[1] = query_ptr -> function_code; // function code - 1 byte
            modbus_rtu_frame_query[2] = (((query_ptr -> start_address) >> 8) & 0xff); // start address high - 1 byte
            modbus_rtu_frame_query[3] = ((query_ptr -> start_address) & 0xff); // start address low - 1 byte
            modbus_rtu_frame_query[4] = (((query_ptr -> quantity) >> 8) & 0xff); // quatity coils high - 1 byte
            modbus_rtu_frame_query[5] = ((query_ptr -> quantity) & 0xff); // quantity coils low - 1 byte
    
            modbus_rtu_crc(modbus_rtu_frame_query, sizeof(modbus_rtu_frame_query), &modbus_rtu_frame_query[6]); // calculate crc code

            uart_write_bytes(UART_NUM, modbus_rtu_frame_query, sizeof(modbus_rtu_frame_query));
            uart_wait_tx_done(UART_NUM, 500);

            while(!(length > 0))
            {
                uart_get_buffered_data_len(UART_NUM, &length);
            }


            uint16_t response_frame_size = 0;

            // compute response frame size

            if(((query_ptr -> function_code) == READ_HOLDING_REGISTERS) || ((query_ptr -> function_code) == READ_INPUT_REGISTERS))
            {
                response_frame_size = 3 + 2*(query_ptr -> quantity) + 2; // 3 + 2*N + CRC
            }
            else if(((query_ptr -> function_code) == READ_COILS) || ((query_ptr -> function_code) == READ_DISCRETE_INPUTS))
            {
                response_frame_size = 3 + ceil((float)(query_ptr -> quantity)/8.0) + 2; // 3 + N/8 + CRC
            }
            else if(((query_ptr -> function_code) == READ_COILS_ERR_CODE) || ((query_ptr -> function_code) ==  READ_DINPUTS_ERR_CODE) || ((query_ptr -> function_code) ==  READ_HOLDING_ERR_CODE) || ((query_ptr -> function_code) == READ_INPUTS_ERR_CODE))
            {
                response_frame_size = 5;
            }

            uint8_t response_frame[response_frame_size];

            // read response
            uart_read_bytes(UART_NUM, &response_frame, response_frame_size, 500);
            uart_flush(UART_NUM); // flush uart for the next transmission

            // verify response

            error_code = (response_frame[0] != (query_ptr -> slave_id)) ? ILLEGAL_ADDRESS : NO_ERROR;

            if((((response_frame[1]) == READ_COILS_ERR_CODE) || ((response_frame[1]) == READ_DINPUTS_ERR_CODE) || ((response_frame[1]) == READ_HOLDING_ERR_CODE) || ((response_frame[1]) == READ_INPUTS_ERR_CODE)) && (error_code == NO_ERROR)) // handle exceptions
            {
                switch(response_frame[2]) // read exception code
                {
                    case ILLEGAL_ADDRESS:
                        return ILLEGAL_ADDRESS;
                        break;
                    case ILLEGAL_FUNCTION:
                        return ILLEGAL_FUNCTION;
                        break;
                    case ILLEGAL_DATA_SIZE:
                        return ILLEGAL_DATA_SIZE;
                        break;
                    case CRC_ERROR:
                        return CRC_ERROR;
                        break;
                    default:
                        return UNKNOWN_ERROR;
                }
            }
            else
            {
                error_code = ((response_frame[1] != (query_ptr -> function_code)) && (error_code == NO_ERROR)) ? ILLEGAL_FUNCTION : ((error_code == NO_ERROR) ? NO_ERROR : error_code);
                
                if(((query_ptr -> function_code) == READ_HOLDING_REGISTERS) || ((query_ptr -> function_code) == READ_INPUT_REGISTERS))
                {
                    error_code = ((response_frame[2]/2 != (query_ptr -> quantity)) && (error_code == NO_ERROR)) ?  ILLEGAL_DATA_SIZE : ((error_code == NO_ERROR) ? NO_ERROR : error_code);
                }
                else if(((query_ptr -> function_code) == READ_COILS) || ((query_ptr -> function_code) == READ_DISCRETE_INPUTS))
                {
                    error_code = ((response_frame[2] != (int)ceil((query_ptr -> quantity)/8.0)) && (error_code == NO_ERROR)) ?  ILLEGAL_DATA_SIZE : ((error_code == NO_ERROR) ? NO_ERROR : error_code);
                }

                uint8_t crc_buffer[2];
                modbus_rtu_crc(response_frame, sizeof(response_frame), crc_buffer);

                error_code = ((((*(&response_frame[2] + (int)response_frame[2] + 2) << 8) | (*(&response_frame[2] + response_frame[2] + 1))) != ((crc_buffer[1] << 8) | (crc_buffer[0]))) && (error_code == NO_ERROR)) ? CRC_ERROR : ((error_code == NO_ERROR) ? NO_ERROR : error_code);

                switch(error_code)
                {
                    case ILLEGAL_ADDRESS:
                        return ILLEGAL_ADDRESS;
                        break;
                    case ILLEGAL_FUNCTION:
                        return ILLEGAL_FUNCTION;
                        break;
                    case ILLEGAL_DATA_SIZE:
                        return ILLEGAL_DATA_SIZE;
                        break;
                    case CRC_ERROR:
                        return CRC_ERROR;
                        break;
                    default:
                        strncpy((char *)query_response, (char *)response_frame, response_frame_size);
                        return NO_ERROR;
                }
            }
        }
        else
        {
            return ILLEGAL_ADDRESS;
        }
    }
    else
    {
        // error
        return ILLEGAL_FUNCTION;
    }
}

uint8_t modbus_rtu_read_write_multi(modbus_rtu_query_frame *query_ptr, uint8_t *query_buffer, uint8_t *response_buffer)
{
    if((query_ptr -> function_code) == READ_WRITE_MULTIPLE_REGISTERS)
    {
        if (((((query_ptr -> start_address) >= COILS_BASE) && ((query_ptr -> start_address) < RESERVED_BASE)) || (((query_ptr -> start_address) >= INPUT_REGISTERS_BASE) && ((query_ptr -> start_address) < LAST_REGISTER_LOCATION))) && /**/ ((((query_ptr -> write_start_address) >= COILS_BASE) && ((query_ptr -> start_address) < RESERVED_BASE)) || (((query_ptr -> start_address) >= HOLDING_REGISTER_BASE) && ((query_ptr -> start_address) < LAST_REGISTER_LOCATION))))
        {
            // handle multiple register read and write
            uint8_t modbus_rtu_frame_query[11 + 2*(query_ptr -> write_quantity) + 2];
            size_t length = 0;
            error_mod error_code = NO_ERROR;

            modbus_rtu_frame_query[0] = query_ptr -> slave_id; // slave id - 1 byte
            modbus_rtu_frame_query[1] = query_ptr -> function_code; // function code - 1 byte
            modbus_rtu_frame_query[2] = (((query_ptr -> start_address) >> 8) & 0xff); // start address high - 1 byte
            modbus_rtu_frame_query[3] = ((query_ptr -> start_address) & 0xff); // start address low - 1 byte
            modbus_rtu_frame_query[4] = (((query_ptr -> quantity) >> 8) & 0xff); // quatity to read high - 1 byte
            modbus_rtu_frame_query[5] = ((query_ptr -> quantity) & 0xff); // quantity to read low - 1 byte
            modbus_rtu_frame_query[6] = (((query_ptr -> write_start_address) >> 8) & 0xff); // start write address high
            modbus_rtu_frame_query[7] = ((query_ptr -> write_start_address) & 0xff); // start write address low
            modbus_rtu_frame_query[8] = (((query_ptr -> write_quantity) >> 8) & 0xff); // quantity to write high
            modbus_rtu_frame_query[9] = ((query_ptr -> write_quantity) & 0xff); // quantity to write low
            modbus_rtu_frame_query[10] = (query_ptr -> write_quantity)*2; // byte count

            for(uint16_t idx = 0; idx < ((query_ptr -> write_quantity)*2); idx++)
            {
                modbus_rtu_frame_query[11 + idx] = query_buffer[idx];
                length = 12 + idx;
            }

            modbus_rtu_crc(modbus_rtu_frame_query, sizeof(modbus_rtu_frame_query), &modbus_rtu_frame_query[length]); // calculate crc code

            length = 0;

            uart_write_bytes(UART_NUM, modbus_rtu_frame_query, sizeof(modbus_rtu_frame_query));
            uart_wait_tx_done(UART_NUM, 500);

            vTaskDelay(5000/portTICK_PERIOD_MS);

            while (!(length > 0))
            {
                uart_get_buffered_data_len(UART_NUM, &length);
                
                uart_write_bytes(UART_NUM, modbus_rtu_frame_query, sizeof(modbus_rtu_frame_query));
                uart_wait_tx_done(UART_NUM, 500);
                vTaskDelay(5000/portTICK_PERIOD_MS);
            }
            
            uint8_t response_frame[3 + ((query_ptr -> quantity)*2) + 2];

            uart_read_bytes(UART_NUM, &response_frame, sizeof(response_frame), 500);

            // verify response
            error_code = (response_frame[0] != (query_ptr -> slave_id)) ? ILLEGAL_ADDRESS : NO_ERROR;

            if((response_frame[1]  == RW_MULTIPLE_ERR_CODE) && error_code == NO_ERROR)
            {
                switch(response_frame[2]) // read exception code
                {
                    case ILLEGAL_ADDRESS:
                        return ILLEGAL_ADDRESS;
                        break;
                    case ILLEGAL_FUNCTION:
                        return ILLEGAL_FUNCTION;
                        break;
                    case ILLEGAL_DATA_SIZE:
                        return ILLEGAL_DATA_SIZE;
                        break;
                    case CRC_ERROR:
                        return CRC_ERROR;
                        break;
                    default:
                        return UNKNOWN_ERROR;
                }
            }
            else
            {
                error_code = ((response_frame[1] != (query_ptr -> function_code)) && (error_code == NO_ERROR)) ? ILLEGAL_FUNCTION : ((error_code == NO_ERROR) ? NO_ERROR : error_code);
                
                error_code = ((response_frame[2]/2 != (query_ptr -> quantity)) && (error_code == NO_ERROR)) ?  ILLEGAL_DATA_SIZE : ((error_code == NO_ERROR) ? NO_ERROR : error_code);

                uint8_t crc_buffer[2];
                modbus_rtu_crc(response_frame, sizeof(response_frame), crc_buffer);

                error_code = ((((*(&response_frame[2] + (int)response_frame[2] + 2) << 8) | (*(&response_frame[2] + response_frame[2] + 1))) != ((crc_buffer[1] << 8) | (crc_buffer[0]))) && (error_code == NO_ERROR)) ? CRC_ERROR : ((error_code == NO_ERROR) ? NO_ERROR : error_code);

                uart_flush(UART_NUM); // flush uart for the next transmission

                switch(error_code)
                {
                    case ILLEGAL_ADDRESS:
                        return ILLEGAL_ADDRESS;
                        break;
                    case ILLEGAL_FUNCTION:
                        return ILLEGAL_FUNCTION;
                        break;
                    case ILLEGAL_DATA_SIZE:
                        return ILLEGAL_DATA_SIZE;
                        break;
                    case CRC_ERROR:
                        return CRC_ERROR;
                        break;
                    default:
                        return NO_ERROR;
                }
            }
        }
        else
        {
            return ILLEGAL_ADDRESS;
        }
    }
    else
    {
        return ILLEGAL_FUNCTION;
    }
}

uint8_t modbus_rtu_write(modbus_rtu_query_frame *query_ptr, uint8_t *query_value)
{
    if(((query_ptr -> function_code) == WRITE_SINGLE_COIL)|| ((query_ptr -> function_code) == WRITE_SINGLE_REGISTER) || ((query_ptr -> function_code) == WRITE_MULTIPLE_COILS) || ((query_ptr -> function_code) == WRITE_MULTIPLE_REGISTERS))
    {
        if((((query_ptr -> start_address) >= COILS_BASE) && ((query_ptr -> start_address) < RESERVED_BASE)) || (((query_ptr -> start_address) >= HOLDING_REGISTER_BASE) && ((query_ptr -> start_address) < LAST_REGISTER_LOCATION)))
        {
            uint16_t frame_size = 0;
            uint8_t *query_val  = query_value;

            if(((query_ptr -> function_code) == WRITE_SINGLE_COIL) || ((query_ptr -> function_code) == WRITE_SINGLE_REGISTER))
            {
                frame_size = 8;
            }
            else if((query_ptr -> function_code) == WRITE_MULTIPLE_COILS)
            {
                frame_size = (9 + ((int)ceil((query_ptr -> write_quantity)/8.0)));
            }
            else if((query_ptr -> function_code) == WRITE_MULTIPLE_REGISTERS)
            {
                frame_size = (9 + 2*((query_ptr -> write_quantity)));
            }

            uint8_t modbus_rtu_frame_query[frame_size];
            size_t length = 0;
            error_mod error_code = NO_ERROR;

            modbus_rtu_frame_query[0] = query_ptr -> slave_id; // slave id - 1 byte
            modbus_rtu_frame_query[1] = query_ptr -> function_code; // function code - 1 byte
            modbus_rtu_frame_query[2] = (((query_ptr -> write_start_address) >> 8) & 0xff); // start address high - 1 byte
            modbus_rtu_frame_query[3] = ((query_ptr -> write_start_address) & 0xff); // start address low - 1 byte

            if(((query_ptr -> function_code) == WRITE_SINGLE_COIL) || ((query_ptr -> function_code) == WRITE_SINGLE_REGISTER))
            {
                modbus_rtu_frame_query[4] = (((query_value[1]) >> 8) & 0xff); // output value high - 1 byte
                modbus_rtu_frame_query[5] = ((query_value[0]) & 0xff); // output value low - 1 byte
                length = 6;
            }
            else if(((query_ptr -> function_code) == WRITE_MULTIPLE_COILS) || ((query_ptr -> function_code) == WRITE_MULTIPLE_REGISTERS))
            {
                modbus_rtu_frame_query[4] = (((query_ptr -> write_quantity) >> 8) & 0xff); // address quantity high byte
                modbus_rtu_frame_query[5] = ((query_ptr -> write_quantity) & 0xff); // address quantity low byte

                uint16_t bytes_count = ((query_ptr -> function_code) == WRITE_MULTIPLE_COILS) ? ((int)ceil((query_ptr -> write_quantity)/8.0)) : (2*(query_ptr -> write_quantity));
                
                modbus_rtu_frame_query[6] = bytes_count;
                
                for(uint8_t idx = 0; idx < bytes_count; idx++)
                {
                    modbus_rtu_frame_query[7 + idx] = query_val[idx];
                    length = 8 + idx;
                }
            }
    
            modbus_rtu_crc(modbus_rtu_frame_query, sizeof(modbus_rtu_frame_query), &modbus_rtu_frame_query[length]); // calculate crc code

            length = 0;
            /* send modbus query */
            
            uart_write_bytes(UART_NUM, modbus_rtu_frame_query, sizeof(modbus_rtu_frame_query));
            uart_wait_tx_done(UART_NUM, 500);

            vTaskDelay(5000/portTICK_PERIOD_MS);

            while (!(length > 0))
            {
                uart_get_buffered_data_len(UART_NUM, &length);
                
                uart_write_bytes(UART_NUM, modbus_rtu_frame_query, sizeof(modbus_rtu_frame_query));
                uart_wait_tx_done(UART_NUM, 500);
                vTaskDelay(5000/portTICK_PERIOD_MS);
            }

            uint16_t response_frame_size = 8;

            uint8_t response_frame[response_frame_size];

            // read response
            uart_read_bytes(UART_NUM, &response_frame, response_frame_size, 100);

            uart_flush(UART_NUM); // flush uart for the next transmission

            // verify response

            error_code = (response_frame[0] != (query_ptr -> slave_id)) ? ILLEGAL_ADDRESS : NO_ERROR;

            if((((response_frame[1]) == WRITE_SINGLE_REG_ERR_CODE) || ((response_frame[1]) == WRITE_COIL_ERR_CODE)) && error_code == NO_ERROR) // handle exceptions
            {
                switch(response_frame[2]) // read exception code
                {
                    case ILLEGAL_ADDRESS:
                        return ILLEGAL_ADDRESS;
                        break;
                    case ILLEGAL_FUNCTION:
                        return ILLEGAL_FUNCTION;
                        break;
                    case ILLEGAL_DATA_SIZE:
                        return ILLEGAL_DATA_SIZE;
                        break;
                    case CRC_ERROR:
                        return CRC_ERROR;
                        break;
                    default:
                        return UNKNOWN_ERROR;
                }
            }
            else
            {
                error_code = ((response_frame[1] != (query_ptr -> function_code)) && (error_code == NO_ERROR)) ? ILLEGAL_FUNCTION : ((error_code == NO_ERROR) ? NO_ERROR : error_code);
                error_code = ((((response_frame[2] << 8) | response_frame[3]) != (query_ptr -> write_start_address)) && (error_code == NO_ERROR)) ?  ILLEGAL_ADDRESS : ((error_code == NO_ERROR) ? NO_ERROR : error_code);
                error_code = ((((response_frame[4] << 8) | response_frame[5]) != ((query_value[1] << 8) | (query_value[0]))) && (error_code == NO_ERROR)) ?  UNKNOWN_ERROR : ((error_code == NO_ERROR) ? NO_ERROR : error_code);

                uint8_t crc_buffer[2];
                modbus_rtu_crc(response_frame, sizeof(response_frame), crc_buffer);

                error_code = ((((response_frame[7] << 8) | (response_frame[6])) != ((crc_buffer[1] << 8) | (crc_buffer[0]))) && (error_code == NO_ERROR)) ? CRC_ERROR : ((error_code == NO_ERROR) ? NO_ERROR : error_code);

                switch(error_code)
                {
                    case ILLEGAL_ADDRESS:
                        return ILLEGAL_ADDRESS;
                        break;
                    case ILLEGAL_FUNCTION:
                        return ILLEGAL_FUNCTION;
                        break;
                    case ILLEGAL_DATA_SIZE:
                        return ILLEGAL_DATA_SIZE;
                        break;
                    case CRC_ERROR:
                        return CRC_ERROR;
                        break;
                    default:
                        return NO_ERROR;
                }
            }
        }
        else
        {
            return ILLEGAL_ADDRESS;
        }
    }
    else
    {
        // error
        return ILLEGAL_FUNCTION;
    }
}

uint8_t *modbus_rtu_crc(uint8_t *modbus_rtu_frame, size_t frame_size, uint8_t *crc_buffer)
{
        
    uint16_t crc = 0xffff;

    uint8_t *modbus_rtu_frame_buffer = modbus_rtu_frame;

    for(uint8_t idx = 0; idx < (frame_size - 2); idx++) // exclude crc bytes
    {
        crc = (crc) ^ (modbus_rtu_frame_buffer[idx]);

        for(uint8_t idx_ = 0; idx_ < 8; idx_++)
        {
            if(crc & (1U << 0))
            {
                crc = ((crc >> 1) ^ 0xA001);
            }
            else
            {
                crc = (crc >> 1);
            }
        }
    }

    *(crc_buffer + 1) = (crc >> 8) & 0xff;
    *(crc_buffer) = crc & 0xff;

    return crc_buffer;
}

