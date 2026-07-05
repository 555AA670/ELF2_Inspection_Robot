#include "ymodem.h"
#include "uart.h"
#include "stm32g4xx_hal.h"
#include <string.h>

static uint16_t UpdateCRC16(uint16_t crc_in, uint8_t byte)
{
    uint32_t crc = crc_in;
    uint32_t in = byte | 0x100;

    do {
        crc <<= 1;
        in <<= 1;
        if (in & 0x100) {
            ++crc;
        }
        if (crc & 0x10000) {
            crc ^= 0x1021;
        }
    } while (!(in & 0x10000));

    return crc & 0xffffu;
}

static uint16_t Cal_CRC16(const uint8_t* p_data, uint32_t size)
{
    uint32_t crc = 0;
    while (size--) {
        crc = UpdateCRC16(crc, *p_data++);
    }
    crc = UpdateCRC16(crc, 0);
    crc = UpdateCRC16(crc, 0);
    return crc & 0xffffu;
}

static uint8_t Receive_Byte(uint8_t *c, uint32_t timeout)
{
    uint32_t tickstart = HAL_GetTick();
    while ((HAL_GetTick() - tickstart) < timeout) {
        if (UART_ReadByte(c)) {
            return 0; // Success
        }
    }
    return 1; // Timeout
}

static uint32_t Receive_Packet(uint8_t *data, uint32_t *length, uint32_t timeout)
{
    uint16_t i, packet_size;
    uint8_t c;
    *length = 0;
    if (Receive_Byte(&c, timeout) != 0) {
        return 1;
    }

    switch (c) {
        case SOH:
            packet_size = PACKET_SIZE;
            break;
        case STX:
            packet_size = PACKET_1K_SIZE;
            break;
        case EOT:
            return 2;
        case CAN:
            if (Receive_Byte(&c, timeout) == 0 && c == CAN) {
                return 3;
            }
            return 1;
        case ABORT1:
        case ABORT2:
            return 3;
        default:
            return 1;
    }

    *data = c;
    for (i = 1; i < (packet_size + PACKET_OVERHEAD); i++) {
        if (Receive_Byte(data + i, timeout) != 0) {
            return 1;
        }
    }

    if (data[PACKET_SEQNO_INDEX] != ((data[PACKET_SEQNO_COMP_INDEX] ^ 0xFF) & 0xFF)) {
        return 1;
    }

    uint16_t crc = Cal_CRC16(&data[PACKET_HEADER], packet_size);
    if (crc != ((data[packet_size + PACKET_HEADER] << 8) | data[packet_size + PACKET_HEADER + 1])) {
        return 1;
    }

    *length = packet_size;
    return 0;
}

uint8_t Ymodem_Receive(uint32_t flash_address, Ymodem_Write_Func write_func, uint32_t *file_size)
{
    static uint8_t packet_data[PACKET_1K_SIZE + PACKET_OVERHEAD];
    uint32_t packet_length;
    uint8_t session_done = 0, errors = 0, session_begin = 0;
    uint32_t current_address = flash_address;
    uint32_t recv_size = 0;
    uint32_t packets_received = 0;
    
    *file_size = 0;

    // Send 'C' to start
    UART_SendByte(CRC16);
    
    while (session_done == 0) {
        uint32_t res = Receive_Packet(packet_data, &packet_length, 1000); // 1 second timeout
        
        if (res == 0) { // Packet received
            errors = 0;
            uint8_t is_file_header = 0;
            
            if (packet_data[PACKET_SEQNO_INDEX] == 0 && packet_data[PACKET_SEQNO_COMP_INDEX] == 0xFF) {
                // Header packet
                if (packet_data[PACKET_HEADER] == 0) {
                    // Empty header -> end of session
                    session_done = 1;
                    UART_SendByte(ACK);
                    break;
                }
                
                // Parse file size
                char *ptr = (char *)&packet_data[PACKET_HEADER];
                while (*ptr != '\0') ptr++;
                ptr++; // Skip null byte
                uint32_t f_size = 0;
                while (*ptr != ' ' && *ptr != '\0') {
                    f_size = f_size * 10 + (*ptr - '0');
                    ptr++;
                }
                if (session_begin == 0) {
                    *file_size = f_size;
                    session_begin = 1;
                }
                is_file_header = 1;
                UART_SendByte(ACK);
                HAL_Delay(10);
                UART_SendByte(CRC16); // Request data
            } else {
                // Data packet
                if (packet_data[PACKET_SEQNO_INDEX] == ((packets_received + 1) & 0xFF)) {
                    // Valid next packet
                    packets_received++;
                    uint32_t chunk_size = packet_length;
                    if (*file_size > 0 && (recv_size + packet_length > *file_size)) {
                        chunk_size = *file_size - recv_size; // Only write actual file size
                    }
                    
                    // Call write function
                    if (write_func(current_address, &packet_data[PACKET_HEADER], chunk_size) != 0) {
                        UART_SendByte(CAN); UART_SendByte(CAN);
                        return YMODEM_ERROR;
                    }
                    current_address += chunk_size;
                    recv_size += chunk_size;
                }
                UART_SendByte(ACK);
            }
        } else if (res == 2) { // EOT
            UART_SendByte(ACK);
            HAL_Delay(10);
            UART_SendByte(CRC16); // Wait for the empty packet closing session
        } else if (res == 3) { // Abort
            return YMODEM_ABORT;
        } else { // Timeout or CRC error
            errors++;
            if (errors > 5) {
                UART_SendByte(CAN); UART_SendByte(CAN);
                return YMODEM_TIMEOUT;
            }
            if (session_begin) {
                UART_SendByte(NAK);
            } else {
                UART_SendByte(CRC16);
            }
        }
    }
    
    return YMODEM_OK;
}
