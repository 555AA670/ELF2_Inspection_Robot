#ifndef __YMODEM_H
#define __YMODEM_H

#include <stdint.h>

#define PACKET_SEQNO_INDEX      (1)
#define PACKET_SEQNO_COMP_INDEX (2)

#define PACKET_HEADER           (3)
#define PACKET_TRAILER          (2)
#define PACKET_OVERHEAD         (PACKET_HEADER + PACKET_TRAILER)
#define PACKET_SIZE             (128)
#define PACKET_1K_SIZE          (1024)

#define FILE_NAME_LENGTH        (64)
#define FILE_SIZE_LENGTH        (16)

#define SOH                     (0x01)  /* start of 128-byte data packet */
#define STX                     (0x02)  /* start of 1024-byte data packet */
#define EOT                     (0x04)  /* end of transmission */
#define ACK                     (0x06)  /* acknowledge */
#define NAK                     (0x15)  /* negative acknowledge */
#define CAN                     (0x18)  /* two of these in succession aborts transfer */
#define CRC16                   (0x43)  /* 'C' == 0x43, request 16-bit CRC */
#define ABORT1                  (0x41)  /* 'A' == 0x41, abort by user */
#define ABORT2                  (0x61)  /* 'a' == 0x61, abort by user */

#define YMODEM_OK               (0)
#define YMODEM_ERROR            (1)
#define YMODEM_ABORT            (2)
#define YMODEM_TIMEOUT          (3)
#define YMODEM_DATA             (4)
#define YMODEM_LIMIT            (5)

// Prototype for the flash write function pointer
typedef uint8_t (*Ymodem_Write_Func)(uint32_t address, uint8_t *data, uint32_t length);

uint8_t Ymodem_Receive(uint32_t flash_address, Ymodem_Write_Func write_func, uint32_t *file_size);

#endif // __YMODEM_H
