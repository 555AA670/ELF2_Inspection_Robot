#ifndef __UART_H
#define __UART_H

#include "stm32g4xx_hal.h"

void UART_Init(void);
void UART_SendByte(uint8_t data);
void UART_SendString(const char *text);
uint8_t UART_ReadByte(uint8_t *data);
uint8_t UART_DataAvailable(void);

#endif // __UART_H
