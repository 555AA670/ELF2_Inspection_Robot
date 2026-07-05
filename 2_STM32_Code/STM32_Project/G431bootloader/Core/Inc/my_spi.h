#ifndef __MY_SPI_H__
#define __MY_SPI_H__

#include "main.h"

void MySPI_Init(void);
void MySPI_Start(void);
void MySPI_Stop(void);
uint8_t MySPI_SwapByte(uint8_t ByteSend);

#endif
