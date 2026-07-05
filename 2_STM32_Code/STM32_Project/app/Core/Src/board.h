#ifndef BOARD_H
#define BOARD_H

#include "main.h"

void Board_Init(void);
void Board_LoadSwitch_Write(GPIO_PinState state);
void Board_Buzzer_Write(GPIO_PinState state);
void Board_Buzzer_Beep(uint32_t duration_ms);

#endif
