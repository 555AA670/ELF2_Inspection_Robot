#ifndef __FLASH_H
#define __FLASH_H

#include "stm32g4xx_hal.h"

// Application start address
#define APP_START_ADDRESS 0x08004000

void InternalFlash_Erase(uint32_t address, uint32_t size);
void InternalFlash_Write(uint32_t address, uint8_t *data, uint32_t size);
void Bootloader_JumpToApp(void);

#endif // __FLASH_H
