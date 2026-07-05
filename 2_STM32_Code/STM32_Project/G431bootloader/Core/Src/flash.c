#include "flash.h"

void InternalFlash_Erase(uint32_t address, uint32_t size)
{
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

    uint32_t FirstPage = 0, NbOfPages = 0;
    // Assuming STM32G431 2KB page size (Category 2 devices usually have 2KB pages in dual bank, 
    // but STM32G431x6/8/B are Category 2 with 128KB flash, usually single bank 2KB).
    // Let's use HAL_FLASHEx_Erase.
    FirstPage = (address - FLASH_BASE) / FLASH_PAGE_SIZE;
    NbOfPages = (size + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE;

    FLASH_EraseInitTypeDef EraseInitStruct;
    EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Banks       = FLASH_BANK_1;
    EraseInitStruct.Page        = FirstPage;
    EraseInitStruct.NbPages     = NbOfPages;

    uint32_t PageError = 0;
    HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);

    HAL_FLASH_Lock();
}

void InternalFlash_Write(uint32_t address, uint8_t *data, uint32_t size)
{
    HAL_FLASH_Unlock();
    
    // Write in Double Words (64 bits)
    for (uint32_t i = 0; i < size; i += 8)
    {
        uint64_t dword = 0xFFFFFFFFFFFFFFFF;
        uint32_t bytes_to_copy = ((size - i) < 8) ? (size - i) : 8;
        
        // Build the 64-bit word
        uint8_t *p_dword = (uint8_t*)&dword;
        for (uint32_t j = 0; j < bytes_to_copy; j++)
        {
            p_dword[j] = data[i + j];
        }

        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address + i, dword);
    }
    
    HAL_FLASH_Lock();
}

typedef void (*pFunction)(void);

void Bootloader_JumpToApp(void)
{
    uint32_t app_address = APP_START_ADDRESS;
    
    // Check if the first word (Stack Pointer) is within SRAM range (0x20000000 - 0x20007FFF)
    // Actually SRAM1 is 22KB, SRAM2 is 10KB. Total 32KB for G431.
    uint32_t app_sp = *(__IO uint32_t*)app_address;
    uint32_t app_reset_handler = *(__IO uint32_t*)(app_address + 4);
    
    // Check if SP is in SRAM and Reset Handler is in Flash (0x08000000)
    if (((app_sp & 0x2FFE0000) == 0x20000000) && ((app_reset_handler & 0x2FFE0000) == 0x08000000))
    {
        // De-init all peripherals and systick
        HAL_RCC_DeInit();
        HAL_DeInit();
        SysTick->CTRL = 0;
        SysTick->LOAD = 0;
        SysTick->VAL = 0;
        
        // Disable all interrupts
        for (int i = 0; i < 8; i++) {
            NVIC->ICER[i] = 0xFFFFFFFF;
            NVIC->ICPR[i] = 0xFFFFFFFF;
        }
        
        // Relocate vector table
        SCB->VTOR = app_address;
        
        // Set Stack Pointer and jump
        uint32_t app_entry = *(__IO uint32_t*)(app_address + 4);
        pFunction JumpToApp = (pFunction)app_entry;
        
        __set_MSP(app_sp);
        JumpToApp();
    }
}
