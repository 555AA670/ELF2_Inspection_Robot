#include "main.h"
#include "uart.h"
#include "flash.h"
#include "ymodem.h"
#include "w25q64.h"
#include "boot_shared.h"
#include <string.h>

void SystemClock_Config(void);

uint8_t W25Q64_WriteFunc(uint32_t address, uint8_t *data, uint32_t length)
{
    // Y-modem packet length could be up to 1024. W25Q64 page is 256 bytes.
    // Also need to handle sector erase (4KB).
    uint32_t ext_addr = OTA_AREA_ADDRESS + address;
    
    // If it's at the start of a 4KB sector, erase it
    if ((ext_addr % 4096) == 0) {
        W25Q64_SectorErase(ext_addr);
    }
    // Also if the write crosses a 4KB boundary, we must erase the next sector
    if ((ext_addr / 4096) != ((ext_addr + length - 1) / 4096)) {
        W25Q64_SectorErase((ext_addr + length - 1) & ~0xFFF);
    }

    uint32_t bytes_written = 0;
    while (bytes_written < length) {
        uint32_t page_offset = (ext_addr + bytes_written) % 256;
        uint32_t bytes_to_write = 256 - page_offset;
        if (bytes_to_write > (length - bytes_written)) {
            bytes_to_write = (length - bytes_written);
        }
        
        W25Q64_PageProgram(ext_addr + bytes_written, data + bytes_written, bytes_to_write);
        bytes_written += bytes_to_write;
    }
    
    return 0; // Success
}

void PC13_Init(void) {
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // Set HIGH before init to prevent glitch
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void PB7_Init(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    PC13_Init(); // Pull PC13 HIGH immediately to keep RK3588 powered
    UART_Init();
    W25Q64_Init();
    PB7_Init();
    
    uint8_t pb7_pressed = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET);
    
    UART_SendString("\r\n=================================\r\n");
    UART_SendString("   STM32G431 Bootloader V2.0\r\n");
    UART_SendString("=================================\r\n");
    
    uint8_t mid;
    uint16_t did;
    W25Q64_ReadID(&mid, &did);
    if (mid != 0xEF) {
        UART_SendString("W25Q64 Not Found!\r\n");
    } else {
        UART_SendString("W25Q64 Ready.\r\n");
    }

    BootInfo_t boot_info;
    W25Q64_ReadData(BOOT_INFO_ADDRESS, (uint8_t*)&boot_info, sizeof(BootInfo_t));
    if (boot_info.magic != BOOT_MAGIC_WORD) {
        UART_SendString("BootInfo invalid. Initializing...\r\n");
        memset(&boot_info, 0, sizeof(BootInfo_t));
        boot_info.magic = BOOT_MAGIC_WORD;
        W25Q64_SectorErase(BOOT_INFO_ADDRESS);
        W25Q64_PageProgram(BOOT_INFO_ADDRESS, (uint8_t*)&boot_info, sizeof(BootInfo_t));
    }
    
    if (boot_info.boot_failure_count >= MAX_BOOT_FAIL_COUNT) {
        if (boot_info.backup_app_size > 0) {
            UART_SendString("Boot Failed 3 times! Rolling back...\r\n");
            InternalFlash_Erase(APP_START_ADDRESS, boot_info.backup_app_size);
            uint8_t buffer[256];
            uint32_t bytes_copied = 0;
            while (bytes_copied < boot_info.backup_app_size) {
                uint32_t chunk = (boot_info.backup_app_size - bytes_copied) > 256 ? 256 : (boot_info.backup_app_size - bytes_copied);
                W25Q64_ReadData(BACKUP_AREA_ADDRESS + bytes_copied, buffer, chunk);
                InternalFlash_Write(APP_START_ADDRESS + bytes_copied, buffer, chunk);
                bytes_copied += chunk;
            }
            UART_SendString("Rollback success!\r\n");
            
            // Restore current_app_size
            boot_info.current_app_size = boot_info.backup_app_size;
            boot_info.boot_failure_count = 0;
            W25Q64_SectorErase(BOOT_INFO_ADDRESS);
            W25Q64_PageProgram(BOOT_INFO_ADDRESS, (uint8_t*)&boot_info, sizeof(BootInfo_t));
            
            // REBOOT to start the restored app cleanly
            HAL_Delay(100);
            NVIC_SystemReset();
        } else {
            UART_SendString("Boot Failed 3 times! No backup available. Clearing count...\r\n");
            boot_info.boot_failure_count = 0;
            W25Q64_SectorErase(BOOT_INFO_ADDRESS);
            W25Q64_PageProgram(BOOT_INFO_ADDRESS, (uint8_t*)&boot_info, sizeof(BootInfo_t));
        }
    }
    
    if (boot_info.is_ota_ready == 1) {
        UART_SendString("New OTA detected. Updating...\r\n");
        // Backup current app if it exists
        if (boot_info.current_app_size > 0) {
            UART_SendString("Backing up current app...\r\n");
            boot_info.backup_app_size = boot_info.current_app_size;
            uint8_t buffer[256];
            uint32_t bytes_copied = 0;
            uint32_t sectors_to_erase = (boot_info.current_app_size + 4095) / 4096;
            for (uint32_t i = 0; i < sectors_to_erase; i++) {
                W25Q64_SectorErase(BACKUP_AREA_ADDRESS + i * 4096);
            }
            while (bytes_copied < boot_info.current_app_size) {
                uint32_t chunk = (boot_info.current_app_size - bytes_copied) > 256 ? 256 : (boot_info.current_app_size - bytes_copied);
                memcpy(buffer, (void*)(APP_START_ADDRESS + bytes_copied), chunk);
                
                // Write chunk to W25Q64 manually handling page boundaries
                uint32_t w_bytes = 0;
                while (w_bytes < chunk) {
                    uint32_t w_chunk = (chunk - w_bytes) > 256 ? 256 : (chunk - w_bytes);
                    W25Q64_PageProgram(BACKUP_AREA_ADDRESS + bytes_copied + w_bytes, buffer + w_bytes, w_chunk);
                    w_bytes += w_chunk;
                }
                
                bytes_copied += chunk;
            }
        }
        
        // Flash new app
        UART_SendString("Flashing new app...\r\n");
        InternalFlash_Erase(APP_START_ADDRESS, boot_info.ota_size);
        uint8_t buffer[256];
        uint32_t bytes_copied = 0;
        while (bytes_copied < boot_info.ota_size) {
            uint32_t chunk = (boot_info.ota_size - bytes_copied) > 256 ? 256 : (boot_info.ota_size - bytes_copied);
            W25Q64_ReadData(OTA_AREA_ADDRESS + bytes_copied, buffer, chunk);
            InternalFlash_Write(APP_START_ADDRESS + bytes_copied, buffer, chunk);
            bytes_copied += chunk;
        }
        
        boot_info.current_app_size = boot_info.ota_size;
        boot_info.is_ota_ready = 0;
        boot_info.boot_failure_count = 0;
        W25Q64_SectorErase(BOOT_INFO_ADDRESS);
        W25Q64_PageProgram(BOOT_INFO_ADDRESS, (uint8_t*)&boot_info, sizeof(BootInfo_t));
        
        UART_SendString("Update complete! Rebooting...\r\n");
        HAL_Delay(100);
        NVIC_SystemReset();
    }

    if (boot_info.iap_request == 1 || pb7_pressed) {
        if (pb7_pressed) {
            UART_SendString("PB7 LOW detected! ");
        }
        UART_SendString("Waiting for Y-Modem update...\r\n");
        // Clear the IAP request flag immediately so we don't get stuck if we reboot
        boot_info.iap_request = 0;
        W25Q64_SectorErase(BOOT_INFO_ADDRESS);
        W25Q64_PageProgram(BOOT_INFO_ADDRESS, (uint8_t*)&boot_info, sizeof(BootInfo_t));
        
        uint32_t file_size = 0;
        uint8_t status = YMODEM_TIMEOUT;
        for (int tries = 0; tries < 3; tries++) {
            status = Ymodem_Receive(0, W25Q64_WriteFunc, &file_size);
            if (status == YMODEM_OK || status == YMODEM_ABORT) {
                break;
            }
        }
    
        if (status == YMODEM_OK && file_size > 0) {
            UART_SendString("\r\nUpdate received. Ready for install.\r\n");
            boot_info.is_ota_ready = 1;
            boot_info.ota_size = file_size;
            W25Q64_SectorErase(BOOT_INFO_ADDRESS);
            W25Q64_PageProgram(BOOT_INFO_ADDRESS, (uint8_t*)&boot_info, sizeof(BootInfo_t));
        } else {
            UART_SendString("\r\nUpdate failed or timed out.\r\n");
        }
        
        UART_SendString("Rebooting...\r\n");
        HAL_Delay(100);
        NVIC_SystemReset();
    }
    
    // Increment failure count for normal boot
    boot_info.boot_failure_count++;
    W25Q64_SectorErase(BOOT_INFO_ADDRESS);
    W25Q64_PageProgram(BOOT_INFO_ADDRESS, (uint8_t*)&boot_info, sizeof(BootInfo_t));

    HAL_Delay(100);
    Bootloader_JumpToApp();
    
    // If jump fails (no valid app)
    UART_SendString("Jump failed! No valid application.\r\n");
    while (1) {
        HAL_Delay(1000);
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
    RCC_OscInitStruct.PLL.PLLN = 85;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}
