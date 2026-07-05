#include "my_spi.h"

static void MySPI_W_SS(uint8_t BitValue)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, BitValue ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void MySPI_W_SCK(uint8_t BitValue)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, BitValue ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void MySPI_W_MOSI(uint8_t BitValue)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, BitValue ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint8_t MySPI_R_MISO(void)
{
    return (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_SET) ? 1 : 0;
}

void MySPI_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable GPIO Clocks */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* Configure Output Pins: CS (PC5) */
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    
    /* Configure Output Pins: CLK (PB10), MOSI (PB2) */
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Configure Input Pin: MISO (PC4) */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* Initial state */
    MySPI_W_SS(1);
    MySPI_W_SCK(0);
}

void MySPI_Start(void)
{
    MySPI_W_SS(0);
}

void MySPI_Stop(void)
{
    MySPI_W_SS(1);
}

uint8_t MySPI_SwapByte(uint8_t ByteSend)
{
    uint8_t i, ByteReceive = 0x00;
    
    for (i = 0; i < 8; i++)
    {
        MySPI_W_MOSI(!!(ByteSend & (0x80 >> i)));
        MySPI_W_SCK(1);
        if (MySPI_R_MISO()) { ByteReceive |= (0x80 >> i); }
        MySPI_W_SCK(0);
    }
    
    return ByteReceive;
}
