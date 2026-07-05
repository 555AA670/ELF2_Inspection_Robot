#include "uart.h"

#define USART3_TX_PIN              GPIO_PIN_10
#define USART3_RX_PIN              GPIO_PIN_11
#define USART3_GPIO_PORT           GPIOC
#define USART3_BAUD_RATE           115200U
#define USART3_RX_RING_BUFFER_SIZE 1024U

static volatile uint8_t rx_buffer[USART3_RX_RING_BUFFER_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;

void UART_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();

    GPIO_InitStruct.Pin = USART3_TX_PIN | USART3_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(USART3_GPIO_PORT, &GPIO_InitStruct);

    __HAL_RCC_USART3_FORCE_RESET();
    __HAL_RCC_USART3_RELEASE_RESET();

    uint32_t uart_clk_hz = HAL_RCC_GetPCLK1Freq();
    uint32_t uart_brr = (uart_clk_hz + (USART3_BAUD_RATE / 2U)) / USART3_BAUD_RATE;
    if (uart_brr == 0U) uart_brr = 1U;

    rx_head = 0;
    rx_tail = 0;

    USART3->CR1 = 0;
    USART3->CR2 = 0;
    USART3->CR3 = 0;
    USART3->BRR = uart_brr;
    USART3->ICR = 0xFFFFFFFFU;
    USART3->RQR = USART_RQR_RXFRQ;
    USART3->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE_RXFNEIE;
    USART3->CR1 |= USART_CR1_UE;

    HAL_NVIC_SetPriority(USART3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
}

void USART3_IRQHandler(void)
{
    uint32_t isr = USART3->ISR;

    if ((isr & (USART_ISR_ORE | USART_ISR_NE | USART_ISR_FE | USART_ISR_PE)) != 0U)
    {
        USART3->ICR = USART_ICR_ORECF | USART_ICR_NECF | USART_ICR_FECF | USART_ICR_PECF;
    }

    while ((USART3->ISR & USART_ISR_RXNE_RXFNE) != 0U)
    {
        uint8_t data = (uint8_t)(USART3->RDR & 0xFFU);
        uint16_t next_head = (rx_head + 1) % USART3_RX_RING_BUFFER_SIZE;
        if (next_head != rx_tail)
        {
            rx_buffer[rx_head] = data;
            rx_head = next_head;
        }
    }
}

void UART_SendByte(uint8_t data)
{
    while ((USART3->ISR & USART_ISR_TXE_TXFNF) == 0U);
    USART3->TDR = data;
}

void UART_SendString(const char *text)
{
    while (*text != '\0')
    {
        UART_SendByte((uint8_t)*text);
        text++;
    }
}

uint8_t UART_ReadByte(uint8_t *data)
{
    if (rx_head == rx_tail) return 0;
    *data = rx_buffer[rx_tail];
    rx_tail = (rx_tail + 1) % USART3_RX_RING_BUFFER_SIZE;
    return 1;
}

uint8_t UART_DataAvailable(void)
{
    return (rx_head != rx_tail) ? 1 : 0;
}
