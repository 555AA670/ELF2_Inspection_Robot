#include "main.h"
#include "OLED.h"
#include "uart2_oled.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define USART3_TX_PIN              GPIO_PIN_10
#define USART3_RX_PIN              GPIO_PIN_11
#define USART3_GPIO_PORT           GPIOC
#define USART3_BAUD_RATE           115200U
#define USART3_RX_RING_BUFFER_SIZE 256U
#define UART2_OLED_LINE_COUNT      6U
#define UART2_OLED_LINE_LENGTH     21U

static volatile uint8_t USART3_rx_ring_buffer[USART3_RX_RING_BUFFER_SIZE];
static volatile uint16_t USART3_rx_ring_head = 0U;
static volatile uint16_t USART3_rx_ring_tail = 0U;
static volatile uint32_t USART3_rx_overflow_count = 0U;
static char uart2_oled_lines[UART2_OLED_LINE_COUNT][UART2_OLED_LINE_LENGTH + 1U];
static uint8_t uart2_oled_row = 0U;
static uint8_t uart2_oled_col = 0U;
static char uart2_line_buffer[96];
static uint8_t uart2_line_length = 0U;
static volatile uint8_t uart2_line_ready = 0U;

static void USART3_GPIO_Init(void);
static void USART3_Init(void);
static uint8_t USART3_ReadByte(uint8_t *data);
static void USART3_RxBuffer_PushByte(uint8_t data);
static void USART3_SendByte(uint8_t data);
static void UART2_OLED_NewLine(void);
static void UART2_OLED_PushChar(uint8_t data);

void UART2_OLED_Init(void)
{
  USART3_GPIO_Init();
  USART3_Init();
  UART2_OLED_ClearText();
}

void UART2_OLED_ProcessRx(void)
{
  uint8_t uart2_rx_data;

  while (USART3_ReadByte(&uart2_rx_data) != 0U)
  {
    UART2_OLED_PushChar(uart2_rx_data);
  }
}

void UART2_OLED_RenderDashboard(uint8_t sensor_ready,
                                float voltage_v, float current_a, float soc_percent,
                                int32_t ax_mg, int32_t ay_mg, int32_t az_mg,
                                float gx_dps, float gy_dps, float gz_dps)
{
  if (sensor_ready != 0U)
  {
    UART2_OLED_Printf("V:%5.2f I:%+4.2f SOC:%5.1f%%\r\n", (double)voltage_v, (double)current_a, (double)soc_percent);
  }
  else
  {
    UART2_OLED_Printf("V:FAIL I:FAIL SOC:FAIL\r\n");
  }

  UART2_OLED_Printf("AX:%+6ld AY:%+6ld AZ:%+6ld\r\n", (long)ax_mg, (long)ay_mg, (long)az_mg);
  UART2_OLED_Printf("GX:%+5.1f GY:%+5.1f GZ:%+5.1f\r\n", (double)gx_dps, (double)gy_dps, (double)gz_dps);
  UART2_OLED_Printf("--------------------------------\r\n");
}

void UART2_OLED_ClearText(void)
{
  uint32_t row;

  for (row = 0U; row < UART2_OLED_LINE_COUNT; row++)
  {
    (void)memset(uart2_oled_lines[row], 0, sizeof(uart2_oled_lines[row]));
  }

  uart2_oled_row = 0U;
  uart2_oled_col = 0U;
}

uint32_t UART2_OLED_GetOverflowCount(void)
{
  return USART3_rx_overflow_count;
}

uint8_t UART2_OLED_ReadLine(char *buffer, size_t buffer_len)
{
  size_t copy_len;

  if ((buffer == NULL) || (buffer_len == 0U) || (uart2_line_ready == 0U))
  {
    return 0U;
  }

  copy_len = (size_t)uart2_line_length;
  if (copy_len >= buffer_len)
  {
    copy_len = buffer_len - 1U;
  }

  (void)memcpy(buffer, uart2_line_buffer, copy_len);
  buffer[copy_len] = '\0';
  uart2_line_ready = 0U;
  uart2_line_length = 0U;
  uart2_line_buffer[0] = '\0';
  return 1U;
}

void UART2_OLED_SendString(const char *text)
{
  if (text == NULL)
  {
    return;
  }

  while (*text != '\0')
  {
    USART3_SendByte((uint8_t)*text);
    text++;
  }
}

void UART2_OLED_Printf(const char *format, ...)
{
  char buffer[128];
  va_list args;

  va_start(args, format);
  (void)vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  UART2_OLED_SendString(buffer);
}

void USART3_IRQHandler_Callback(void)
{
  uint32_t isr;

  isr = USART3->ISR;

  if ((isr & (USART_ISR_ORE | USART_ISR_NE | USART_ISR_FE | USART_ISR_PE)) != 0U)
  {
    USART3->ICR = USART_ICR_ORECF | USART_ICR_NECF | USART_ICR_FECF | USART_ICR_PECF;
  }

  while ((USART3->ISR & USART_ISR_RXNE_RXFNE) != 0U)
  {
    USART3_RxBuffer_PushByte((uint8_t)(USART3->RDR & 0xFFU));
  }
}

static void USART3_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitStruct.Pin = USART3_TX_PIN | USART3_RX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(USART3_GPIO_PORT, &GPIO_InitStruct);
}

static void USART3_Init(void)
{
  uint32_t uart_clk_hz;
  uint32_t uart_brr;

  __HAL_RCC_USART3_CLK_ENABLE();
  __HAL_RCC_USART3_FORCE_RESET();
  __HAL_RCC_USART3_RELEASE_RESET();

  uart_clk_hz = HAL_RCC_GetPCLK1Freq();
  uart_brr = (uart_clk_hz + (USART3_BAUD_RATE / 2U)) / USART3_BAUD_RATE;
  if (uart_brr == 0U)
  {
    uart_brr = 1U;
  }

  USART3_rx_ring_head = 0U;
  USART3_rx_ring_tail = 0U;
  USART3_rx_overflow_count = 0U;

  USART3->CR1 = 0U;
  USART3->CR2 = 0U;
  USART3->CR3 = 0U;
  USART3->BRR = uart_brr;
  USART3->ICR = 0xFFFFFFFFU;
  USART3->RQR = USART_RQR_RXFRQ;
  USART3->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE_RXFNEIE;
  USART3->CR1 |= USART_CR1_UE;

  HAL_NVIC_SetPriority(USART3_IRQn, 1U, 0U);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
}

static uint8_t USART3_ReadByte(uint8_t *data)
{
  uint16_t tail;

  tail = USART3_rx_ring_tail;
  if (tail == USART3_rx_ring_head)
  {
    return 0U;
  }

  *data = USART3_rx_ring_buffer[tail];
  tail++;
  if (tail >= USART3_RX_RING_BUFFER_SIZE)
  {
    tail = 0U;
  }
  USART3_rx_ring_tail = tail;
  return 1U;
}

static void USART3_SendByte(uint8_t data)
{
  while ((USART3->ISR & USART_ISR_TXE_TXFNF) == 0U)
  {
  }
  USART3->TDR = data;
}

static void USART3_RxBuffer_PushByte(uint8_t data)
{
  uint16_t next_head;

  next_head = USART3_rx_ring_head + 1U;
  if (next_head >= USART3_RX_RING_BUFFER_SIZE)
  {
    next_head = 0U;
  }

  if (next_head == USART3_rx_ring_tail)
  {
    USART3_rx_overflow_count++;
    return;
  }

  USART3_rx_ring_buffer[USART3_rx_ring_head] = data;
  USART3_rx_ring_head = next_head;
}

static void UART2_OLED_NewLine(void)
{
  uint32_t row;

  if (uart2_oled_row < (UART2_OLED_LINE_COUNT - 1U))
  {
    uart2_oled_row++;
  }
  else
  {
    for (row = 0U; row < (UART2_OLED_LINE_COUNT - 1U); row++)
    {
      (void)memcpy(uart2_oled_lines[row], uart2_oled_lines[row + 1U], sizeof(uart2_oled_lines[row]));
    }
    (void)memset(uart2_oled_lines[UART2_OLED_LINE_COUNT - 1U], 0,
                 sizeof(uart2_oled_lines[UART2_OLED_LINE_COUNT - 1U]));
  }

  uart2_oled_col = 0U;
}

static void UART2_OLED_PushChar(uint8_t data)
{
  if (data == '\r')
  {
    return;
  }

  if (data == '\n')
  {
    if ((uart2_line_length > 0U) && (uart2_line_ready == 0U))
    {
      uart2_line_buffer[uart2_line_length] = '\0';
      uart2_line_ready = 1U;
    }
    UART2_OLED_NewLine();
    return;
  }

  if (data == '\b')
  {
    if (uart2_oled_col > 0U)
    {
      uart2_oled_col--;
      uart2_oled_lines[uart2_oled_row][uart2_oled_col] = '\0';
    }
    return;
  }

  if ((data < 32U) || (data > 126U))
  {
    data = '.';
  }

  if ((uart2_line_ready == 0U) && (uart2_line_length < (sizeof(uart2_line_buffer) - 1U)))
  {
    uart2_line_buffer[uart2_line_length] = (char)data;
    uart2_line_length++;
    uart2_line_buffer[uart2_line_length] = '\0';
  }

  uart2_oled_lines[uart2_oled_row][uart2_oled_col] = (char)data;
  if (uart2_oled_col < (UART2_OLED_LINE_LENGTH - 1U))
  {
    uart2_oled_col++;
  }
  else
  {
    UART2_OLED_NewLine();
  }

  uart2_oled_lines[uart2_oled_row][uart2_oled_col] = '\0';
}
