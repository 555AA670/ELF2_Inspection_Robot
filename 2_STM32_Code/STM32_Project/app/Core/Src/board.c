#include "board.h"

#define BOARD_LOAD_SWITCH_PIN        GPIO_PIN_13
#define BOARD_LOAD_SWITCH_GPIO_PORT  GPIOC
#define BOARD_BUZZER_PIN             GPIO_PIN_9
#define BOARD_BUZZER_GPIO_PORT       GPIOB

void Board_SystemClock_Config(void);
static void Board_GPIO_Init(void);

void Board_Init(void)
{
  Board_SystemClock_Config();
  HAL_PWREx_DisableUCPDDeadBattery();
  Board_GPIO_Init();
}

void Board_LoadSwitch_Write(GPIO_PinState state)
{
  HAL_GPIO_WritePin(BOARD_LOAD_SWITCH_GPIO_PORT, BOARD_LOAD_SWITCH_PIN, state);
}

void Board_Buzzer_Write(GPIO_PinState state)
{
  HAL_GPIO_WritePin(BOARD_BUZZER_GPIO_PORT, BOARD_BUZZER_PIN, state);
}

void Board_Buzzer_Beep(uint32_t duration_ms)
{
  Board_Buzzer_Write(GPIO_PIN_SET);
  HAL_Delay(duration_ms);
  Board_Buzzer_Write(GPIO_PIN_RESET);
}

void Board_SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

static void Board_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Prevent glitch by setting PC13 High before Init */
  HAL_GPIO_WritePin(BOARD_LOAD_SWITCH_GPIO_PORT, BOARD_LOAD_SWITCH_PIN, GPIO_PIN_SET);
  
  GPIO_InitStruct.Pin = BOARD_LOAD_SWITCH_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BOARD_LOAD_SWITCH_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = BOARD_BUZZER_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BOARD_BUZZER_GPIO_PORT, &GPIO_InitStruct);

  /* Keep the external load ON to prevent RK3588 from rebooting */
  HAL_GPIO_WritePin(BOARD_LOAD_SWITCH_GPIO_PORT, BOARD_LOAD_SWITCH_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BOARD_BUZZER_GPIO_PORT, BOARD_BUZZER_PIN, GPIO_PIN_RESET);
}

