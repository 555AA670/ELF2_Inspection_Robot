#include "motor.h"

#define MOTOR_IN1_PIN                GPIO_PIN_9
#define MOTOR_IN2_PIN                GPIO_PIN_8
#define MOTOR_IN_GPIO_PORT           GPIOC

#define MOTOR2_IN1_PIN               GPIO_PIN_9
#define MOTOR2_IN2_PIN               GPIO_PIN_10
#define MOTOR2_IN_GPIO_PORT          GPIOA

#define MOTOR_ENC_A_PIN              GPIO_PIN_6
#define MOTOR_ENC_B_PIN              GPIO_PIN_7
#define MOTOR_ENC_GPIO_PORT          GPIOC
#define MOTOR2_ENC_A_PIN             GPIO_PIN_4
#define MOTOR2_ENC_B_PIN             GPIO_PIN_5
#define MOTOR2_ENC_GPIO_PORT         GPIOB

#define MOTOR_PWM_FREQ_HZ            20000U
#define MOTOR1_ENCODER_FORWARD_SIGN  (-1)
#define MOTOR2_ENCODER_FORWARD_SIGN  1

typedef enum
{
  MOTOR_DRIVE_STOP = 0,
  MOTOR_DRIVE_FORWARD = 1,
  MOTOR_DRIVE_REVERSE = 2,
  MOTOR_DRIVE_BRAKE = 3,
} MotorDriveMode;

static uint16_t motor_encoder_last_count = 0U;
static uint16_t motor2_encoder_last_count = 0U;
static volatile uint8_t motor_soft_pwm_counter = 0U;
static volatile uint8_t motor_control_1ms_flag = 0U;

static MotorDriveMode motor_left_mode = MOTOR_DRIVE_STOP;
static MotorDriveMode motor_right_mode = MOTOR_DRIVE_STOP;

static uint32_t motor1_dma_up_val = 0;
static uint32_t motor1_dma_cc_val = 0;

static void Motor_GPIO_Init(void);
static void Motor2_GPIO_Init(void);
static void Motor_PWM_Init(void);
static void Motor1_DMA_Timer_Init(void);
static void Motor_Encoder_Init(void);
static void Motor2_Encoder_Init(void);
static int32_t Motor_ReadLeftDeltaRaw(void);
static int32_t Motor_ReadRightDeltaRaw(void);

void Motor_Init(void)
{
  Motor_GPIO_Init();
  Motor2_GPIO_Init();
  Motor_PWM_Init();
  Motor1_DMA_Timer_Init();
  Motor_Encoder_Init();
  Motor2_Encoder_Init();
}

void Motor_SetLeftDutyPercent(int8_t duty_percent)
{
  if (duty_percent > 100) duty_percent = 100;
  else if (duty_percent < -100) duty_percent = -100;

  uint8_t magnitude = (uint8_t)((duty_percent < 0) ? -duty_percent : duty_percent);
  uint8_t reverse = (duty_percent < 0) ? 1U : 0U;
  uint32_t period_counts = TIM2->ARR + 1U;

  if (magnitude == 0U)
  {
    motor_left_mode = MOTOR_DRIVE_STOP;
    motor1_dma_up_val = 0;
    motor1_dma_cc_val = 0;
    GPIOC->BSRR = (MOTOR_IN1_PIN | MOTOR_IN2_PIN) << 16U; 
    return;
  }

  uint32_t compare = (period_counts * (100U - magnitude)) / 100U;
  if (magnitude == 100U) compare = 0; // 0% high time when full speed
  
  TIM2->CCR1 = compare;

  if (reverse == 0U)
  {
    motor_left_mode = MOTOR_DRIVE_FORWARD;
    GPIOC->BSRR = MOTOR_IN1_PIN; // PC9 HIGH constantly
    motor1_dma_up_val = MOTOR_IN2_PIN; // PC8 HIGH at UP
    motor1_dma_cc_val = MOTOR_IN2_PIN << 16U; // PC8 LOW at CC
  }
  else
  {
    motor_left_mode = MOTOR_DRIVE_REVERSE;
    GPIOC->BSRR = MOTOR_IN2_PIN; // PC8 HIGH constantly
    motor1_dma_up_val = MOTOR_IN1_PIN; // PC9 HIGH at UP
    motor1_dma_cc_val = MOTOR_IN1_PIN << 16U; // PC9 LOW at CC
  }
}

void Motor_SetRightDutyPercent(int8_t duty_percent)
{
  if (duty_percent > 100) duty_percent = 100;
  else if (duty_percent < -100) duty_percent = -100;

  uint8_t magnitude = (uint8_t)((duty_percent < 0) ? -duty_percent : duty_percent);
  uint8_t reverse = (duty_percent < 0) ? 1U : 0U;
  uint32_t period_counts = TIM1->ARR + 1U;

  if (magnitude == 0U)
  {
    motor_right_mode = MOTOR_DRIVE_STOP;
    TIM1->CCR2 = 0; 
    TIM1->CCR3 = 0; 
    return;
  }

  uint32_t compare = (period_counts * (100U - magnitude)) / 100U;
  if (magnitude == 100U) compare = 0;

  if (reverse == 0U)
  {
    motor_right_mode = MOTOR_DRIVE_FORWARD;
    TIM1->CCR3 = period_counts + 1U; 
    TIM1->CCR2 = compare; 
  }
  else
  {
    motor_right_mode = MOTOR_DRIVE_REVERSE;
    TIM1->CCR2 = period_counts + 1U; 
    TIM1->CCR3 = compare; 
  }
}

void Motor_BrakeAll(void)
{
  motor_left_mode = MOTOR_DRIVE_BRAKE;
  motor_right_mode = MOTOR_DRIVE_BRAKE;

  motor1_dma_up_val = 0;
  motor1_dma_cc_val = 0;
  GPIOC->BSRR = MOTOR_IN1_PIN | MOTOR_IN2_PIN; 

  TIM1->CCR2 = TIM1->ARR + 1U; 
  TIM1->CCR3 = TIM1->ARR + 1U; 
}

void Motor_ReleaseAll(void)
{
  motor_left_mode = MOTOR_DRIVE_STOP;
  motor_right_mode = MOTOR_DRIVE_STOP;

  motor1_dma_up_val = 0;
  motor1_dma_cc_val = 0;
  GPIOC->BSRR = (MOTOR_IN1_PIN | MOTOR_IN2_PIN) << 16U; 

  TIM1->CCR2 = 0; 
  TIM1->CCR3 = 0; 
}

int32_t Motor_GetLeftDeltaCount(void)
{
  return Motor_ReadLeftDeltaRaw() * MOTOR1_ENCODER_FORWARD_SIGN;
}

int32_t Motor_GetRightDeltaCount(void)
{
  return Motor_ReadRightDeltaRaw() * MOTOR2_ENCODER_FORWARD_SIGN;
}

uint8_t Motor_TakeControl1msFlag(void)
{
  uint8_t flag = motor_control_1ms_flag;
  motor_control_1ms_flag = 0U;
  return flag;
}

void Motor_TIM2_IRQHandler(void)
{
  if ((TIM2->SR & TIM_SR_UIF) != 0U)
  {
    TIM2->SR = (uint16_t)~TIM_SR_UIF;
    motor_soft_pwm_counter++;
    if (motor_soft_pwm_counter >= 20U)
    {
      motor_soft_pwm_counter = 0U;
      motor_control_1ms_flag = 1U;
    }
  }
}

static void Motor_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  
  GPIO_InitStruct.Pin = MOTOR_IN1_PIN | MOTOR_IN2_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(MOTOR_IN_GPIO_PORT, &GPIO_InitStruct);

  HAL_GPIO_WritePin(MOTOR_IN_GPIO_PORT, MOTOR_IN1_PIN | MOTOR_IN2_PIN, GPIO_PIN_RESET);
}

static void Motor2_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  
  GPIO_InitStruct.Pin = MOTOR2_IN1_PIN | MOTOR2_IN2_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
  HAL_GPIO_Init(MOTOR2_IN_GPIO_PORT, &GPIO_InitStruct);
}

static void Motor_PWM_Init(void)
{
  uint32_t timer_clk_hz;
  uint32_t period;

  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_TIM1_FORCE_RESET();
  __HAL_RCC_TIM1_RELEASE_RESET();

  timer_clk_hz = HAL_RCC_GetPCLK2Freq();
  period = timer_clk_hz / MOTOR_PWM_FREQ_HZ;
  if (period == 0U) period = 1U;

  TIM1->CR1 = 0U;
  TIM1->CR2 = 0U;
  TIM1->SMCR = 0U;
  TIM1->DIER = 0U;
  TIM1->CCER = 0U;
  TIM1->BDTR = 0U;
  TIM1->PSC = 0U;
  TIM1->ARR = (uint16_t)(period - 1U);
  TIM1->CCR2 = 0U;
  TIM1->CCR3 = 0U;
  TIM1->CCMR1 = TIM_CCMR1_OC2PE | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
  TIM1->CCMR2 = TIM_CCMR2_OC3PE | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;
  TIM1->CCER = TIM_CCER_CC2E | TIM_CCER_CC3E;
  TIM1->BDTR = TIM_BDTR_MOE;
  TIM1->EGR = TIM_EGR_UG;
  TIM1->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
}

static void Motor1_DMA_Timer_Init(void)
{
  uint32_t timer_clk_hz;
  uint32_t period;

  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_TIM2_FORCE_RESET();
  __HAL_RCC_TIM2_RELEASE_RESET();

  timer_clk_hz = HAL_RCC_GetPCLK1Freq();
  period = timer_clk_hz / MOTOR_PWM_FREQ_HZ;
  if (period == 0U) period = 1U;

  TIM2->CR1 = 0U;
  TIM2->CR2 = 0U;
  TIM2->SMCR = 0U;
  TIM2->DIER = TIM_DIER_UDE | TIM_DIER_CC1DE | TIM_DIER_UIE; 
  TIM2->CCER = 0U;
  TIM2->PSC = 0U;
  TIM2->ARR = (uint32_t)(period - 1U);
  TIM2->CCR1 = 0U;
  TIM2->CNT = 0U;
  TIM2->EGR = TIM_EGR_UG;
  TIM2->SR = 0U;

  HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);

  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMAMUX1_CLK_ENABLE();

  DMA1_Channel1->CCR = 0;
  DMA1_Channel1->CPAR = (uint32_t)&GPIOC->BSRR;
  DMA1_Channel1->CMAR = (uint32_t)&motor1_dma_up_val;
  DMA1_Channel1->CNDTR = 1;
  DMA1_Channel1->CCR = DMA_CCR_DIR | DMA_CCR_CIRC | DMA_CCR_PSIZE_1 | DMA_CCR_MSIZE_1;
  DMAMUX1_Channel0->CCR = DMA_REQUEST_TIM2_UP; 

  DMA1_Channel2->CCR = 0;
  DMA1_Channel2->CPAR = (uint32_t)&GPIOC->BSRR;
  DMA1_Channel2->CMAR = (uint32_t)&motor1_dma_cc_val;
  DMA1_Channel2->CNDTR = 1;
  DMA1_Channel2->CCR = DMA_CCR_DIR | DMA_CCR_CIRC | DMA_CCR_PSIZE_1 | DMA_CCR_MSIZE_1;
  DMAMUX1_Channel1->CCR = DMA_REQUEST_TIM2_CH1; 

  DMA1_Channel1->CCR |= DMA_CCR_EN;
  DMA1_Channel2->CCR |= DMA_CCR_EN;

  TIM2->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
}

static void Motor_Encoder_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_TIM8_CLK_ENABLE();
  __HAL_RCC_TIM8_FORCE_RESET();
  __HAL_RCC_TIM8_RELEASE_RESET();

  GPIO_InitStruct.Pin = MOTOR_ENC_A_PIN | MOTOR_ENC_B_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_TIM8;
  HAL_GPIO_Init(MOTOR_ENC_GPIO_PORT, &GPIO_InitStruct);

  TIM8->CR1 = 0U;
  TIM8->CR2 = 0U;
  TIM8->SMCR = 0U;
  TIM8->DIER = 0U;
  TIM8->CCER = 0U;
  TIM8->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
  TIM8->ARR = 0xFFFFU;
  TIM8->PSC = 0U;
  TIM8->CNT = 0U;
  TIM8->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
  TIM8->EGR = TIM_EGR_UG;
  TIM8->CR1 = TIM_CR1_CEN;

  motor_encoder_last_count = (uint16_t)TIM8->CNT;
}

static void Motor2_Encoder_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_TIM3_FORCE_RESET();
  __HAL_RCC_TIM3_RELEASE_RESET();

  GPIO_InitStruct.Pin = MOTOR2_ENC_A_PIN | MOTOR2_ENC_B_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(MOTOR2_ENC_GPIO_PORT, &GPIO_InitStruct);

  TIM3->CR1 = 0U;
  TIM3->CR2 = 0U;
  TIM3->SMCR = 0U;
  TIM3->DIER = 0U;
  TIM3->CCER = 0U;
  TIM3->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
  TIM3->ARR = 0xFFFFU;
  TIM3->PSC = 0U;
  TIM3->CNT = 0U;
  TIM3->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
  TIM3->EGR = TIM_EGR_UG;
  TIM3->CR1 = TIM_CR1_CEN;

  motor2_encoder_last_count = (uint16_t)TIM3->CNT;
}

static int32_t Motor_ReadLeftDeltaRaw(void)
{
  uint16_t current_count;
  int32_t delta_count;

  current_count = (uint16_t)TIM8->CNT;
  delta_count = (int16_t)(current_count - motor_encoder_last_count);
  motor_encoder_last_count = current_count;
  return delta_count;
}

static int32_t Motor_ReadRightDeltaRaw(void)
{
  uint16_t current_count;
  int32_t delta_count;

  current_count = (uint16_t)TIM3->CNT;
  delta_count = (int16_t)(current_count - motor2_encoder_last_count);
  motor2_encoder_last_count = current_count;
  return delta_count;
}
