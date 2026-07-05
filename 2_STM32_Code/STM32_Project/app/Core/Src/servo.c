#include "main.h"
#include "servo.h"

#define SERVO_PWM_PIN              GPIO_PIN_6
#define SERVO_PWM_GPIO_PORT        GPIOB
#define SERVO_PWM_FREQ_HZ          50U
#define SERVO_MIN_PULSE_US         500U
#define SERVO_MAX_PULSE_US         2500U
#define SERVO_MAX_ANGLE_DEG        180.0f

static float servo_current_angle_deg = SERVO_CENTER_ANGLE_DEG;
static float servo_target_angle_deg = SERVO_CENTER_ANGLE_DEG;
#define SERVO_MAX_SLEW_RATE_DEG_PER_MS 0.15f  // 150 degrees per sec. (3x the ultra-slow mode)

// Define strict mechanical bounds to prevent stalls
#define SERVO_SAFE_MIN_ANGLE_DEG   (SERVO_CENTER_ANGLE_DEG - 30.0f)
#define SERVO_SAFE_MAX_ANGLE_DEG   (SERVO_CENTER_ANGLE_DEG + 30.0f)

static void Servo_GPIO_Init(void);
static void Servo_PWM_Init(void);
static void Servo_WriteHW(float angle_deg);

void Servo_Init(void)
{
  Servo_GPIO_Init();
  Servo_PWM_Init();
  Servo_WriteHW(SERVO_CENTER_ANGLE_DEG);
}

void Servo_SetAngleDeg(float angle_deg)
{
  if (angle_deg < SERVO_SAFE_MIN_ANGLE_DEG) {
    angle_deg = SERVO_SAFE_MIN_ANGLE_DEG;
  } else if (angle_deg > SERVO_SAFE_MAX_ANGLE_DEG) {
    angle_deg = SERVO_SAFE_MAX_ANGLE_DEG;
  }
  servo_target_angle_deg = angle_deg;
}

void Servo_Update(uint32_t delta_ms)
{
  float max_step = SERVO_MAX_SLEW_RATE_DEG_PER_MS * (float)delta_ms;
  float error = servo_target_angle_deg - servo_current_angle_deg;

  if (error > max_step) {
    servo_current_angle_deg += max_step;
  } else if (error < -max_step) {
    servo_current_angle_deg -= max_step;
  } else {
    servo_current_angle_deg = servo_target_angle_deg;
  }
  
  Servo_WriteHW(servo_current_angle_deg);
}

static void Servo_WriteHW(float angle_deg)
{
  float pulse_span_us;
  float pulse_us;

  pulse_span_us = (float)(SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US);
  pulse_us = (float)SERVO_MIN_PULSE_US
           + (angle_deg / SERVO_MAX_ANGLE_DEG) * pulse_span_us;

  if (pulse_us > (float)TIM4->ARR)
  {
    pulse_us = (float)TIM4->ARR;
  }

  TIM4->CCR1 = (uint16_t)pulse_us;
}

float Servo_GetAngleDeg(void)
{
  return servo_current_angle_deg;
}

void Servo_TestSweep(void)
{
  float angle = 0.0f;
  
  /* 0 -> 180 */
  for (angle = 0.0f; angle <= 180.0f; angle += 2.0f)
  {
    Servo_SetAngleDeg(angle);
    HAL_Delay(10);
  }
  
  /* 180 -> 0 */
  for (angle = 180.0f; angle >= 0.0f; angle -= 2.0f)
  {
    Servo_SetAngleDeg(angle);
    HAL_Delay(10);
  }
  
  /* Back to center */
  Servo_SetAngleDeg(SERVO_CENTER_ANGLE_DEG);
  HAL_Delay(500);
}

static void Servo_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Pin = SERVO_PWM_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(SERVO_PWM_GPIO_PORT, &GPIO_InitStruct);
}

static void Servo_PWM_Init(void)
{
  uint32_t timer_clk_hz;
  uint32_t prescaler;
  uint32_t period;

  __HAL_RCC_TIM4_CLK_ENABLE();
  __HAL_RCC_TIM4_FORCE_RESET();
  __HAL_RCC_TIM4_RELEASE_RESET();

  timer_clk_hz = HAL_RCC_GetPCLK1Freq();
  prescaler = (timer_clk_hz / 1000000U);
  if (prescaler == 0U)
  {
    prescaler = 1U;
  }

  period = (1000000U / SERVO_PWM_FREQ_HZ);
  if (period == 0U)
  {
    period = 1U;
  }

  TIM4->CR1 = 0U;
  TIM4->CR2 = 0U;
  TIM4->SMCR = 0U;
  TIM4->DIER = 0U;
  TIM4->CCER = 0U;
  TIM4->PSC = (uint16_t)(prescaler - 1U);
  TIM4->ARR = (uint16_t)(period - 1U);
  TIM4->CCR1 = 0U;
  TIM4->CCMR1 = TIM_CCMR1_OC1PE
              | TIM_CCMR1_OC1M_1
              | TIM_CCMR1_OC1M_2;
  TIM4->CCER = TIM_CCER_CC1E;
  TIM4->EGR = TIM_EGR_UG;
  TIM4->CR1 = TIM_CR1_ARPE;

  Servo_SetAngleDeg(SERVO_CENTER_ANGLE_DEG);
  TIM4->CR1 |= TIM_CR1_CEN;
}
