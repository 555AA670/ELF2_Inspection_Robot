#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"

#define MOTOR_LEFT_ENCODER_COUNTS_PER_REV   1565.0f
#define MOTOR_RIGHT_ENCODER_COUNTS_PER_REV  1565.0f

void Motor_Init(void);
void Motor_SetLeftDutyPercent(int8_t duty_percent);
void Motor_SetRightDutyPercent(int8_t duty_percent);
void Motor_BrakeAll(void);
void Motor_ReleaseAll(void);
int32_t Motor_GetLeftDeltaCount(void);
int32_t Motor_GetRightDeltaCount(void);
void Motor_TIM2_IRQHandler(void);
uint8_t Motor_TakeControl1msFlag(void);

#endif
