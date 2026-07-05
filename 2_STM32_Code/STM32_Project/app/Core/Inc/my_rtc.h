#ifndef MY_RTC_H
#define MY_RTC_H

#include "main.h"

void MyRTC_Init(void);
void MyRTC_SetTime(uint8_t hours, uint8_t minutes, uint8_t seconds);
void MyRTC_GetTime(uint8_t *hours, uint8_t *minutes, uint8_t *seconds);
void MyRTC_SetAlarmA(uint8_t hours, uint8_t minutes, uint8_t seconds);

extern volatile uint8_t rtc_alarm_triggered;
extern RTC_HandleTypeDef hrtc;

#endif /* MY_RTC_H */

