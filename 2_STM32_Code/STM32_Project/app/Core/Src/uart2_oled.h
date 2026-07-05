#ifndef UART2_OLED_H
#define UART2_OLED_H

#include <stdint.h>
#include <stddef.h>

void UART2_OLED_Init(void);
void UART2_OLED_ProcessRx(void);
void UART2_OLED_RenderDashboard(uint8_t sensor_ready,
                                float voltage_v, float current_a, float soc_percent,
                                int32_t ax_mg, int32_t ay_mg, int32_t az_mg,
                                float gx_dps, float gy_dps, float gz_dps);
void UART2_OLED_ClearText(void);
uint32_t UART2_OLED_GetOverflowCount(void);
void USART3_IRQHandler_Callback(void);
uint8_t UART2_OLED_ReadLine(char *buffer, size_t buffer_len);
void UART2_OLED_SendString(const char *text);
void UART2_OLED_Printf(const char *format, ...);

#endif
