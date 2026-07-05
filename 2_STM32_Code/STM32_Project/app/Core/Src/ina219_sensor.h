#ifndef INA219_SENSOR_H
#define INA219_SENSOR_H

#include "main.h"

HAL_StatusTypeDef INA219_Sensor_Init(void);
float INA219_Sensor_ReadBusVoltage_V(void);
float INA219_Sensor_ReadCurrent_A(void);
void INA219_BatteryReset(void);
uint8_t INA219_BatterySeedSocFromVoltage4S(float voltage_v);
uint8_t INA219_BatteryUpdateSoc4S(float voltage_v, float sensor_current_a);
float INA219_BatteryGetSocPercent(void);

#endif
