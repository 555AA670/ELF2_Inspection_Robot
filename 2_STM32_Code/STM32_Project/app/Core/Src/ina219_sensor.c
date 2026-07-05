#include "ina219_sensor.h"

#define INA219_SDA_PIN               GPIO_PIN_0
#define INA219_SCL_PIN               GPIO_PIN_1
#define INA219_GPIO_PORT             GPIOC
#define INA219_I2C_ADDRESS           0x44
#define INA219_REG_CONFIG            0x00
#define INA219_REG_SHUNT_VOLTAGE     0x01
#define INA219_REG_BUS_VOLTAGE       0x02
#define INA219_CONFIG_DEFAULT        0x3FFF
#define INA219_SHUNT_RESISTOR_OHM    0.005f
#define BATTERY_POINT_COUNT          12U
#define BATTERY_CAPACITY_AH          2.2f
#define BATTERY_IDLE_CURRENT_A       0.05f
#define BATTERY_INTERNAL_RESISTANCE_OHM 0.035f
#define BATTERY_COMP_FILTER_GAIN_ACTIVE 0.0001f
#define BATTERY_COMP_FILTER_GAIN_IDLE   0.01f
#define BATTERY_MAX_UPDATE_MS        2000U

static uint8_t ina219_address = 0U;
static uint8_t battery_soc_valid = 0U;
static uint32_t battery_last_update_ms = 0U;
static float battery_soc_percent = 0.0f;
static float battery_remaining_ah = 0.0f;

static const float battery_voltage_table[BATTERY_POINT_COUNT] = {
  14.00f, 14.43f, 14.75f, 14.91f, 15.06f, 15.18f,
  15.34f, 15.50f, 15.81f, 16.09f, 16.45f, 16.80f
};
static const uint8_t battery_percent_table[BATTERY_POINT_COUNT] = {
  0U, 5U, 10U, 20U, 30U, 40U, 50U, 60U, 70U, 80U, 90U, 100U
};

static void INA219_GPIO_Init(void);
static void INA219_Delay(void);
static void INA219_SDA_Write(uint8_t state);
static void INA219_SCL_Write(uint8_t state);
static uint8_t INA219_SDA_Read(void);
static void INA219_I2C_Start(void);
static void INA219_I2C_Stop(void);
static uint8_t INA219_I2C_WriteByte(uint8_t data);
static uint8_t INA219_I2C_ReadByte(uint8_t ack);
static HAL_StatusTypeDef INA219_WriteRegister(uint8_t reg, uint16_t value);
static HAL_StatusTypeDef INA219_ReadRegister(uint8_t reg, uint16_t *value);
static float Battery_GetSocFromVoltage_4S(float voltage_v);

HAL_StatusTypeDef INA219_Sensor_Init(void)
{
  INA219_GPIO_Init();
  ina219_address = INA219_I2C_ADDRESS;
  INA219_BatteryReset();
  return INA219_WriteRegister(INA219_REG_CONFIG, INA219_CONFIG_DEFAULT);
}

float INA219_Sensor_ReadBusVoltage_V(void)
{
  uint16_t reg;

  if (INA219_ReadRegister(INA219_REG_BUS_VOLTAGE, &reg) != HAL_OK)
  {
    return 0.0f;
  }

  reg >>= 3;
  return (float)reg * 0.004f;
}

float INA219_Sensor_ReadCurrent_A(void)
{
  uint16_t reg;
  int16_t shunt_raw;
  float shunt_voltage_v;

  if (INA219_ReadRegister(INA219_REG_SHUNT_VOLTAGE, &reg) != HAL_OK)
  {
    return 0.0f;
  }

  shunt_raw = (int16_t)reg;
  shunt_voltage_v = (float)shunt_raw * 0.00001f;
  return shunt_voltage_v / INA219_SHUNT_RESISTOR_OHM;
}

void INA219_BatteryReset(void)
{
  battery_soc_valid = 0U;
  battery_last_update_ms = 0U;
  battery_soc_percent = 0.0f;
  battery_remaining_ah = 0.0f;
}

uint8_t INA219_BatterySeedSocFromVoltage4S(float voltage_v)
{
  battery_soc_percent = Battery_GetSocFromVoltage_4S(voltage_v);
  battery_remaining_ah = (battery_soc_percent * BATTERY_CAPACITY_AH) / 100.0f;
  battery_soc_valid = 1U;
  battery_last_update_ms = HAL_GetTick();
  return (uint8_t)(battery_soc_percent + 0.5f);
}

uint8_t INA219_BatteryUpdateSoc4S(float voltage_v, float sensor_current_a)
{
  float discharge_current_a;
  uint32_t now_ms;

  discharge_current_a = (sensor_current_a >= 0.0f) ? sensor_current_a : -sensor_current_a;
  now_ms = HAL_GetTick();

  if (!battery_soc_valid)
  {
    return INA219_BatterySeedSocFromVoltage4S(voltage_v);
  }

  if (now_ms > battery_last_update_ms)
  {
    uint32_t delta_ms = now_ms - battery_last_update_ms;
    float delta_s, delta_h;
    float comp_voltage_v;
    float voltage_soc;
    float filter_gain;
    float correction_percent;

    if (delta_ms > BATTERY_MAX_UPDATE_MS)
    {
      delta_ms = BATTERY_MAX_UPDATE_MS;
    }

    delta_s = (float)delta_ms / 1000.0f;
    delta_h = (float)delta_ms / 3600000.0f;
    battery_last_update_ms = now_ms;

    if (discharge_current_a >= BATTERY_IDLE_CURRENT_A)
    {
      battery_remaining_ah -= discharge_current_a * delta_h;
    }

    battery_soc_percent = (battery_remaining_ah / BATTERY_CAPACITY_AH) * 100.0f;

    comp_voltage_v = voltage_v + (discharge_current_a * BATTERY_INTERNAL_RESISTANCE_OHM);
    voltage_soc = Battery_GetSocFromVoltage_4S(comp_voltage_v);

    if (discharge_current_a < BATTERY_IDLE_CURRENT_A)
    {
      filter_gain = BATTERY_COMP_FILTER_GAIN_IDLE * delta_s;
    }
    else
    {
      filter_gain = BATTERY_COMP_FILTER_GAIN_ACTIVE * delta_s;
    }

    correction_percent = (voltage_soc - battery_soc_percent) * filter_gain;

    // Removed clamp to allow the complementary filter to work symmetrically

    battery_soc_percent += correction_percent;

    if (battery_soc_percent < 0.0f)
    {
      battery_soc_percent = 0.0f;
    }
    else if (battery_soc_percent > 100.0f)
    {
      battery_soc_percent = 100.0f;
    }

    battery_remaining_ah = (battery_soc_percent * BATTERY_CAPACITY_AH) / 100.0f;
  }

  return (uint8_t)(battery_soc_percent + 0.5f);
}

float INA219_BatteryGetSocPercent(void)
{
  return battery_soc_percent;
}

static void INA219_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitStruct.Pin = INA219_SDA_PIN | INA219_SCL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(INA219_GPIO_PORT, &GPIO_InitStruct);

  INA219_SDA_Write(1);
  INA219_SCL_Write(1);
  HAL_Delay(10);
}

static void INA219_Delay(void)
{
  volatile uint32_t i;

  for (i = 0; i < 60; i++)
  {
    __NOP();
  }
}

static void INA219_SDA_Write(uint8_t state)
{
  HAL_GPIO_WritePin(INA219_GPIO_PORT, INA219_SDA_PIN, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
  INA219_Delay();
}

static void INA219_SCL_Write(uint8_t state)
{
  HAL_GPIO_WritePin(INA219_GPIO_PORT, INA219_SCL_PIN, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
  INA219_Delay();
}

static uint8_t INA219_SDA_Read(void)
{
  return (uint8_t)(HAL_GPIO_ReadPin(INA219_GPIO_PORT, INA219_SDA_PIN) == GPIO_PIN_SET);
}

static void INA219_I2C_Start(void)
{
  INA219_SDA_Write(1);
  INA219_SCL_Write(1);
  INA219_SDA_Write(0);
  INA219_SCL_Write(0);
}

static void INA219_I2C_Stop(void)
{
  INA219_SDA_Write(0);
  INA219_SCL_Write(1);
  INA219_SDA_Write(1);
}

static uint8_t INA219_I2C_WriteByte(uint8_t data)
{
  uint8_t i;
  uint8_t ack;

  for (i = 0; i < 8; i++)
  {
    INA219_SDA_Write((data & 0x80U) != 0U);
    INA219_SCL_Write(1);
    INA219_SCL_Write(0);
    data <<= 1;
  }

  INA219_SDA_Write(1);
  INA219_SCL_Write(1);
  ack = (uint8_t)!INA219_SDA_Read();
  INA219_SCL_Write(0);

  return ack;
}

static uint8_t INA219_I2C_ReadByte(uint8_t ack)
{
  uint8_t i;
  uint8_t data = 0;

  INA219_SDA_Write(1);
  for (i = 0; i < 8; i++)
  {
    data <<= 1;
    INA219_SCL_Write(1);
    if (INA219_SDA_Read())
    {
      data |= 0x01U;
    }
    INA219_SCL_Write(0);
  }

  INA219_SDA_Write(ack ? 0 : 1);
  INA219_SCL_Write(1);
  INA219_SCL_Write(0);
  INA219_SDA_Write(1);

  return data;
}

static HAL_StatusTypeDef INA219_WriteRegister(uint8_t reg, uint16_t value)
{
  if (ina219_address == 0U)
  {
    return HAL_ERROR;
  }

  INA219_I2C_Start();
  if (!INA219_I2C_WriteByte((uint8_t)(ina219_address << 1)))
  {
    INA219_I2C_Stop();
    return HAL_ERROR;
  }
  if (!INA219_I2C_WriteByte(reg))
  {
    INA219_I2C_Stop();
    return HAL_ERROR;
  }
  if (!INA219_I2C_WriteByte((uint8_t)(value >> 8)))
  {
    INA219_I2C_Stop();
    return HAL_ERROR;
  }
  if (!INA219_I2C_WriteByte((uint8_t)(value & 0xFFU)))
  {
    INA219_I2C_Stop();
    return HAL_ERROR;
  }
  INA219_I2C_Stop();

  return HAL_OK;
}

static HAL_StatusTypeDef INA219_ReadRegister(uint8_t reg, uint16_t *value)
{
  uint8_t msb;
  uint8_t lsb;

  if ((ina219_address == 0U) || (value == 0))
  {
    return HAL_ERROR;
  }

  INA219_I2C_Start();
  if (!INA219_I2C_WriteByte((uint8_t)(ina219_address << 1)))
  {
    INA219_I2C_Stop();
    return HAL_ERROR;
  }
  if (!INA219_I2C_WriteByte(reg))
  {
    INA219_I2C_Stop();
    return HAL_ERROR;
  }

  INA219_I2C_Start();
  if (!INA219_I2C_WriteByte((uint8_t)((ina219_address << 1) | 0x01U)))
  {
    INA219_I2C_Stop();
    return HAL_ERROR;
  }
  msb = INA219_I2C_ReadByte(1);
  lsb = INA219_I2C_ReadByte(0);
  INA219_I2C_Stop();

  *value = (uint16_t)(((uint16_t)msb << 8) | lsb);
  return HAL_OK;
}

static float Battery_GetSocFromVoltage_4S(float voltage_v)
{
  uint32_t i;

  if (voltage_v <= battery_voltage_table[0])
  {
    return (float)battery_percent_table[0];
  }

  if (voltage_v >= battery_voltage_table[BATTERY_POINT_COUNT - 1U])
  {
    return (float)battery_percent_table[BATTERY_POINT_COUNT - 1U];
  }

  for (i = 0U; i < (BATTERY_POINT_COUNT - 1U); i++)
  {
    float v0 = battery_voltage_table[i];
    float v1 = battery_voltage_table[i + 1U];

    if ((voltage_v >= v0) && (voltage_v <= v1))
    {
      float p0 = (float)battery_percent_table[i];
      float p1 = (float)battery_percent_table[i + 1U];
      float ratio = (voltage_v - v0) / (v1 - v0);
      return p0 + ratio * (p1 - p0);
    }
  }

  return 0.0f;
}
