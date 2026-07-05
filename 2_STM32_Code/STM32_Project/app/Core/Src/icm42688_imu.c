#include "icm42688_imu.h"

#define ICM42688_GPIO_PORT           GPIOA
#define ICM42688_CS_PIN              GPIO_PIN_4
#define ICM42688_SCK_PIN             GPIO_PIN_5
#define ICM42688_MISO_PIN            GPIO_PIN_6
#define ICM42688_MOSI_PIN            GPIO_PIN_7
#define ICM42688_REG_DEVICE_CONFIG   0x11
#define ICM42688_REG_ACCEL_DATA_X1   0x1F
#define ICM42688_REG_GYRO_DATA_X1    0x25
#define ICM42688_REG_PWR_MGMT0       0x4E
#define ICM42688_REG_GYRO_CONFIG0    0x4F
#define ICM42688_REG_ACCEL_CONFIG0   0x50
#define ICM42688_REG_WHO_AM_I        0x75
#define ICM42688_REG_BANK_SEL        0x76
#define ICM42688_WHO_AM_I_VALUE      0x47
#define ICM42688_SOFT_RESET          0x01
#define ICM42688_SPI_TIMEOUT_MS      5U
#define ICM42688_ACCEL_CONFIG_2G     0x66
#define ICM42688_GYRO_CONFIG_250DPS  0x66
#define ICM42688_IMU_MODE_LN         0x0F
#define ICM42688_ACCEL_SCALE_2G_MG   16.384f
#define ICM42688_GYRO_SCALE_250DPS   131.0f

static void ICM42688_GPIO_Init(void);
static void ICM42688_SPI1_Init(void);
static void ICM42688_CS_Write(uint8_t state);
static HAL_StatusTypeDef ICM42688_SPI_WaitFlag(uint32_t flag, uint32_t state);
static HAL_StatusTypeDef ICM42688_SPI_Transfer(uint8_t tx_data, uint8_t *rx_data);
static HAL_StatusTypeDef ICM42688_WriteRegister(uint8_t reg, uint8_t value);
static HAL_StatusTypeDef ICM42688_ReadRegister(uint8_t reg, uint8_t *value);
static HAL_StatusTypeDef ICM42688_ReadRegisters(uint8_t reg, uint8_t *buffer, uint8_t length);
static HAL_StatusTypeDef ICM42688_ReadAccelRaw(int16_t *ax, int16_t *ay, int16_t *az);
static HAL_StatusTypeDef ICM42688_ReadGyroRaw(int16_t *gx, int16_t *gy, int16_t *gz);

HAL_StatusTypeDef ICM42688_IMU_Init(uint8_t *who_am_i)
{
  uint8_t device_id = 0U;

  ICM42688_GPIO_Init();
  ICM42688_SPI1_Init();

  if (ICM42688_WriteRegister(ICM42688_REG_DEVICE_CONFIG, ICM42688_SOFT_RESET) != HAL_OK)
  {
    return HAL_ERROR;
  }
  HAL_Delay(10);

  if (ICM42688_WriteRegister(ICM42688_REG_BANK_SEL, 0x00U) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (ICM42688_ReadRegister(ICM42688_REG_WHO_AM_I, &device_id) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (who_am_i != 0)
  {
    *who_am_i = device_id;
  }

  if (device_id != ICM42688_WHO_AM_I_VALUE)
  {
    return HAL_ERROR;
  }

  if (ICM42688_WriteRegister(ICM42688_REG_PWR_MGMT0, ICM42688_IMU_MODE_LN) != HAL_OK)
  {
    return HAL_ERROR;
  }
  HAL_Delay(50);

  if (ICM42688_WriteRegister(ICM42688_REG_ACCEL_CONFIG0, ICM42688_ACCEL_CONFIG_2G) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (ICM42688_WriteRegister(ICM42688_REG_GYRO_CONFIG0, ICM42688_GYRO_CONFIG_250DPS) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

HAL_StatusTypeDef ICM42688_IMU_ReadAccelMg(int32_t *ax_mg, int32_t *ay_mg, int32_t *az_mg)
{
  int16_t raw_x;
  int16_t raw_y;
  int16_t raw_z;

  if ((ax_mg == 0) || (ay_mg == 0) || (az_mg == 0))
  {
    return HAL_ERROR;
  }

  if (ICM42688_ReadAccelRaw(&raw_x, &raw_y, &raw_z) != HAL_OK)
  {
    return HAL_ERROR;
  }

  *ax_mg = (int32_t)((float)raw_x / ICM42688_ACCEL_SCALE_2G_MG);
  *ay_mg = (int32_t)((float)raw_y / ICM42688_ACCEL_SCALE_2G_MG);
  *az_mg = (int32_t)((float)raw_z / ICM42688_ACCEL_SCALE_2G_MG);
  return HAL_OK;
}

HAL_StatusTypeDef ICM42688_IMU_ReadGyroDps(float *gx_dps, float *gy_dps, float *gz_dps)
{
  int16_t raw_x;
  int16_t raw_y;
  int16_t raw_z;

  if ((gx_dps == 0) || (gy_dps == 0) || (gz_dps == 0))
  {
    return HAL_ERROR;
  }

  if (ICM42688_ReadGyroRaw(&raw_x, &raw_y, &raw_z) != HAL_OK)
  {
    return HAL_ERROR;
  }

  *gx_dps = (float)raw_x / ICM42688_GYRO_SCALE_250DPS;
  *gy_dps = (float)raw_y / ICM42688_GYRO_SCALE_250DPS;
  *gz_dps = (float)raw_z / ICM42688_GYRO_SCALE_250DPS;
  return HAL_OK;
}

HAL_StatusTypeDef ICM42688_IMU_CalibrateGyroBias(uint32_t sample_count, ICM42688_GyroBias *bias)
{
  uint32_t i;
  float sum_x = 0.0f;
  float sum_y = 0.0f;
  float sum_z = 0.0f;

  if ((sample_count == 0U) || (bias == 0))
  {
    return HAL_ERROR;
  }

  for (i = 0U; i < sample_count; i++)
  {
    float gx_dps;
    float gy_dps;
    float gz_dps;

    if (ICM42688_IMU_ReadGyroDps(&gx_dps, &gy_dps, &gz_dps) != HAL_OK)
    {
      return HAL_ERROR;
    }

    sum_x += gx_dps;
    sum_y += gy_dps;
    sum_z += gz_dps;
    HAL_Delay(5);
  }

  bias->x_dps = sum_x / (float)sample_count;
  bias->y_dps = sum_y / (float)sample_count;
  bias->z_dps = sum_z / (float)sample_count;
  return HAL_OK;
}

static void ICM42688_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitStruct.Pin = ICM42688_CS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(ICM42688_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = ICM42688_SCK_PIN | ICM42688_MISO_PIN | ICM42688_MOSI_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(ICM42688_GPIO_PORT, &GPIO_InitStruct);

  ICM42688_CS_Write(1);
  HAL_Delay(10);
}

static void ICM42688_SPI1_Init(void)
{
  __HAL_RCC_SPI1_CLK_ENABLE();
  __HAL_RCC_SPI1_FORCE_RESET();
  __HAL_RCC_SPI1_RELEASE_RESET();

  SPI1->CR1 = 0U;
  SPI1->CR2 = 0U;
  SPI1->CR1 = SPI_CR1_MSTR
            | SPI_CR1_SSM
            | SPI_CR1_SSI
            | SPI_CR1_BR_0;
  SPI1->CR2 = SPI_CR2_DS_2
            | SPI_CR2_DS_1
            | SPI_CR2_DS_0
            | SPI_CR2_FRXTH;
  SPI1->CR1 |= SPI_CR1_SPE;
}

static void ICM42688_CS_Write(uint8_t state)
{
  HAL_GPIO_WritePin(ICM42688_GPIO_PORT, ICM42688_CS_PIN, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
  __NOP();
  __NOP();
}

static HAL_StatusTypeDef ICM42688_SPI_WaitFlag(uint32_t flag, uint32_t state)
{
  uint32_t start_tick = HAL_GetTick();

  while (((SPI1->SR & flag) != 0U) != (state != 0U))
  {
    if ((HAL_GetTick() - start_tick) >= ICM42688_SPI_TIMEOUT_MS)
    {
      return HAL_TIMEOUT;
    }
  }

  return HAL_OK;
}

static HAL_StatusTypeDef ICM42688_SPI_Transfer(uint8_t tx_data, uint8_t *rx_data)
{
  if (rx_data == 0)
  {
    return HAL_ERROR;
  }

  if (ICM42688_SPI_WaitFlag(SPI_SR_TXE, 1U) != HAL_OK)
  {
    return HAL_TIMEOUT;
  }
  *(__IO uint8_t *)&SPI1->DR = tx_data;

  if (ICM42688_SPI_WaitFlag(SPI_SR_RXNE, 1U) != HAL_OK)
  {
    return HAL_TIMEOUT;
  }
  *rx_data = *(__IO uint8_t *)&SPI1->DR;

  return HAL_OK;
}

static HAL_StatusTypeDef ICM42688_WriteRegister(uint8_t reg, uint8_t value)
{
  uint8_t dummy;
  HAL_StatusTypeDef status;

  ICM42688_CS_Write(0);
  status = ICM42688_SPI_Transfer((uint8_t)(reg & 0x7FU), &dummy);
  if (status == HAL_OK)
  {
    status = ICM42688_SPI_Transfer(value, &dummy);
  }
  if (status == HAL_OK)
  {
    status = ICM42688_SPI_WaitFlag(SPI_SR_BSY, 0U);
  }
  ICM42688_CS_Write(1);

  return status;
}

static HAL_StatusTypeDef ICM42688_ReadRegister(uint8_t reg, uint8_t *value)
{
  uint8_t dummy;
  HAL_StatusTypeDef status;

  if (value == 0)
  {
    return HAL_ERROR;
  }

  ICM42688_CS_Write(0);
  status = ICM42688_SPI_Transfer((uint8_t)(reg | 0x80U), &dummy);
  if (status == HAL_OK)
  {
    status = ICM42688_SPI_Transfer(0x00U, value);
  }
  if (status == HAL_OK)
  {
    status = ICM42688_SPI_WaitFlag(SPI_SR_BSY, 0U);
  }
  ICM42688_CS_Write(1);

  return status;
}

static HAL_StatusTypeDef ICM42688_ReadRegisters(uint8_t reg, uint8_t *buffer, uint8_t length)
{
  uint8_t i;
  uint8_t dummy;
  HAL_StatusTypeDef status;

  if ((buffer == 0) || (length == 0U))
  {
    return HAL_ERROR;
  }

  ICM42688_CS_Write(0);
  status = ICM42688_SPI_Transfer((uint8_t)(reg | 0x80U), &dummy);
  for (i = 0; (i < length) && (status == HAL_OK); i++)
  {
    status = ICM42688_SPI_Transfer(0x00U, &buffer[i]);
  }
  if (status == HAL_OK)
  {
    status = ICM42688_SPI_WaitFlag(SPI_SR_BSY, 0U);
  }
  ICM42688_CS_Write(1);

  return status;
}

static HAL_StatusTypeDef ICM42688_ReadAccelRaw(int16_t *ax, int16_t *ay, int16_t *az)
{
  uint8_t data[6];

  if ((ax == 0) || (ay == 0) || (az == 0))
  {
    return HAL_ERROR;
  }

  if (ICM42688_ReadRegisters(ICM42688_REG_ACCEL_DATA_X1, data, 6) != HAL_OK)
  {
    return HAL_ERROR;
  }

  *ax = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
  *ay = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
  *az = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
  return HAL_OK;
}

static HAL_StatusTypeDef ICM42688_ReadGyroRaw(int16_t *gx, int16_t *gy, int16_t *gz)
{
  uint8_t data[6];

  if ((gx == 0) || (gy == 0) || (gz == 0))
  {
    return HAL_ERROR;
  }

  if (ICM42688_ReadRegisters(ICM42688_REG_GYRO_DATA_X1, data, 6) != HAL_OK)
  {
    return HAL_ERROR;
  }

  *gx = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
  *gy = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
  *gz = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
  return HAL_OK;
}
