#ifndef ICM42688_IMU_H
#define ICM42688_IMU_H

#include "main.h"

typedef struct
{
  float x_dps;
  float y_dps;
  float z_dps;
} ICM42688_GyroBias;

HAL_StatusTypeDef ICM42688_IMU_Init(uint8_t *who_am_i);
HAL_StatusTypeDef ICM42688_IMU_ReadAccelMg(int32_t *ax_mg, int32_t *ay_mg, int32_t *az_mg);
HAL_StatusTypeDef ICM42688_IMU_ReadGyroDps(float *gx_dps, float *gy_dps, float *gz_dps);
HAL_StatusTypeDef ICM42688_IMU_CalibrateGyroBias(uint32_t sample_count, ICM42688_GyroBias *bias);

#endif
