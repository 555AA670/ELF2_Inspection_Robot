#include "fusion_tracker.h"

#include "Fusion/FusionAhrs.c"
#include "Fusion/FusionBias.c"

void FusionTracker_Init(FusionTracker *tracker, uint32_t sample_rate_hz)
{
  FusionAhrsSettings settings;
  FusionBiasSettings bias_settings;

  if (tracker == 0)
  {
    return;
  }

  FusionAhrsInitialise(&tracker->ahrs);
  settings.convention = FusionConventionNwu;
  settings.gain = 0.5f;
  settings.gyroscopeRange = 250.0f;
  settings.accelerationRejection = 10.0f;
  settings.magneticRejection = 0.0f;
  settings.recoveryTriggerPeriod = sample_rate_hz * 5U;
  FusionAhrsSetSettings(&tracker->ahrs, &settings);

  FusionBiasInitialise(&tracker->bias);
  bias_settings = fusionBiasDefaultSettings;
  bias_settings.sampleRate = (float)sample_rate_hz;
  FusionBiasSetSettings(&tracker->bias, &bias_settings);

  tracker->euler = FUSION_EULER_ZERO;
  tracker->last_update_ms = HAL_GetTick();
}

void FusionTracker_Update(FusionTracker *tracker,
                          float gx_dps, float gy_dps, float gz_dps,
                          int32_t ax_mg, int32_t ay_mg, int32_t az_mg,
                          float delta_time_s)
{
  FusionVector gyroscope;
  FusionVector accelerometer;
  FusionVector magnetometer = FUSION_VECTOR_ZERO;

  if (tracker == 0)
  {
    return;
  }

  gyroscope.axis.x = gx_dps;
  gyroscope.axis.y = gy_dps;
  gyroscope.axis.z = gz_dps;
  gyroscope = FusionBiasUpdate(&tracker->bias, gyroscope);

  accelerometer.axis.x = (float)ax_mg / 1000.0f;
  accelerometer.axis.y = (float)ay_mg / 1000.0f;
  accelerometer.axis.z = (float)az_mg / 1000.0f;

  FusionAhrsUpdate(&tracker->ahrs, gyroscope, accelerometer, magnetometer, delta_time_s);
  tracker->euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&tracker->ahrs));
}
