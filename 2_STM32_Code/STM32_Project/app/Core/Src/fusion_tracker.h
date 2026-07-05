#ifndef FUSION_TRACKER_H
#define FUSION_TRACKER_H

#include "main.h"
#include "Fusion/FusionAhrs.h"
#include "Fusion/FusionBias.h"

typedef struct
{
  FusionAhrs ahrs;
  FusionBias bias;
  FusionEuler euler;
  uint32_t last_update_ms;
} FusionTracker;

void FusionTracker_Init(FusionTracker *tracker, uint32_t sample_rate_hz);
void FusionTracker_Update(FusionTracker *tracker,
                          float gx_dps, float gy_dps, float gz_dps,
                          int32_t ax_mg, int32_t ay_mg, int32_t az_mg,
                          float delta_time_s);

#endif
