#ifndef SERVO_H
#define SERVO_H

#define SERVO_CENTER_ANGLE_DEG 100.0f

void Servo_Init(void);
void Servo_Update(uint32_t delta_ms);
void Servo_SetAngleDeg(float angle_deg);
float Servo_GetAngleDeg(void);
void Servo_TestSweep(void);

#endif
