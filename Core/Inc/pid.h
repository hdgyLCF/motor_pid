#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>

#ifndef ENCODER_PULSES_PER_REV
#define ENCODER_PULSES_PER_REV 20
#endif

#define MOTOR_DEADZONE 30.0f
#define PID_I_LIMIT 100.0f
#define PID_RATE_LIMIT 50.0f
#define MEAS_LPF_ALPHA 0.3f

/* PID init and update API */
void PID_Init(float Kp, float Ki, float Kd, float dt, float out_min, float out_max);
void PID_SetTarget(float rpm);
float PID_Update(float measurement_rpm);
float PID_GetTarget(void);
void PID_Reset(void);

#endif /* __PID_H__ */
