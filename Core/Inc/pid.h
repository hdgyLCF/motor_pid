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

typedef struct {
    float kp;
    float ki;
    float kd;
    float dt;
    float out_min;
    float out_max;
    float target;
    float e_k_1;
    float e_k_2;
    float u_k_1;
    float integral_sum;
    float filtered_meas;
} PID_Controller;

/* PID init and update API */
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float dt, float out_min, float out_max);
void PID_SetTarget(PID_Controller *pid, float target);
float PID_Update(PID_Controller *pid, float measurement);
float PID_GetTarget(PID_Controller *pid);
void PID_Reset(PID_Controller *pid);

#endif /* __PID_H__ */
