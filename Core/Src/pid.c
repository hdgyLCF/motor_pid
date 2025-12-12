#include "pid.h"

static float kp = 0.0f;
static float ki = 0.0f;
static float kd = 0.0f;
static float dt_global = 0.01f;
static float out_min_global = 0.0f;
static float out_max_global = 1000.0f;

static float target_rpm = 0.0f;

static float e_k_1 = 0.0f;
static float e_k_2 = 0.0f;
static float u_k_1 = 0.0f;
static float integral_sum = 0.0f;
static float filtered_rpm = 0.0f;

void PID_Init(float Kp, float Ki, float Kd, float dt, float out_min, float out_max)
{
    kp = Kp;
    ki = Ki;
    kd = Kd;
    dt_global = dt;
    out_min_global = out_min;
    out_max_global = out_max;
    e_k_1 = e_k_2 = 0.0f;
    u_k_1 = 0.0f;
    integral_sum = 0.0f;
    filtered_rpm = 0.0f;
}

void PID_Reset(void)
{
    e_k_1 = e_k_2 = 0.0f;
    u_k_1 = 0.0f;
    integral_sum = 0.0f;
    filtered_rpm = 0.0f;
}

//设置目标转速
void PID_SetTarget(float rpm)
{
    target_rpm = rpm;
}

float PID_GetTarget(void)
{
    return target_rpm;
}

//增量式PID实现
float PID_Update(float measurement_rpm)
{
    filtered_rpm = MEAS_LPF_ALPHA * measurement_rpm + (1.0f - MEAS_LPF_ALPHA) * filtered_rpm;

    float e_k = target_rpm - filtered_rpm;

    float de1 = e_k - e_k_1;
    float de2 = e_k - 2.0f*e_k_1 + e_k_2;

    integral_sum = integral_sum + e_k * dt_global;
    if (u_k_1 >= out_max_global && e_k > 0.0f) {
        integral_sum = integral_sum - e_k * dt_global;
    }
    if (u_k_1 <= out_min_global && e_k < 0.0f) {
        integral_sum = integral_sum - e_k * dt_global;
    }
    if (integral_sum > PID_I_LIMIT) integral_sum = PID_I_LIMIT;
    if (integral_sum < -PID_I_LIMIT) integral_sum = -PID_I_LIMIT;

    float du = kp * de1 + ki * integral_sum + kd * (de2 / dt_global);
    float u_k = u_k_1 + du;

    float max_change = PID_RATE_LIMIT;
    if (u_k > u_k_1 + max_change) u_k = u_k_1 + max_change;
    if (u_k < u_k_1 - max_change) u_k = u_k_1 - max_change;

    if (u_k > out_max_global) u_k = out_max_global;
    if (u_k < out_min_global) u_k = out_min_global;

    e_k_2 = e_k_1;
    e_k_1 = e_k;
    u_k_1 = u_k;

    return u_k;
}
