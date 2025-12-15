#include "pid.h"

void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float dt, float out_min, float out_max)
{
    pid->kp = Kp;
    pid->ki = Ki;
    pid->kd = Kd;
    pid->dt = dt;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->target = 0.0f;
    pid->e_k_1 = 0.0f;
    pid->e_k_2 = 0.0f;
    pid->u_k_1 = 0.0f;
    pid->integral_sum = 0.0f;
    pid->filtered_meas = 0.0f;
}

void PID_Reset(PID_Controller *pid)
{
    pid->e_k_1 = 0.0f;
    pid->e_k_2 = 0.0f;
    pid->u_k_1 = 0.0f;
    pid->integral_sum = 0.0f;
    pid->filtered_meas = 0.0f;
}

//设置目标
void PID_SetTarget(PID_Controller *pid, float target)
{
    pid->target = target;
}

float PID_GetTarget(PID_Controller *pid)
{
    return pid->target;
}


float PID_Update(PID_Controller *pid, float measurement)
{
    pid->filtered_meas = MEAS_LPF_ALPHA * measurement + (1.0f - MEAS_LPF_ALPHA) * pid->filtered_meas;

    float e_k = pid->target - pid->filtered_meas;

    float de1 = e_k - pid->e_k_1;
    float de2 = e_k - 2.0f * pid->e_k_1 + pid->e_k_2;

    pid->integral_sum = pid->integral_sum + e_k * pid->dt;
    if (pid->u_k_1 >= pid->out_max && e_k > 0.0f) {
        pid->integral_sum = pid->integral_sum - e_k * pid->dt;
    }
    if (pid->u_k_1 <= pid->out_min && e_k < 0.0f) {
        pid->integral_sum = pid->integral_sum - e_k * pid->dt;
    }
    if (pid->integral_sum > PID_I_LIMIT) pid->integral_sum = PID_I_LIMIT;
    if (pid->integral_sum < -PID_I_LIMIT) pid->integral_sum = -PID_I_LIMIT;

    float du = pid->kp * de1 + pid->ki * pid->integral_sum + pid->kd * (de2 / pid->dt);
    float u_k = pid->u_k_1 + du;

    float max_change = PID_RATE_LIMIT;
    if (u_k > pid->u_k_1 + max_change) u_k = pid->u_k_1 + max_change;
    if (u_k < pid->u_k_1 - max_change) u_k = pid->u_k_1 - max_change;

    if (u_k > pid->out_max) u_k = pid->out_max;
    if (u_k < pid->out_min) u_k = pid->out_min;

    pid->e_k_2 = pid->e_k_1;
    pid->e_k_1 = e_k;
    pid->u_k_1 = u_k;

    return u_k;
}
