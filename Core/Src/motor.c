#include "motor.h"
#include "tim.h"
#include "main.h"
#include "pid.h"

void Motor_Init(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
}

void Motor_SetPWM(uint32_t compare)
{
    if (compare > 999) compare = 999;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, compare);
}

float Motor_GetRPM(void)
{
    static int16_t last_cnt = 0;
    int16_t cnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
    int16_t delta = (int16_t)(cnt - last_cnt);
    last_cnt = cnt;
    float dt = 0.01f;
    float pulses = (float)delta;
    float rpm = (pulses / (float)ENCODER_PULSES_PER_REV) * (60.0f / dt);
    return rpm;
}

void Motor_SetDirection(uint8_t forward)
{
    if (forward) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    }
}

void Motor_Stop(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
}
