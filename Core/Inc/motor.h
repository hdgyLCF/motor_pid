#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <stdint.h>

void Motor_Init(void);
void Motor_SetPWM(uint32_t compare);
float Motor_GetRPM(void);
void Motor_SetDirection(uint8_t forward);
void Motor_Stop(void);

#endif /* __MOTOR_H__ */
