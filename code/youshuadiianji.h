// Motor.h
#ifndef YOU_SHUADIAN_JI_H
#define YOU_SHUADIAN_JI_H

#include "zf_common_headfile.h"



#define MOTOR_PWM_FREQ 17000
#define MOTOR_DUTY_MAX 10000

void Motor_Init(void);
void Motor_SetSpeedr(int8_t Speed);
void Motor_SetSpeedl(int8_t Speed);

#endif
