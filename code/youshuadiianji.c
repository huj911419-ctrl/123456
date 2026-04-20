// Motor.c
#include "youshuadiianji.h"

void Motor_Init(void)
{
    // 方向引脚初始化，默认低电平（刹车状态）
    gpio_init(P10_5, GPO, 0, GPO_PUSH_PULL);
    gpio_init(P10_6, GPO, 0, GPO_PUSH_PULL);

    //gpio_init(P33_6, GPO, 0, GPO_PUSH_PULL);
   // gpio_init(P33_7, GPO, 0, GPO_PUSH_PULL);

    // PWM初始化，初始占空比为0
    pwm_init(ATOM0_CH1_P21_3, MOTOR_PWM_FREQ, 0);
    pwm_init(ATOM0_CH3_P21_5, MOTOR_PWM_FREQ, 0);
}


void Motor_SetSpeedr(int8_t Speed)
{
    if (Speed > 0)
    {   gpio_init(P10_6, GPO, 1, GPO_PUSH_PULL);

        //gpio_set_level(P33_6, 1);
        //gpio_set_level(P33_7, 0);
        pwm_set_duty(ATOM0_CH3_P21_5, (uint32)Speed * MOTOR_DUTY_MAX / 100);
    }
    else if (Speed < 0)
    {
        gpio_init(P10_6, GPO, 0, GPO_PUSH_PULL);
        //gpio_set_level(P33_6, 0);
        //gpio_set_level(P33_7, 1);
        pwm_set_duty(ATOM0_CH3_P21_5, (uint32)(-Speed) * MOTOR_DUTY_MAX / 100);
    }

}

void Motor_SetSpeedl(int8_t Speed)
{
    if (Speed > 0)
    {
     gpio_init(P10_5, GPO, 1, GPO_PUSH_PULL);
       // gpio_set_level(P21_3, 0);
        pwm_set_duty(ATOM0_CH1_P21_3, (uint32)Speed * MOTOR_DUTY_MAX / 100);
    }
    else if (Speed < 0)
    {
        gpio_init(P10_5, GPO, 0, GPO_PUSH_PULL);
       // gpio_set_level(P21_3, 1);
        pwm_set_duty(ATOM0_CH1_P21_3, (uint32)(-Speed) * MOTOR_DUTY_MAX / 100);
    }

}

