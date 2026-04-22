#ifndef CODE_PI_H_
#define CODE_PI_H_

#include "zf_common_headfile.h"
#include "small_driver_uart_control.h"

typedef struct
{
    int16 target_left_speed;
    int16 target_right_speed;
    int16 current_left_speed;
    int16 current_right_speed;
    int16 left_output;
    int16 right_output;
} pi_control_struct;

extern pi_control_struct pi;

void pi_control_init(void);
void pi_set_target_speed(int16 left_speed, int16 right_speed);
void pi_control_callback(void);

#endif