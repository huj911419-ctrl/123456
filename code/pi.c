#include "pi.h"

pi_control_struct pi;

#define KP 0.5
#define KI 0
#define MAX_OUTPUT 10000
#define MIN_OUTPUT -10000

void pi_control_init(void)
{
    pi.target_left_speed = 0;
    pi.target_right_speed = 0;
    pi.current_left_speed = 0;
    pi.current_right_speed = 0;
    pi.left_output = 0;
    pi.right_output = 0;
}

void pi_set_target_speed(int16 left_speed, int16 right_speed)
{
    pi.target_left_speed = left_speed;
    pi.target_right_speed = right_speed;
}

void pi_control_callback(void)
{
    static int16 left_integral = 0;
    static int16 right_integral = 0;

    pi.current_left_speed = motor_value.receive_left_speed_data;
    pi.current_right_speed = motor_value.receive_right_speed_data;

    int16 left_error = pi.target_left_speed - pi.current_left_speed;
    int16 right_error = pi.target_right_speed - pi.current_right_speed;

    left_integral += left_error;
    right_integral += right_error;

    if (left_integral > 10000) left_integral = 10000;
    if (left_integral < -10000) left_integral = -10000;
    if (right_integral > 10000) right_integral = 10000;
    if (right_integral < -10000) right_integral = -10000;

    pi.left_output = KP * left_error + KI * left_integral;
    pi.right_output = KP * right_error + KI * right_integral;

    if (pi.left_output > MAX_OUTPUT) pi.left_output = MAX_OUTPUT;
    if (pi.left_output < MIN_OUTPUT) pi.left_output = MIN_OUTPUT;
    if (pi.right_output > MAX_OUTPUT) pi.right_output = MAX_OUTPUT;
    if (pi.right_output < MIN_OUTPUT) pi.right_output = MIN_OUTPUT;

    small_driver_set_duty(pi.left_output, pi.right_output);
}
