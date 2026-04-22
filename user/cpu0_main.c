#include "zf_common_headfile.h"
#pragma section all "cpu0_dsram"
int core0_main(void)
{
    clock_init();
    debug_init();

    //pi_control_init();
    //pi_set_target_speed(-300, 300);
    small_driver_uart_init();
    tft180_set_dir(TFT180_CROSSWISE);
    tft180_init();
    small_driver_set_duty(300, 300);

    cpu_wait_event_ready();
    while (TRUE)
    {


        //small_driver_get_speed();
        //system_delay_ms(5);
        //pi_control_callback();
        printf("left speed:%d, right speed:%d\r\n", motor_value.receive_left_speed_data, motor_value.receive_right_speed_data);
        system_delay_ms(10);


    }
}

#pragma section all restore
