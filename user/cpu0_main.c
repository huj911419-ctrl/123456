#include "zf_common_headfile.h"

#pragma section all "cpu0_dsram"

int core0_main(void)
{
    clock_init();
    debug_init();
    tft180_set_font(TFT180_6X8_FONT);
    tft180_set_dir(TFT180_CROSSWISE);
    tft180_init();
    mt9v03x_init();
    track_fusion_init();
    small_driver_uart_init();
    line_pid_init();
    pit_ms_init(CCU60_CH0, 30);
    key_init_all();
    cpu_wait_event_ready();
    neg_pressure_init(50, 2000);  // 10kHz£¬50%Õ¼¿Õ±È
    while (TRUE)
    {
        track_fusion_update();
        printf("%d,%d\r\n",
               motor_value.receive_left_speed_data,
               motor_value.receive_right_speed_data);
        
        base_speed = (int16)motor_speed * 20; 
        
        key_process();
        menu_show();
    }
}

#pragma section all restore
