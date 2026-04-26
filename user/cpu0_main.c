#include "zf_common_headfile.h"
#pragma section all "cpu0_dsram"

int core0_main(void)
{
    clock_init();
    debug_init();
    tft180_set_font(TFT180_6X8_FONT);// 设置字体
    tft180_set_dir(TFT180_CROSSWISE);// 设置横屏显示
    tft180_init();// TFT180 初始化
    mt9v03x_init();      // 摄像头初始化
    track_fusion_init(); // 赛道融合算法初始化
    // key_init_all();// 按键+拨码初始化
   small_driver_uart_init();// 无刷驱动串口通讯初始化
    //small_driver_set_duty(800, 800);
   line_pid_init();// PID控制初始化
    pit_ms_init(CCU60_CH0, 30); // 30ms 定时器周期
 //   gpio_init(P20_8, GPO, 0, GPO_PUSH_PULL);
   // gpio_init(P20_9, GPO, 0, GPO_PUSH_PULL);

    cpu_wait_event_ready();

    while (TRUE)
    {
        track_fusion_update(); // 赛道融合算法更新
        //  key_process();// 按键逻辑处理
        // menu_show();// 菜单显示-
        draw_line();
        tft180_show_int(130,0, motor_value.receive_left_speed_data,3);
        tft180_show_int(130,10,motor_value.receive_right_speed_data,3);
        printf("%d,%d\r\n",
               motor_value.receive_left_speed_data,
               motor_value.receive_right_speed_data);
   
     
    }
}

#pragma section all restore
