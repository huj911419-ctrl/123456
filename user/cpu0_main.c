#include "zf_common_headfile.h"
#pragma section all "cpu0_dsram"
int core0_main(void)
{
    clock_init();                   // 获取时钟频率<务必保留>
    debug_init();                   // 初始化默认调试串口
    // 此处编写用户代码 例如外设初始化代码等

    small_driver_uart_init();		// 初始化驱动通讯功能
    tft180_set_dir(TFT180_CROSSWISE);
    tft180_init();
    small_driver_set_duty(300, 300);

    // 此处编写用户代码 例如外设初始化代码等
    cpu_wait_event_ready();         // 等待所有核心初始化完毕
    while (TRUE)
    {
        // 此处编写需要循环执行的代码


        printf("left speed:%d, right speed:%d\r\n", motor_value.receive_left_speed_data, motor_value.receive_right_speed_data);
        tft180_show_float(0, 0,motor_value.receive_left_speed_data, 8, 6);
        system_delay_ms(50);


        // 此处编写需要循环执行的代码
    }
}

#pragma section all restore
