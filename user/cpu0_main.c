#include "zf_common_headfile.h"
#include "IMU.h"

#pragma section all "cpu0_dsram"

int core0_main(void)
{
    /* 初始化系统时钟 */
    clock_init();
    
    /* 初始化调试串口 */
    debug_init();
    
    /* 配置显示屏字体和方向 */
    tft180_set_font(TFT180_6X8_FONT);
    tft180_set_dir(TFT180_CROSSWISE);
    tft180_init();
    
    /* 初始化摄像头 */
    mt9v03x_init();
    
    /* 初始化赛道融合检测 */
    track_fusion_init();
    
    /* 初始化小车主控串口通信 */
    small_driver_uart_init();
    
    /* 初始化巡线PID参数 */
    line_pid_init();
    
    /* 初始化IMU惯性测量单元 */
    imu_init();
    
    /* 初始化30ms周期中断 */
    pit_ms_init(CCU60_CH0, 30);
    
    /* 初始化按键 */
    key_init_all();
    
    /* 等待所有核事件就绪 */
    cpu_wait_event_ready();

    while (TRUE)
    {
        /* 更新赛道融合检测结果 */
        track_fusion_update();
        
        /* 直角检测 */
        right_angle_detect();
        
        /* 计算基础速度: 电机速度值 * 20 */
        base_speed = (int16)motor_speed * 20;
        
        /* 按键处理 */
        key_process();
        
        /* 显示菜单 */
        menu_show();
    }
}

#pragma section all restore