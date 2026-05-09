#include "zf_common_headfile.h"
#include "IMU.h"
#include "Menu.h"

#pragma section all "cpu0_dsram"

int core0_main(void)
{
    /* 初始化系统时钟 */
    clock_init();

    /* 初始化调试串口 */
    debug_init();

    /* 配置显示字体和方向 */
    tft180_set_font(TFT180_6X8_FONT);
    tft180_set_dir(TFT180_CROSSWISE);
    tft180_init();

    /* 初始化摄像头 */
    mt9v03x_init();
    mt9v03x_set_exposure_time((uint16)cam_exposure);

    /* 初始化赛道融合检测 */
    track_fusion_init();

    /* 初始化小车主控串口通讯 */
    small_driver_uart_init();

    /* 初始化增量式PID参数 */
    line_pid_init();

    /* 初始化IMU角度测量单元 */

   // imu_init();

    /* 初始化11ms周期中断 */
    pit_ms_init(CCU60_CH0, 11);

    /* 初始化按键 */
    key_init_all();

    /* 等待所有核事件就绪 */
    cpu_wait_event_ready();

    while (TRUE)
    {
        /* 更新赛道融合检测结果 */
        track_fusion_update();

        /* 直角预判检测（基于当前赛道二值图，必须在 track_fusion_update 之后） */
        right_angle_pre_detect();

        /* 拐点法路口检测 */
        detect_intersection();

        // base_speed 直接由菜单设置，不再自动计算
        // base_speed = (int16)motor_speed * 20;

        /* 按键处理 */
        key_process();

        /* 显示菜单 */
        menu_show();
    }
}

#pragma section all restore