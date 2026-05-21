#include "zf_common_headfile.h"
#include "IMU.h"
#include "Menu.h"

extern int16 cam_exposure;

volatile uint32 prof_tf_us = 0u;
volatile uint32 prof_inter_us = 0u;

#pragma section all "cpu0_dsram"

int core0_main(void)
{
    clock_init();
    debug_init();

#if !RACE_MODE
    tft180_set_font(TFT180_6X8_FONT);
    tft180_set_dir(TFT180_CROSSWISE);
    tft180_init();
#endif

    mt9v03x_init();
    mt9v03x_set_exposure_time((uint16)cam_exposure);

    track_fusion_init();
    vision_share_init();

    small_driver_uart_init();
    line_pid_init();
    imu_init();

    pit_ms_init(CCU60_CH0, 11);
    key_init_all();

    cpu_wait_event_ready();

    while (TRUE)
    {
        /* CPU0 只消费 CPU1 发布的新视觉快照，PID 中断使用已经复制好的稳定数据。 */
        if (vision_apply_latest())
        {
            key_process();
            menu_show();
        }
    }
}

#pragma section all restore
