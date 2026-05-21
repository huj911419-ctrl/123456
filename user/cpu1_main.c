#include "zf_common_headfile.h"

#pragma section all "cpu1_dsram"
/* CPU1 专门做视觉处理，工作快照放在 CPU1 DSRAM。 */
static VisionSnapshot_t s_cpu1_vision_work;

void core1_main(void)
{
    disable_Watchdog();
    interrupt_global_enable(0);

    cpu_wait_event_ready();

    vision_set_work_snapshot(&s_cpu1_vision_work);
    track_fusion_init();

    while (TRUE)
    {
        uint32 tf_us;
        uint32 inter_us;

        while (!mt9v03x_finish_flag);
        mt9v03x_finish_flag = 0u;

        /* CPU0 如果已经消费路口标志，CPU1 同步清除工作区标志。 */
        vision_sync_cpu0_ack();

        system_start();
        track_fusion_update();
        tf_us = system_getval_us();

        system_start();
        right_angle_pre_detect();
        detect_intersection();
        inter_us = system_getval_us();

        vision_publish_from_work(&s_cpu1_vision_work, tf_us, inter_us);
    }
}

#pragma section all restore
