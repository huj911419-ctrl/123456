#include "zf_common_headfile.h"
#include "IMU.h"
#include "Menu.h"
#include "ImageTransfer.h"
#include "Pid.h"
#include "Battery.h"
#include "AutoTuneLog.h"

/* 摄像头曝光时间变量（由菜单调节），控制图像亮度 */
extern int16 cam_exposure;

/* ========================================================================
 * 性能分析变量
 * ========================================================================
 * 用于测量关键函数的执行时间（微秒），帮助优化实时性：
 *   prof_tf_us：track_fusion_update() 的执行耗时
 *   prof_inter_us：right_angle_pre_detect() + detect_intersection() 的执行耗时
 * 在TFT屏幕上显示，方便调试时评估CPU负载。
 * ======================================================================== */
volatile uint32 prof_tf_us = 0u;            /* 视觉处理耗时（微秒） */
volatile uint32 prof_inter_us = 0u;         /* 路口检测耗时（微秒） */

/* 将CPU0的所有代码和数据分配到CPU0的本地RAM中，提高访问速度 */
#pragma section all "cpu0_dsram"

/**
 * @brief CPU0主函数 - 系统入口点
 *
 * TC264D双核系统的主控核入口。执行完整的系统初始化和主循环：
 *
 * 初始化流程：
 *   1. 系统时钟配置（CPU、PLL、SPB频率）
 *   2. 调试串口初始化
 *   3. TFT显示屏初始化（非比赛模式）
 *   4. 摄像头初始化（MT9V03X，188x120分辨率）
 *   5. 赛道融合检测模块初始化
 *   6. 电机驱动串口初始化（UART3，460800波特率）
 *   7. PID控制器初始化
 *   8. 负压PWM初始化
 *   9. IMU初始化（IMU660RC陀螺仪，含零偏校准）
 *  10. PID周期定时器初始化（CCU60_CH0，8ms）
 *  11. 按键GPIO初始化
 *  12. 电池电压监测初始化
 *  13. 等待所有CPU核心就绪
 *
 * 主循环流程（每帧）：
 *   1. 等待摄像头新帧完成（mt9v03x_finish_flag）
 *   2. 赛道融合处理：压缩 -> Otsu -> 二值化 -> 降噪 -> 边缘扫描 -> 误差计算
 *   3. 直角预检测：远场边缘丢失检测，用于提前减速
 *   4. 路口检测：拐点扫描 + 检测框 + 类型分类
 *   5. 发送调试数据到PC（非比赛模式）
 *   6. 按键处理和菜单显示
 */
int core0_main(void)
{
    uint8 battery_frame_div = 0u;

    /* ---- 系统初始化阶段 ---- */

    /* 初始化系统时钟：配置CPU=200MHz, PLL=320MHz, SPB=100MHz */
    clock_init();

    /* 初始化调试串口（UART0），用于printf输出和图像传输 */
    debug_init();

    /* 配置TFT显示屏：字体6x8像素，横向显示（160x128） */
#if !RACE_MODE
   tft180_set_font(TFT180_6X8_FONT);        /* 设置ASCII字体为6x8像素 */
    tft180_set_dir(TFT180_CROSSWISE);        /* 设置屏幕为横向显示模式 */
    tft180_init();                            /* 初始化TFT180显示屏SPI接口 */
#endif

    /* 初始化摄像头MT9V03X：188x120分辨率，灰度图像 */
    mt9v03x_init();
    /* 设置摄像头曝光时间（行周期数），值越大画面越亮 */
    mt9v03x_set_exposure_time((uint16)cam_exposure);

    /* 初始化赛道融合检测模块：清零所有边缘/中线/误差数据 */
   track_fusion_init();

    /* 初始化电机驱动串口通信（UART3，460800波特率） */
   small_driver_uart_init();

    /* 初始化PID控制器：复位转向PD、速度PI、RA状态机等所有状态 */
    line_pid_init();
    /* 初始化负压PWM输出（P21_2引脚，10kHz，初始占空比0） */
    pwm_init(VAC_PWM_CH, VAC_PWM_FREQ, 0u);

    /* 初始化IMU660RC陀螺仪：硬件初始化 + 零偏校准（约1秒） + 启动5ms PIT中断 */
    imu_init();

    /* 初始化PID周期中断：CCU60通道0，每8ms触发一次PID控制 */
   pit_ms_init(CCU60_CH0, PID_PERIOD_MS);

    /* 初始化所有按键和拨码开关的GPIO引脚（4按键 + 2拨码 + 1比赛键 + 3LED） */
    key_init_all();

    /* 初始化电池电压监测（ADC0_CH11，12位精度，32次平均滤波） */
    battery_init();

    /* 等待所有CPU核心初始化完成后才进入主循环 */
    cpu_wait_event_ready();

    /* ---- 主循环 ---- */
    while (TRUE)
    {
        /* 帧同步等待：等待摄像头完成一帧图像的采集和DMA传输 */
        /* 在等待期间执行非实时任务：自动调参日志、按键处理、菜单显示 */
        while (!mt9v03x_finish_flag)
        {
            race_control_process();
#if !RACE_MODE
            auto_tune_log_task();
            if (motor_enable == 0 || !run_quiet_active())
            {
                ui_process_keys();
                menu_show();
            }
#endif
        }
        mt9v03x_finish_flag = 0;              /* 清除帧完成标志，准备接收下一帧 */

        if (motor_enable != 0)
        {
            battery_frame_div++;
            if (battery_frame_div >= BATTERY_RUN_FRAME_DIV)
            {
                battery_frame_div = 0u;
                battery_check();
            }
        }
        else
        {
            battery_frame_div = 0u;
            battery_check();                 /* 读取电池电压，更新低电量标志 */
            auto_tune_log_task();            /* 处理自动调参数据dump */
        }

        /* ---- 视觉处理流水线 ---- */

        /* 步骤1：赛道融合处理（压缩->Otsu->二值化->降噪->边缘扫描->误差计算） */
        system_start();                      /* 开始计时 */
       track_fusion_update();                 /* 执行完整的视觉处理流水线 */
        prof_tf_us = system_getval_us();     /* 记录耗时（微秒） */

        /* 步骤2：直角/弯道预检测（远场检测，用于提前减速） */
        system_start();                      /* 开始计时 */
       right_angle_pre_detect();              /* 检测远场边缘丢失，设置g_ra_pre_flag */

        /* 步骤3：路口检测（拐点扫描 + 检测框 + 类型分类） */
        detect_intersection();               /* 检测路口类型，设置g_ra_flag */
        prof_inter_us = system_getval_us();    /* 记录耗时（微秒） */

        /* 步骤4：发布完整结果（一次publish，避免PID读到半成品） */
        track_fusion_publish();

        /* ---- 调试数据传输（非比赛模式） ---- */
#if !RACE_MODE
        /* Runtime keeps lightweight UART logs; stopped debug can send full packets. */
        if (motor_enable != 0)
        {
           send_image_uart0_runtime();
        }
        else if (!run_quiet_active() && !auto_tune_log_busy())
        {
           send_image_uart0();               /* 只有停车调试才发图传 */
        }
#endif

        /* ---- 用户交互 ---- */
        race_control_process();
#if !RACE_MODE
        if (motor_enable == 0 || !run_quiet_active())
        {
            ui_process_keys();
            menu_show();
        }
#endif
    }
}

/* 恢复默认代码段配置 */
#pragma section all restore
