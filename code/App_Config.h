/**
 * ========================================================================
 * App_Config.h - 应用全局配置文件
 * ========================================================================
 * 本文件集中定义所有可编译时配置的宏开关和参数。
 * 通过修改这些宏可以在不改动代码逻辑的情况下切换工作模式：
 *
 * 主要配置项：
 *   RACE_MODE     - 比赛模式开关（0=调试模式，1=比赛模式）
 *   UART0_DEBUG   - 调试串口数据发送开关
 *   AUTO_TUNE_LOG - 自动调参日志开关
 *   VACUUM_RUN    - 负压风扇控制开关
 *   TF_OTSU_INTERVAL - Otsu自适应阈值计算间隔
 * ======================================================================== */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* ========================================================================
 * 工作模式配置
 * ======================================================================== */

/**
 * RACE_MODE - 比赛模式开关
 *   0 = 调试模式（默认）：启用TFT显示、UART图像传输、菜单按键功能
 *   1 = 比赛模式：禁用所有调试功能，最大化CPU用于实时控制
 *       - TFT屏幕不刷新
 *       - UART0不发送调试数据
 *       - 菜单按键不响应（仅保留比赛按键）
 */
#ifndef RACE_MODE
#define RACE_MODE 0
#endif

/* ========================================================================
 * 运行静默模式配置
 * ======================================================================== */

/**
 * RUN_QUIET_DEFAULT_ENABLE - 运行静默模式默认开关
 *   0 = 默认关闭：运行时保持TFT/UART/菜单按键工作
 *   1 = 默认开启：运行时关闭TFT显示和UART调试输出，节省CPU时间
 * 可通过菜单在运行时动态切换。
 */
#define RUN_QUIET_DEFAULT_ENABLE 0

/**
 * RUN_QUIET_STOP_KEY - 静默模式下保留的紧急停止按键
 *   1 = KEY1（推荐），2 = KEY2，3 = KEY3，4 = KEY4
 * 在静默模式下，仅此按键可用于停止电机，其他按键被忽略。
 */
#define RUN_QUIET_STOP_KEY      1u

/* ========================================================================
 * UART0 调试数据流配置
 * ========================================================================
 * 仅在 RACE_MODE=0 且静默模式关闭时生效。
 * 通过分频系数控制各类数据的发送频率，降低串口负载：
 *   div=1：每帧都发送
 *   div=5：每5帧发送1帧
 *   div=0：不发送
 */

#define UART0_DEBUG_ENABLE      0u          /* UART0调试总开关：1=开启，0=关闭 */
#define UART0_IMAGE_DIV         5u          /* 二值化图像发送分频：每5帧发1帧（降低带宽） */
#define UART0_EDGES_DIV         5u          /* 边线坐标发送分频：每5帧发1帧 */
#define UART0_PARAMS_DIV        1u          /* TFT参数发送分频：每帧都发（数据量小） */
#define UART0_TELEMETRY_DIV     1u          /* 遥测数据发送分频：每帧都发 */

/* Runtime UART logging: keep lightweight packets while the car is running.
 * UART0 writes are blocking at 115200bps, so keep these dividers sparse. */
#define UART0_RUN_ENABLE        1u
#define UART0_RUN_IMAGE_DIV     0u
#define UART0_RUN_EDGES_DIV     0u
#define UART0_RUN_PARAMS_DIV    6u
#define UART0_RUN_TELEMETRY_DIV 6u

/* ========================================================================
 * 自动调参日志（AutoTune Log）配置
 * ========================================================================
 * 在运行时记录每帧的PID状态到环形缓冲区，停止后通过UART0批量dump到PC。
 * 用于离线分析PID参数效果，优化控制性能。
 */

#define AUTO_TUNE_LOG_ENABLE    1u          /* 自动调参日志开关：1=开启 */
#define AUTO_TUNE_LOG_PID_DIV   4uu          /* 记录分频：每6个PID周期记录1次（约48ms） */
#define AUTO_TUNE_LOG_CAPACITY  128u        /* 环形缓冲区容量：512条记录（约24秒数据） */
#define AUTO_TUNE_DUMP_PER_PKT  4u          /* 每个UART包包含4条记录 */
#define AUTO_TUNE_LIVE_ENABLE   0u          /* 实时发送开关：1=运行时也发送最新记录 */
#define AUTO_TUNE_LIVE_DIV      32u         /* 实时发送分频：每32条记录发1条 */
#define BATTERY_RUN_FRAME_DIV   3u          /* 运行时电池采样分频：每N个图像帧更新一次 */

/* ========================================================================
 * 负压风扇（Vacuum）配置
 * ========================================================================
 * 负压风扇通过PWM控制，用于增加车轮与地面的摩擦力。
 */

#define VACUUM_RUN_ENABLE      1u           /* 负压运行使能：1=比赛运行时开启负压 */
#define VACUUM_PREARM_ENABLE   1u           /* 负压预启动：1=在电机启动前先开启负压 */

/* ========================================================================
 * Otsu自适应阈值配置
 * ========================================================================
 * Otsu算法通过最大化类间方差自动计算二值化阈值。
 * 每隔一定帧数重新计算一次，平衡自适应性和CPU负载。
 */

#define TF_OTSU_INTERVAL_DEBUG 6u           /* 调试模式Otsu间隔：每6帧计算一次 */
#define TF_OTSU_INTERVAL_RACE  8u           /* 比赛模式Otsu间隔：每8帧计算一次（节省CPU） */

/**
 * 固定阈值模式（可选）
 * 在光照稳定的比赛环境中，可跳过Otsu计算，直接使用固定阈值。
 * 优点：减少CPU负载，提高帧率
 * 缺点：光照变化时二值化质量下降
 */
#define TF_FIXED_THRESHOLD_ENABLE 0         /* 固定阈值开关：0=使用Otsu自适应，1=使用固定阈值 */
#define TF_FIXED_THRESHOLD_VALUE  70        /* 固定阈值（仅TF_FIXED_THRESHOLD_ENABLE=1时生效） */

#endif /* APP_CONFIG_H */
