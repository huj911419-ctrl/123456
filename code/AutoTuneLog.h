/**
 * ========================================================================
 * AutoTuneLog.h - 自动调参日志模块头文件
 * ========================================================================
 * 在运行时记录PID控制状态到环形缓冲区，停止后批量dump到PC。
 * 用于离线分析PID参数效果，优化直角弯、速度规划等控制策略。
 *
 * 数据记录格式：
 *   每条记录74字节，包含完整的PID状态快照
 *   通过UART0以自定义协议发送到PC
 * ======================================================================== */
#ifndef AUTO_TUNE_LOG_H_
#define AUTO_TUNE_LOG_H_

#include "zf_common_headfile.h"
#include "App_Config.h"

/* ========================================================================
 * 协议常量定义
 * ======================================================================== */

#define AUTO_TUNE_TYPE_SUMMARY 0xA0u        /* 数据包类型：摘要包（版本、配置、记录数） */
#define AUTO_TUNE_TYPE_RECORD  0xA1u        /* 数据包类型：记录数据包（多条记录打包） */
#define AUTO_TUNE_TYPE_END     0xA2u        /* 数据包类型：传输结束标记 */
#define AUTO_TUNE_TYPE_LIVE    0xA3u        /* 数据包类型：实时记录包（运行时单条记录） */
#define AUTO_TUNE_VERSION      6u           /* 协议版本号，PC端据此解析数据格式 */
#define AUTO_TUNE_RECORD_SIZE  117u         /* 单条记录的字节数 */
#define AUTO_TUNE_LIVE_SIZE    31u          /* 实时记录的字节数（精简版） */

/* ========================================================================
 * 接口函数声明
 * ========================================================================
 * 当 AUTO_TUNE_LOG_ENABLE 或 AUTO_TUNE_LIVE_ENABLE 为1时，提供完整实现。
 * 否则提供空内联实现，不占用任何ROM/RAM。
 * ======================================================================== */

#if (AUTO_TUNE_LOG_ENABLE || AUTO_TUNE_LIVE_ENABLE) || defined(AUTO_TUNE_LOG_IMPLEMENTATION)

/**
 * @brief PID周期tick记录函数（在PID ISR中调用）
 *
 * 每个PID周期调用一次，根据分频系数决定是否记录当前状态。
 * 记录内容包括：速度、误差、yaw角、RA状态、电机duty等。
 * 在直角弯/路口/丢线等特殊状态下自动提高记录频率。
 */
void auto_tune_log_pid_tick(void);

/**
 * @brief 后台任务函数（在主循环中调用）
 *
 * 运行时：发送实时记录（如果 AUTO_TUNE_LIVE_ENABLE=1）
 * 停止时：将缓冲区中的记录批量dump到PC
 */
void auto_tune_log_task(void);

/**
 * @brief 查询是否正在传输数据
 * @return 0=空闲，1=正在记录或dump数据（此时不应发送其他UART数据）
 */
uint8 auto_tune_log_busy(void);

#else
/* 自动调参功能关闭时，提供空实现 */
static inline void auto_tune_log_pid_tick(void) {}
static inline void auto_tune_log_task(void) {}
static inline uint8 auto_tune_log_busy(void) { return 0u; }
#endif

#endif
