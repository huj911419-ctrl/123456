#ifndef AUTO_TUNE_LOG_H_
#define AUTO_TUNE_LOG_H_

#include "zf_common_headfile.h"
#include "App_Config.h"

#define AUTO_TUNE_TYPE_SUMMARY 0xA0u
#define AUTO_TUNE_TYPE_RECORD  0xA1u
#define AUTO_TUNE_TYPE_END     0xA2u
#define AUTO_TUNE_TYPE_LIVE    0xA3u
#define AUTO_TUNE_VERSION      2u
#define AUTO_TUNE_RECORD_SIZE  67u
#define AUTO_TUNE_LIVE_SIZE    28u

#if (AUTO_TUNE_LOG_ENABLE || AUTO_TUNE_LIVE_ENABLE) || defined(AUTO_TUNE_LOG_IMPLEMENTATION)
void auto_tune_log_pid_tick(void);
void auto_tune_log_task(void);
uint8 auto_tune_log_busy(void);
#else
static inline void auto_tune_log_pid_tick(void) {}
static inline void auto_tune_log_task(void) {}
static inline uint8 auto_tune_log_busy(void) { return 0u; }
#endif

#endif
