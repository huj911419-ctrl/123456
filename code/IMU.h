#ifndef CODE_IMU_H_
#define CODE_IMU_H_

#include "zf_common_headfile.h"

// ================================================================
//  Yaw 数据（在 IMU.c 中定义）
// ================================================================
extern float yaw_angle;   // 累积偏航角（度），正值=右转
extern float yaw_rate;    // 当前角速度（度/秒）

// ================================================================
//  函数接口
// ================================================================
void imu_init(void);       // IMU 初始化 + 零偏标定（上电调用一次）
void imu_update(void);     // Yaw 数据更新（在 5ms PIT 中断中调用）
void imu_reset_yaw(void);  // 清零 yaw_angle（直角弯后调用）

#endif /* CODE_IMU_H_ */
