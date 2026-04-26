#ifndef CODE_PID_H_
#define CODE_PID_H_

#include "zf_common_headfile.h"
// ==================== 转向 PD 参数 ====================

#define STEER_KP 50.0f
#define STEER_KD 25.0f
#define STEER_MAX 4000.0f // 转向输出限幅

// 转向死区：位置误差绝对值小于此值时，认为已对中，转向输出归零
#define STEER_DEADZONE 2 // 单位：像素

// ==================== 速度 PI 参数 ====================

#define SPEED_KP 0.45f
#define SPEED_KI 0.6f
#define SPEED_I_MAX 3000.0f // 积分限幅

// 积分分离阈值：位置误差绝对值超过此值时，关闭速度积分，防止超调
#define SPEED_I_SEPARATION 20 // 单位：像素

// ==================== 速度目标参数 ====================

#define BASE_SPEED 900.0f  // 直道目标速度（编码器计数/周期）  2000
#define MIN_SPEED 300.0f    // 弯道最低目标速度   500
#define MAX_DUTY 1300.0f   // 输出占空比最大限幅    10000
#define SPEED_REDUCE_K 5.0f // 弯道自适应降速系数

// ==================== 对外接口 ====================

void line_pid_init(void);
void line_pid_control(void);
void line_pid_reset_derivative(void); // 微分手动复位

#endif /* CODE_PID_H_ */    
