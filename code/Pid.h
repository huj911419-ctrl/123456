#ifndef CODE_PID_H_
#define CODE_PID_H_

#include "zf_common_headfile.h"

extern int16 base_speed; // 基础速度

// 菜单实时可调的 PID 变量（定义在 Menu.c）
extern int16 motor_speed; // 电机速度 0~400
extern int16 pid_kp;      // 转向 P，STEER_KP = pid_kp * 0.8
extern int16 pid_ki;      // 速度 I，SPEED_KI = pid_ki * 0.25
extern int16 pid_kd;      // 转向 D，STEER_KD = pid_kd * 0.6

// 速度自适应参数（定义在 Menu.c）
extern int16 sp_err_t1;     // 偏差阈值1（直道，低于此值全速）
extern int16 sp_err_t2;     // 偏差阈值2（急弯，高于此值最低速）
extern int16 sp_ratio_1;    // 直道速度百分比 (0~100)
extern int16 sp_ratio_2;    // 弯道速度百分比 (0~100)
extern int16 steer_speed_k; // 转向速度耦合系数 (0~50, 代表0.000~0.050)

// ================================================================
// 转向 PD 参数（菜单变量驱动，无需重新编译即可调参）
// ================================================================
#define STEER_KP  ((float)pid_kp * 0.8f)   // 转向P，默认15→12.0
#define STEER_KD  ((float)pid_kd * 0.6f)   // 转向D，默认8→4.8
#define STEER_MAX 4000.0f // 转向输出限幅

// 软死区：偏差在此范围内使用二次曲线过渡，消除硬死区的"突然转向"感
// 例如偏差=3时输出=0.5*P*3=9.6(柔和)，而不是突然跳到6.4*3=19.2
#define STEER_DEADZONE 2  // 完全死区（偏差 < 此值输出0）
#define STEER_SOFT_END 6  // 软过渡结束点（偏差 > 此值全P输出）
                           // 死区 ~ 软结束之间：二次曲线平滑过渡

// 转向输出变化率限制（每帧最大变化量），防止电机指令突变
#define STEER_SLEW_MAX 250.0f  // 每11ms最多变化250，全行程需~16帧≈176ms

// 转向输出低通滤波系数（0~1，越大越平滑但响应越慢）
#define STEER_OUTPUT_FILTER 0.5f

// ================================================================
// 速度 PI 参数
// ================================================================
#define SPEED_KP 0.5f     // 速度P比例系数
#define SPEED_KI ((float)pid_ki * 0.25f) // 速度I，默认2→0.5
#define SPEED_I_MAX 500.0f // 积分限幅

// 积分分离：偏差大于此值时不积分，防止积分饱和
#define SPEED_I_SEPARATION 20 // 积分分离阈值

// ================================================================
// 速度相关参数
// ================================================================
#define MAX_DUTY 5000.0f // 输出限幅（电机驱动协议支持±10000）

// 低通滤波系数：0~1，越大越平滑（抑制摄像头帧间噪声）
#define ERROR_FILTER_ALPHA 0.65f

// ================================================================
//  Yaw 补偿参数（IMU 相关，当前未启用）
// ================================================================
#define YAW_KP 10.0f      // Yaw 补偿比例系数
#define YAW_DEADZONE 1.0f // Yaw 死区（角度绝对值小于此值不补偿）

// ================================================================
// 函数接口
// ================================================================
void line_pid_init(void);             // PID初始化
void line_pid_control(void);          // PID控制主函数（在定时器中调用）
void line_pid_reset_derivative(void); // 复位D项（微分复位）

#endif /* CODE_PID_H_ */
