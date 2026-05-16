#ifndef CODE_PID_H_
#define CODE_PID_H_

#include "zf_common_headfile.h"

extern int16 base_speed; // 基础速度

// 菜单实时可调的 PID 变量（定义在 Menu.c）
extern int16 motor_speed; // 电机速度 0~400
extern int16 pid_kp;      // 转向 P，STEER_KP = pid_kp * 0.8
extern int16 pid_ki;      // 速度 I，SPEED_KI = pid_ki * 0.25
extern int16 pid_kd;      // 转向 D，STEER_KD = pid_kd * 0.6

// 直角弯参数（定义在 Menu.c）
extern int16 ra_hard_inner;       // 内轮duty（负=反转）
extern int16 ra_hard_outer;       // 外轮duty
extern int16 ra_hard_yaw;         // 退出yaw角度（IMU禁用时无效）
extern int16 ra_slow_row;         // 减速触发行号
extern int16 ra_slow_pct;         // 减速百分比（base_speed * pct / 100）
extern int16 ra_turn_row;         // 接近触发IP行号
extern int16 ra_approach_frames;  // APPROACH阶段持续帧数

// 速度自适应参数（定义在 Menu.c）
extern int16 sp_err_t1;     // 偏差阈值1（直道，低于此值全速）
extern int16 sp_err_t2;     // 偏差阈值2（急弯，高于此值最低速）
extern int16 sp_ratio_1;    // 直道速度百分比 (0~100)
extern int16 sp_ratio_2;    // 弯道速度百分比 (0~100)
extern int16 steer_speed_k; // 转向速度耦合系数 (0~50, 代表0.000~0.050)

// ================================================================
// 转向 PD 参数（菜单变量驱动，无需重新编译即可调参）
// ================================================================
#define STEER_KP  ((float)pid_kp * 0.8f)
#define STEER_KD  ((float)pid_kd * 0.6f)
#define STEER_MAX 4000.0f
#define STEER_DEADZONE 2
#define STEER_SOFT_END 6
#define STEER_SLEW_MAX 250.0f

// ================================================================
// 速度 PI 参数
// ================================================================
#define SPEED_KP 0.5f
#define SPEED_KI ((float)pid_ki * 0.25f)
#define SPEED_I_MAX 500.0f
#define SPEED_I_SEPARATION 20
#define SPEED_FF_RATIO 0.3f
#define SPEED_FACTOR_MAX 5.0f

// ================================================================
// 输出限幅
// ================================================================
#define MAX_DUTY 5000.0f

// 误差低通滤波系数（0~1，越大越平滑）
#define ERROR_FILTER_ALPHA 0.65f

// ================================================================
// Yaw 补偿参数
// ================================================================
#define YAW_DEADZONE 1.0f

// ================================================================
// 直角弯状态机参数（11ms/帧）
// ================================================================
#define RA_HARD_TIMEOUT    45u   // HARD阶段单独超时(495ms)
#define RA_TIMEOUT_FRAMES  300u   // RA总超时(3.3秒)
#define RA_WAIT_TIMEOUT    80u   // WAIT阶段最大帧数(880ms)
#define RA_SLOW_TIMEOUT    60u   // SLOW阶段最大帧数(660ms)

// ================================================================
// 函数接口
// ================================================================
void line_pid_init(void);
void line_pid_control(void);
void line_pid_reset_derivative(void);

#endif /* CODE_PID_H_ */
