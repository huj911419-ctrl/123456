/**
 * ========================================================================
 * Pid.h - PID控制模块头文件
 * ========================================================================
 * 定义了转向PD控制、速度PI控制、RA状态机、路线表、
 * 串级PID、单边巡线、丢线搜索等所有控制参数和接口。
 *
 * 本文件是整个控制系统的核心配置文件，所有可调参数均在此定义。
 * 参数通过菜单系统（Menu.c）在运行时动态调节。
 * ======================================================================== */
#ifndef CODE_PID_H_
#define CODE_PID_H_

#include "zf_common_headfile.h"
#include "App_Config.h"

/* ========================================================================
 * PID周期时间参数
 * ========================================================================
 * PID控制周期为8ms（125Hz），所有时间相关的增益和斜坡参数
 * 都基于11ms的参考周期进行了归一化处理。
 * PID_DT_SCALE = 8/11 ≈ 0.727，用于缩放步长和积分项。
 * PID_D_SCALE = 11/8 = 1.375，用于缩放微分项（周期越短，变化率越大）。
 * ======================================================================== */

#define PID_PERIOD_MS      8u               /* PID控制周期（毫秒）：8ms = 125Hz */
#define PID_PERIOD_REF_MS  11u              /* 参考周期（毫秒）：参数调试时的基准周期 */
#define PID_DT_SCALE       ((float)PID_PERIOD_MS / (float)PID_PERIOD_REF_MS) /* 时间缩放因子 ≈ 0.727 */
#define PID_D_SCALE        ((float)PID_PERIOD_REF_MS / (float)PID_PERIOD_MS) /* 微分缩放因子 ≈ 1.375 */

/**
 * PID_MS_TO_TICKS - 将毫秒转换为PID周期数（向上取整）
 * @ms: 时间（毫秒）
 * 返回: 对应的PID周期数
 * 例如：500ms / 8ms = 62.5 -> 63个周期
 */
#define PID_MS_TO_TICKS(ms) \
    ((uint16)((((uint32)(ms)) + (uint32)PID_PERIOD_MS - 1u) / (uint32)PID_PERIOD_MS))

/**
 * PID_SECONDS_TO_TICKS - 将秒转换为PID周期数（向上取整）
 * @sec: 时间（秒）
 * 返回: 对应的PID周期数
 */
#define PID_SECONDS_TO_TICKS(sec) \
    ((uint32)((((uint32)(sec) * 1000u) + (uint32)PID_PERIOD_MS - 1u) / (uint32)PID_PERIOD_MS))

/* ========================================================================
 * 调试输出变量声明
 * ======================================================================== */

extern int16 base_speed;            /* 基础速度（由速度规划器输出） */
extern int16 speed_dbg_out;         /* 速度PI调试输出 */
extern int16 steer_dbg_out;         /* 转向PD调试输出 */
extern int16 duty_dbg_left;         /* 最终左电机duty（发送给电机驱动板） */
extern int16 duty_dbg_right;        /* 最终右电机duty */
extern uint8 speed_dbg_vq_pct;      /* 视觉质量降速百分比（100=无降速） */
extern uint8 speed_dbg_pre_lock;    /* 预减速锁定标志 */
extern int16 speed_dbg_raw;         /* 原始目标速度（规划前） */
extern int16 speed_dbg_plan;        /* 规划后目标速度 */
extern uint8 speed_dbg_reason;      /* 速度变化原因编号 */

/* ========================================================================
 * 菜单可调参数声明（定义在Menu.c中）
 * ======================================================================== */

extern int16 motor_speed;           /* 目标电机速度（菜单可调） */
extern int16 pid_kp;                /* 转向PD比例系数（菜单可调） */
extern int16 pid_ki;                /* 速度PI积分系数（菜单可调） */
extern int16 pid_kd;                /* 转向PD微分系数（菜单可调） */
extern int16 yaw_kp;               /* 串级PID内环比例系数（菜单可调） */

extern int16 cascade_en;            /* 串级PID使能：0=普通PD，1=串级IMU */
extern int16 yaw_kd;                /* 串级PID内环微分系数（菜单可调） */
extern int16 yaw_damp_gain;         /* 独立yaw阻尼增益（cascade_en=0时使用） */

extern int16 ra_hard_inner;         /* RA HARD阶段内侧duty */
extern int16 ra_hard_outer;         /* RA HARD阶段外侧duty */
extern int16 ra_hard_rate;          /* RA HARD阶段目标yaw速率（度/秒） */
extern int16 ra_hard_yaw;           /* RA HARD阶段退出航向角阈值（度） */
extern int16 ra_slow_row;           /* RA SLOW阶段触发行号 */
extern int16 ra_slow_pct;           /* RA SLOW阶段速度百分比 */
extern int16 ra_turn_row;           /* RA APPROACH阶段触发行号 */
extern int16 ra_approach_frames;    /* RA APPROACH阶段刹车稳车帧数 */

extern int16 sp_err_t1;             /* 速度规划：直道误差阈值 */
extern int16 sp_err_t2;             /* 速度规划：弯道误差阈值 */
extern int16 sp_ratio_1;            /* 速度规划：直道速度百分比 */
extern int16 sp_ratio_2;            /* 速度规划：弯道速度百分比 */
extern int16 steer_speed_k;         /* 转向速度耦合系数 */
extern int16 steer_ff_k;            /* 前瞻前馈系数 */

/* ========================================================================
 * RA调试变量声明
 * ======================================================================== */

extern uint8 ra_dbg_state;          /* RA状态机当前状态 */
extern uint8 ra_dbg_phase;          /* RA当前阶段 */
extern uint8 ra_dbg_dir;            /* RA方向 */
extern uint8 ra_dbg_ip_row;         /* RA最大拐点行 */
extern uint16 ra_dbg_timer;         /* RA全局计时器 */
extern uint16 ra_dbg_hard_cnt;      /* RA HARD帧计数 */
extern uint8 ra_dbg_exit_good_cnt;  /* RA退出确认计数 */
extern int16 ra_dbg_yaw10;          /* RA yaw进度x10 */
extern uint8 ra_dbg_slow_row;       /* RA SLOW触发行 */
extern uint8 ra_dbg_turn_row;       /* RA TURN触发行 */
extern uint8 ra_dbg_exit_reason;    /* RA退出原因 */
extern int16 ra_dbg_hard_target10;  /* RA HARD目标yawx10 */
extern int16 ra_dbg_outer_cmd;      /* RA外侧命令 */
extern uint8 route_dbg_step;        /* 路线调试当前步数 */
extern uint8 route_dbg_total;       /* 路线调试总步数 */
extern uint8 route_dbg_flag;        /* 路线调试当前flag */
extern uint8 route_dbg_count;       /* 路线调试当前count */
extern uint8 route_dbg_action;      /* 路线调试当前动作 */
extern volatile uint8 vacuum_enable; /* 负压实际运行状态 */

/* ========================================================================
 * 转向PD控制参数
 * ========================================================================
 * 转向PD控制器根据视觉误差计算转向校正量。
 * 使用软死区防止小幅抖动，使用增益调度适应不同速度和弯道。
 */

#define STEER_KP  ((float)pid_kp * 0.8f)   /* 转向比例系数 = 菜单Kp * 0.8 */
#define STEER_KD  ((float)pid_kd * 0.6f)   /* 转向微分系数 = 菜单Kd * 0.6 */
#define STEER_MAX 4000.0f                   /* 转向输出最大值 */
#define STEER_DEADZONE 2                    /* 死区宽度：误差<=2时输出为0 */
#define STEER_SOFT_END 6                    /* 软死区结束：死区到此处用二次曲线平滑 */
#define STEER_SLEW_MAX 600.0f               /* 输出变化率限制：每帧最大变化600 */

/* 增益调度参数：根据速度和曲率动态调整Kp/Kd/FF */
#define STEER_GAIN_SPEED_START 180          /* 速度因子起点 */
#define STEER_GAIN_SPEED_END 800            /* 速度因子终点 */
#define STEER_GAIN_CURVE_T1 10              /* 曲率因子起点 */
#define STEER_GAIN_CURVE_T2 38              /* 曲率因子终点 */
#define STEER_FAST_KP_PCT 105               /* 高速Kp百分比（略增） */
#define STEER_FAST_KD_PCT 105               /* 高速Kd百分比（略增） */
#define STEER_CURVE_KP_PCT 155              /* 弯道Kp百分比（增大响应） */
#define STEER_CURVE_KD_PCT 145              /* 弯道Kd百分比（增大阻尼） */
#define STEER_FF_SPEED_START 180            /* 前馈启用速度起点 */
#define STEER_FF_SPEED_END 800              /* 前馈启用速度终点 */
#define STEER_FF_MAX 600.0f                 /* 前馈最大值 */
#define STEER_FF_FILTER_ALPHA 0.35f         /* 前馈滤波系数 */
#define STEER_DIFF_MIN_LIMIT 250.0f         /* 差速最小限制 */
#define STEER_DIFF_MAX_LIMIT 1900.0f        /* 差速最大限制 */
#define STEER_DIFF_NORMAL_PCT 95            /* 正常差速百分比 */
#define STEER_DIFF_STRAIGHT_PCT 95          /* 直道差速百分比 */
#define STEER_DIFF_SINGLE_EDGE_PCT 112      /* 单边差速百分比 */
#define STEER_DIFF_RECOVER_PCT 120          /* 恢复差速百分比 */
#define STEER_STRAIGHT_KP_PCT 122           /* 直道Kp百分比 */
#define STEER_STRAIGHT_KD_PCT 114           /* 直道Kd百分比 */
#define STEER_STRAIGHT_FF_PCT 85            /* 直道前馈百分比 */
#define STEER_STRAIGHT_SLEW_PCT 130         /* 直道变化率百分比 */
#define STEER_RECENTER_T1 8                 /* 重归中阈值1 */
#define STEER_RECENTER_T2 28                /* 重归中阈值2 */
#define STEER_RECENTER_MAX_PCT 165          /* 重归中最大百分比 */
#define STEER_SINGLE_EDGE_RECENTER_MAX_PCT 185 /* 单边重归中最大百分比 */
#define STEER_LEFT_BIAS_VALID_ROWS 38u      /* 左偏置有效行数 */
#define STEER_LEFT_BIAS_ERR_MIN 3           /* 左偏置最小误差 */
#define STEER_LEFT_BIAS_ERR_MAX 22          /* 左偏置最大误差 */
#define STEER_LEFT_BIAS_MAX 260.0f          /* 左偏置最大值 */
#define STEER_LEFT_BIAS_START_MAX 320.0f    /* 左偏置起始最大值 */
#define STEER_LEFT_BIAS_START_ROUTE_STEP 0u /* 左偏置起始路线步 */
#define STEER_COMPLEX_CURVE_KP_PCT 240      /* 复杂弯道Kp百分比 */
#define STEER_COMPLEX_CURVE_KD_PCT 190      /* 复杂弯道Kd百分比 */
#define STEER_COMPLEX_CURVE_FF_PCT 190      /* 复杂弯道前馈百分比 */
#define STEER_COMPLEX_CURVE_SLEW_PCT 320    /* 复杂弯道变化率百分比 */
#define STEER_COMPLEX_CURVE_DIFF_PCT 190    /* 复杂弯道差速百分比 */
#define STEER_COMPLEX_CURVE_DIFF_MAX 3900.0f /* 复杂弯道差速最大值 */
#define STEER_RA_CURVE_KP_PCT 260           /* RA弯道Kp百分比 */
#define STEER_RA_CURVE_KD_PCT 210           /* RA弯道Kd百分比 */
#define STEER_RA_CURVE_FF_PCT 220           /* RA弯道前馈百分比 */
#define STEER_RA_CURVE_SLEW_PCT 360         /* RA弯道变化率百分比 */
#define STEER_RA_CURVE_DIFF_PCT 220         /* RA弯道差速百分比 */
#define STEER_RA_CURVE_DIFF_MAX 4200.0f     /* RA弯道差速最大值 */

/* ========================================================================
 * 速度PI控制参数
 * ======================================================================== */

#define SPEED_KP 0.5f                       /* 速度比例系数 */
#define SPEED_KI ((float)pid_ki * 0.25f)    /* 速度积分系数 = 菜单Ki * 0.25 */
#define SPEED_I_MAX 500.0f                  /* 积分限幅 */
#define SPEED_I_SEPARATION 20               /* 积分分离阈值：误差>20时不积分 */
#define SPEED_FF_RATIO 0.55f                /* 前馈比例 */
#define SPEED_ACCEL_FF_GAIN 0.35f           /* 加速度前馈增益 */
#define SPEED_ACCEL_FF_LIMIT 280.0f         /* 加速度前馈限幅 */
#define SPEED_FACTOR_MAX 2.0f               /* 速度因子最大值 */
#define SPEED_BASE_CAP_NORMAL 2200          /* 正常基础速度上限 */
#define SPEED_BASE_CAP_RA 1450              /* RA基础速度上限 */
#define SPEED_OUT_MAX_PCT 118               /* 速度输出最大百分比 */
#define SPEED_OUT_MIN_HEADROOM 120.0f       /* 速度输出最小余量 */
#define SPEED_OUT_MAX_HEADROOM 430.0f       /* 速度输出最大余量 */
#define SPEED_OUT_TURN_T1 12                /* 转弯输出阈值1 */
#define SPEED_OUT_TURN_T2 30                /* 转弯输出阈值2 */
#define SPEED_OUT_TURN_MIN_HEADROOM_PCT 35  /* 转弯最小余量百分比 */
#define SPEED_STRAIGHT_VALID_ROWS 28u       /* 直道有效行数 */
#define SPEED_STRAIGHT_ERR_MAX 16           /* 直道最大误差 */
#define SPEED_STRAIGHT_LOOKAHEAD_MAX 18     /* 直道最大前瞻误差 */
#define SPEED_STRAIGHT_TREND_MAX 18         /* 直道最大趋势误差 */
#define SPEED_SYM_VALID_ROWS 30u            /* 对称有效行数 */
#define SPEED_SYM_ERR_MAX 12                /* 对称最大误差 */
#define SPEED_STRAIGHT_CONFIRM_FRAMES 3u    /* 直道确认帧数 */
#define SPEED_STRAIGHT_BOOST_PCT 116        /* 直道加速百分比 */
#define SPEED_STRAIGHT_HOLD_FRAMES 4u       /* 直道保持帧数 */
#define SPEED_STRAIGHT_HOLD_VALID_ROWS 30u  /* 直道保持有效行数 */
#define SPEED_STRAIGHT_HOLD_ERR_MAX 16      /* 直道保持最大误差 */
#define SPEED_STRAIGHT_HOLD_LOOKAHEAD_MAX 18 /* 直道保持最大前瞻误差 */
#define SPEED_STRAIGHT_HOLD_TREND_MAX 18    /* 直道保持最大趋势误差 */
#define SPEED_STRAIGHT_STEER_PCT 100        /* 直道转向百分比 */
#define SPEED_LOOKAHEAD_SLOW_T1 4           /* 前瞻减速阈值1 */
#define SPEED_LOOKAHEAD_SLOW_T2 24          /* 前瞻减速阈值2 */
#define SPEED_LOOKAHEAD_MIN_PCT 76          /* 前瞻最小百分比 */
#define SPEED_TREND_SLOW_T1 4               /* 趋势减速阈值1 */
#define SPEED_TREND_SLOW_T2 22              /* 趋势减速阈值2 */
#define SPEED_TREND_MIN_PCT 78              /* 趋势最小百分比 */
#define SPEED_VISION_BAD_VALID_ROWS 6u      /* 视觉差有效行数 */
#define SPEED_VISION_BAD_PCT 55             /* 视觉差百分比 */
#define SPEED_INTER_RISK_VALID_ROWS 34u     /* 路口风险有效行数 */
#define SPEED_INTER_RISK_IP_ROW 46u         /* 路口风险拐点行 */
#define SPEED_INTER_RISK_PCT 48             /* 路口风险百分比 */
#define SPEED_QUALITY_GOOD_ROWS 35u         /* 质量好有效行数 */
#define SPEED_QUALITY_ROW_MIN_PCT 82        /* 质量行最小百分比 */
#define SPEED_COMPONENT_VALID_ROWS 30u      /* 组件有效行数 */
#define SPEED_COMPONENT_ERR_MAX 16          /* 组件最大误差 */
#define SPEED_COMPONENT_LA_MAX 18           /* 组件最大前瞻误差 */
#define SPEED_COMPONENT_TREND_MAX 18        /* 组件最大趋势误差 */
#define SPEED_SINGLE_EDGE_VALID_ROWS 28u    /* 单边有效行数 */
#define SPEED_SINGLE_EDGE_ERR_MAX 16        /* 单边最大误差 */
#define SPEED_SINGLE_EDGE_LOOKAHEAD_MAX 24  /* 单边最大前瞻误差 */
#define SPEED_SINGLE_EDGE_TREND_MAX 24      /* 单边最大趋势误差 */
#define SPEED_SINGLE_EDGE_HOLD_ERR_MAX 22   /* 单边保持最大误差 */
#define SPEED_SINGLE_EDGE_HOLD_LOOKAHEAD_MAX 30 /* 单边保持最大前瞻误差 */
#define SPEED_SINGLE_EDGE_HOLD_TREND_MAX 30 /* 单边保持最大趋势误差 */
#define SPEED_SINGLE_EDGE_BOOST_PCT 124     /* 单边加速百分比 */
#define SPEED_SINGLE_EDGE_HOLD_FRAMES 4u    /* 单边保持帧数 */
#define SPEED_VALID_RUSH_ROWS 48u           /* 冲刺有效行数 */
#define SPEED_VALID_RUSH_ERR_MAX 16         /* 冲刺最大误差 */
#define SPEED_VALID_RUSH_LA_MAX 18          /* 冲刺最大前瞻误差 */
#define SPEED_VALID_RUSH_TREND_MAX 18       /* 冲刺最大趋势误差 */
#define SPEED_VALID_RUSH_PCT 96             /* 冲刺百分比 */
#define SPEED_VALID_RUN_ROWS 42u            /* 运行有效行数 */
#define SPEED_VALID_RUN_ERR_MAX 24          /* 运行最大误差 */
#define SPEED_VALID_RUN_LA_MAX 34           /* 运行最大前瞻误差 */
#define SPEED_VALID_RUN_TREND_MAX 34        /* 运行最大趋势误差 */
#define SPEED_VALID_RUN_MIN_PCT 86          /* 运行最小百分比 */
#define SPEED_ERR_JUMP_T1 10                /* 误差跳变阈值1 */
#define SPEED_ERR_JUMP_T2 28                /* 误差跳变阈值2 */
#define SPEED_ERR_JUMP_MIN_PCT 82           /* 误差跳变最小百分比 */
#define SPEED_LINE_LOST_PCT 35              /* 丢线百分比 */
#define SPEED_RAMP_UP_STEP 1400             /* 速度上升步长 */
#define SPEED_RAMP_STRAIGHT_UP_STEP 1700    /* 直道上升步长 */
#define SPEED_RAMP_DOWN_STEP 460            /* 速度下降步长 */
#define SPEED_RAMP_SOFT_DOWN_STEP 260       /* 软下降步长 */

/* ========================================================================
 * 电机和真空泵参数
 * ======================================================================== */

#define MAX_DUTY 5000.0f                    /* 电机最大duty（±5000） */
#define VAC_PWM_CH ATOM0_CH0_P21_2         /* 负压PWM通道：P21_2引脚 */
#define VAC_PWM_FREQ 10000u                 /* 负压PWM频率：10kHz */
#define VAC_PWM_DUTY 5000u                  /* 负压满占空比 */
#define VAC_PWM_RAMP_START_DUTY 1000u       /* 负压启动占空比（软启动） */
#define VAC_PWM_RAMP_STEP 25u               /* 负压斜坡步长 */
#define VAC_PREARM_TIMEOUT_TICKS PID_MS_TO_TICKS(5000u) /* 负压预启动超时：5秒 */
#define ERROR_FILTER_ALPHA 0.55f            /* 误差滤波系数（正常模式） */
#define ERROR_FILTER_STRAIGHT_ALPHA 0.78f   /* 误差滤波系数（直道模式，更平滑） */

/* ========================================================================
 * Yaw补偿参数（默认关闭）
 * ======================================================================== */

#define YAW_COMP_ENABLE 0                   /* Yaw补偿开关：0=关闭 */
#define YAW_DEADZONE 1.0f                   /* Yaw死区（度/秒） */
#define YAW_RATE_LIMIT 200.0f               /* Yaw速率限幅（度/秒） */
#define YAW_RATE_LIMIT_MIN_PCT 35           /* Yaw速率限幅最低保留百分比 */

/* ========================================================================
 * RA（直角弯/路口）状态机参数
 * ========================================================================
 * RA状态机阶段：WAIT -> SLOW -> APPROACH -> HARD -> YAW_LOCK -> RECOVER
 *   WAIT：等待拐点接近
 *   SLOW：减速阶段
 *   APPROACH：接近阶段，准备转弯
 *   HARD：急转弯阶段，固定duty驱动
 *   YAW_LOCK：yaw锁定阶段
 *   RECOVER：恢复阶段，平滑过渡到正常巡线
 * ======================================================================== */

#define RA_HARD_TIMEOUT          52u        /* HARD超时帧数 */
#define RA_FAST_SPEED_START      520        /* 高速模式速度阈值 */
#define RA_FAST_HARD_TIMEOUT     52u        /* 高速HARD超时 */
#define RA_CROSS_HARD_TIMEOUT    28u        /* 十字路口HARD超时 */
#define RA_HARD_FORCE_TIMEOUT_EXTRA 6u      /* 强制超时额外帧数 */
#define RA_HARD_EMERGENCY_TIMEOUT_EXTRA 8u  /* 紧急超时额外帧数 */
#define RA_TIMEOUT_FRAMES        150u       /* RA全局超时帧数 */
#define RA_WAIT_TIMEOUT          80u        /* WAIT超时帧数 */
#define RA_SLOW_TIMEOUT          28u        /* SLOW超时帧数 */
#define RA_FAST_SLOW_TIMEOUT     10u        /* 高速SLOW超时 */
#define RA_LOW_SPEED_START       360        /* 低速模式速度阈值 */
#define RA_LOW_SLOW_TIMEOUT      10u        /* 低速SLOW超时 */
#define RA_PRE_SLOW_PCT          50         /* 预减速百分比 */
#define RA_CROSS_WAIT_TIMEOUT    2u         /* 十字WAIT超时 */
#define RA_CROSS_SLOW_TIMEOUT    10u        /* 十字SLOW超时 */
#define RA_HARD_MIN_FRAMES       4u         /* HARD最小帧数 */
#define RA_FAST_HARD_MIN_FRAMES  4u         /* 高速HARD最小帧数 */
#define RA_HARD_RAMP_FRAMES      2u         /* HARD斜坡帧数 */
#define RA_FAST_HARD_RAMP_FRAMES 1u         /* 高速HARD斜坡帧数 */
#define RA_CROSS_HARD_MIN_FRAMES 4u         /* 十字HARD最小帧数 */
#define RA_EXIT_VALID_ROWS       10u        /* 退出有效行数 */
#define RA_EXIT_ERROR_MAX        28         /* 退出最大误差 */
#define RA_EXIT_CONFIRM_FRAMES   2u         /* 退出确认帧数 */
#define RA_RECOVER_FIXED_FRAMES  2u         /* RECOVER固定帧数 */
#define RA_RECOVER_SPEED_PCT     92         /* RECOVER速度百分比 */
#define RA_RECOVER_LOST_SPEED_PCT 75        /* RECOVER丢线速度百分比 */
#define RA_RECOVER_STEER_PCT     100        /* RECOVER转向百分比 */
#define RA_RECOVER_SEEN_ROWS     RA_EXIT_VALID_ROWS /* RECOVER可见行数 */
#define RA_RECOVER_SEEN_CONFIRM_FRAMES 2u   /* RECOVER可见确认帧数 */
#define RA_RECOVER_VALID_ROWS    6u         /* RECOVER有效行数 */
#define RA_RECOVER_ERROR_MAX     18         /* RECOVER最大误差 */
#define RA_RECOVER_LOOKAHEAD_MAX 24         /* RECOVER最大前瞻误差 */
#define RA_RECOVER_TREND_MAX     28         /* RECOVER最大趋势误差 */
#define RA_RECOVER_CONFIRM_FRAMES 1u        /* RECOVER确认帧数 */
#define RA_RECOVER_STABLE_MIN_YAW_DEG 58.0f /* RECOVER稳定最小yaw角度 */
#define RA_RECOVER_MAX_FRAMES    8u         /* RECOVER最大帧数 */
#define RA_RECOVER_YAW_ERROR_MAX 1.0f       /* RECOVER yaw误差最大值 */
#define RA_RECOVER_YAW_RATE_MAX  130.0f     /* RECOVER yaw速率最大值 */
#define RA_RECOVER_NEAR_DETECT_MIN_FRAMES RA_RECOVER_FIXED_FRAMES
#define RA_RECOVER_YAW_TARGET_DEG 70.0f     /* RECOVER yaw目标角度 */
#define RA_RECOVER_YAW_KP         0.0f      /* RECOVER yaw比例系数 */
#define RA_RECOVER_YAW_MAX        280.0f    /* RECOVER yaw最大值 */
#define RA_RECOVER_YAW_RATE_KD    0.00f     /* RECOVER yaw速率微分系数 */
#define RA_RECOVER_YAW_RATE_MAX_CORR 900.0f /* RECOVER yaw速率最大校正 */
#define RA_RECOVER_YAW_DRIVE_RATE_MAX 150.0f /* RECOVER yaw驱动速率最大值 */
#define RA_RECOVER_YAW_DEADZONE   3.0f      /* RECOVER yaw死区 */
#define RA_RECOVER_NO_REVERSE_FRAMES 4u     /* RECOVER禁止反转帧数 */
#define RA_RECOVER_NO_REVERSE_RATE 260.0f   /* RECOVER禁止反转速率 */
#define RA_RECOVER_SEED_STEER_PCT 20        /* RECOVER种子转向百分比 */
#define RA_EXIT_VIS_MIN_YAW_DEG 50.0f       /* 退出视觉最小yaw角度 */
#define RA_EXIT_VIS_STRONG_ROWS 16u         /* 退出视觉强行数 */
#define RA_EXIT_BEZIER_VALID_ROWS 14u       /* 退出贝塞尔有效行数 */
#define RA_EXIT_BEZIER_IP_ROW   34u         /* 退出贝塞尔拐点行 */
#define RA_EXIT_FIND_YAW_DEG    78.0f       /* 退出查找yaw角度 */
#define RA_EXIT_FORCE_YAW_DEG   90.0f       /* 退出强制yaw角度 */
#define RA_EXIT_VIS_DUTY_MIN      560.0f    /* 退出视觉最小duty */
#define RA_EXIT_VIS_DUTY_MAX      1350.0f   /* 退出视觉最大duty */
#define RA_EXIT_VIS_KP_SCALE      1.45f     /* 退出视觉Kp缩放 */
#define RA_EXIT_VIS_KD_SCALE      2.20f     /* 退出视觉Kd缩放 */
#define RA_EXIT_VIS_TURN_MAX      980.0f    /* 退出视觉最大转向 */
#define RA_EXIT_VIS_TURN_PCT      95        /* 退出视觉转向百分比 */
#define RA_EXIT_VIS_SLEW_MAX      520.0f    /* 退出视觉最大变化率 */
#define RA_EXIT_LOST_OUTER_DUTY   850.0f    /* 退出丢线外侧duty */
#define RA_EXIT_LOST_INNER_DUTY   650.0f    /* 退出丢线内侧duty */
#define RA_FAST_DIRECT_YAW_OFFSET 0.0f      /* 高速直接yaw偏移 */
#define RA_HARD_YAW_MAX_DEG       92.0f     /* HARD最大yaw角度 */
#define RA_HARD_YAW_PREDICT_MS    20.0f     /* HARD yaw预测毫秒 */
#define RA_HARD_COAST_REMAIN_DEG 0.0f       /* HARD滑行剩余角度 */
#define RA_HARD_TIMEOUT_REMAIN_DEG 2.0f     /* HARD超时剩余角度 */
#define RA_HARD_COAST_YAW_RATE 520.0f       /* HARD滑行yaw速率 */
#define RA_YAW_LOCK_FRAMES        12u       /* YAW_LOCK帧数 */
#define RA_YAW_LOCK_SPEED_PCT     58        /* YAW_LOCK速度百分比 */
#define RA_YAW_LOCK_DUTY_MIN      360.0f    /* YAW_LOCK最小duty */
#define RA_YAW_LOCK_DUTY_MAX      1400.0f   /* YAW_LOCK最大duty */
#define RA_YAW_LOCK_EXTRA_DEG     0.0f      /* YAW_LOCK额外角度 */
#define RA_YAW_LOCK_FINAL_DEG     90.0f     /* YAW_LOCK最终角度 */
#define RA_YAW_LOCK_TARGET_MARGIN_DEG 1.0f  /* YAW_LOCK目标边距 */
#define RA_YAW_LOCK_RATE_DONE     70.0f     /* YAW_LOCK完成速率 */
#define RA_YAW_LOCK_BRAKE_RATE    55.0f     /* YAW_LOCK刹车速率 */
#define RA_YAW_LOCK_BRAKE_KD      3.5f      /* YAW_LOCK刹车微分系数 */
#define RA_YAW_LOCK_BRAKE_MAX     1400.0f   /* YAW_LOCK刹车最大值 */
#define RA_CURVE_PID_TURN_ENABLE  0u        /* 弯道PID转弯开关 */
#define RA_COMPLEX_CURVE_PID_ENABLE 1u      /* 复杂弯道PID开关 */
#define RA_CURVE_PID_SPEED_PCT    58        /* 弯道PID速度百分比 */
#define RA_CURVE_PID_EXIT_REMAIN_DEG 0.8f   /* 弯道PID退出剩余角度 */
#define RA_CURVE_PID_MIN_FRAMES   3u        /* 弯道PID最小帧数 */
#define RA_CURVE_PID_MAX_FRAMES   44u       /* 弯道PID最大帧数 */
#define RA_COMPLEX_CURVE_PID_MAX_FRAMES 36u /* 复杂弯道PID最大帧数 */
#define RA_COMPLEX_WAIT_SPEED_PCT 82        /* 复杂等待速度百分比 */
#define RA_COMPLEX_CURVE_PID_SPEED_PCT 56   /* 复杂弯道PID速度百分比 */
#define RA_COMPLEX_CURVE_PID_EXIT_REMAIN_DEG 0.8f /* 复杂弯道PID退出剩余角度 */
#define RA_COMPLEX_CURVE_INNER_ERR_BIAS 36  /* 复杂弯道内侧误差偏差 */
#define RA_COMPLEX_CURVE_PID_LINE_EXIT_MIN_YAW_DEG 86.0f
#define RA_COMPLEX_CURVE_PID_TIMEOUT_MIN_YAW_DEG 86.0f
#define RA_COMPLEX_CURVE_LOCK_MIN_YAW_DEG 86.0f
#define RA_COMPLEX_CURVE_ASSIST_PCT 145     /* 复杂弯道辅助百分比 */
#define RA_COMPLEX_CURVE_ASSIST_LATE_MIN_PCT 90 /* 复杂弯道晚期最小辅助百分比 */
#define RA_COMPLEX_CURVE_ASSIST_RATE_MIN_PCT 72
#define RA_CURVE_PID_LINE_EXIT_MIN_FRAMES 7u
#define RA_CURVE_PID_LINE_EXIT_MIN_YAW_DEG 86.0f
#define RA_CURVE_PID_TIMEOUT_MIN_YAW_DEG 86.0f
#define RA_CURVE_PID_LINE_VALID_ROWS 30u    /* 弯道PID行有效行数 */
#define RA_CURVE_PID_LINE_ERR_MAX 16        /* 弯道PID行最大误差 */
#define RA_CURVE_PID_LINE_LA_MAX 22         /* 弯道PID行最大前瞻误差 */
#define RA_CURVE_PID_LINE_TREND_MAX 24      /* 弯道PID行最大趋势误差 */
#define RA_CURVE_PID_LINE_TAKEOVER_YAW_DEG 72.0f
#define RA_CURVE_PID_TAKEOVER_BIAS_PCT 0
#define RA_CURVE_PID_TAKEOVER_ASSIST_PCT 0
#define RA_CURVE_ASSIST_ENABLE   0u         /* 弯道辅助开关 */
#define RA_CURVE_ASSIST_MIN      900.0f     /* 弯道辅助最小值 */
#define RA_CURVE_ASSIST_MAX      2600.0f    /* 弯道辅助最大值 */
#define RA_CURVE_ASSIST_SPEED_END 1600      /* 弯道辅助速度终点 */
#define RA_CURVE_ASSIST_TAPER_DEG 24.0f     /* 弯道辅助锥度角度 */
#define RA_CURVE_ASSIST_RATE_LIMIT 520.0f   /* 弯道辅助速率限制 */
#define RA_CURVE_ASSIST_RATE_MIN_PCT 35     /* 弯道辅助速率最小百分比 */
#define RA_CURVE_SLOW_ASSIST_PCT 115        /* 弯道慢速辅助百分比 */
#define RA_CURVE_INNER_ERR_BIAS  46         /* 弯道内侧误差偏差 */
#define RA_CURVE_YAW_GUARD_DEG   89.0f      /* 弯道yaw保护角度 */
#define RA_CURVE_YAW_GUARD_PREDICT_MS 60.0f /* 弯道yaw保护预测毫秒 */
#define RA_CURVE_YAW_BRAKE_KP    55.0f      /* 弯道yaw刹车比例系数 */
#define RA_CURVE_YAW_BRAKE_KD    1.1f       /* 弯道yaw刹车微分系数 */
#define RA_CURVE_YAW_BRAKE_MAX   1200.0f    /* 弯道yaw刹车最大值 */
#define RA_RECOVER_YAW_BRAKE_MAX 520.0f     /* RECOVER yaw刹车最大值 */
#define RA_SLOW_BEFORE_TURN_ROWS 38u        /* 转弯前行数 */
#define RA_FAST_SLOW_MENU_MAX_ROW 38u       /* 高速时菜单SlwRow上限 */
#define RA_SLOW_TO_HARD_FALLBACK_FRAMES 1u  /* SLOW到HARD回退帧数 */
#define RA_SLOW_TO_APPROACH_FALLBACK_FRAMES 2u
#define RA_COMPLEX_SLOW_TO_APPROACH_FALLBACK_FRAMES 3u
#define RA_CURVE_SLOW_TO_APPROACH_FALLBACK_FRAMES 8u
#define RA_CURVE_FALLBACK_VALID_ROWS 44u    /* 弯道回退有效行数 */
#define RA_APPROACH_SPEED_PCT    55         /* APPROACH速度百分比 */
#define RA_HARD_INNER_DUTY       -60.0f     /* HARD内侧duty（反转） */
#define RA_VOLT_COMP_ENABLE      1u         /* 电压补偿开关 */
#define RA_VOLT_REF_X10          120u       /* 电压参考x10：12.0V */
#define RA_VOLT_COMP_MIN_PCT     90         /* 电压补偿最小百分比 */
#define RA_VOLT_COMP_MAX_PCT     112        /* 电压补偿最大百分比 */
#define RA_PIVOT_OUTER_MIN_DUTY  1250.0f    /* 枢轴外侧最小duty */
#define RA_COMPLEX_PIVOT_OUTER_MIN_DUTY 1500.0f /* 复杂枢轴外侧最小duty */
#define RA_PIVOT_TAPER_REMAIN_DEG 36.0f     /* 枢轴锥度剩余角度 */
#define RA_PIVOT_TAPER_MIN_PCT   18         /* 枢轴锥度最小百分比 */
#define RA_PIVOT_YAW_RATE_SOFT_LIMIT 340.0f /* 枢轴yaw速率软限制 */
#define RA_PIVOT_YAW_RATE_MIN_PCT 32        /* 枢轴yaw速率最小百分比 */
#define RA_HARD_RATE_MIN          230.0f    /* HARD速率最小值 */
#define RA_HARD_RATE_LIMIT        650.0f    /* HARD速率限制 */
#define RA_FAST_HARD_RATE_BOOST   60.0f     /* 高速HARD速率提升 */
#define RA_COMPLEX_HARD_RATE_PCT  88        /* 复杂HARD速率百分比 */
#define RA_HARD_RATE_TAPER_REMAIN_DEG 5.0f  /* HARD速率锥度剩余角度 */
#define RA_HARD_RATE_STOP_REMAIN_DEG 1.0f   /* HARD速率停止剩余角度 */
#define RA_HARD_RATE_KP           3.0f      /* HARD速率比例系数 */
#define RA_HARD_DIFF_MAX_PCT      170       /* HARD差速最大百分比 */
#define RA_HARD_BRAKE_DIFF_MAX    300.0f    /* HARD刹车差速最大值 */
#define RA_HARD_DIFF_SLEW_MAX     1180.0f   /* HARD差速变化率最大值 */
#define RA_HARD_LINE_TRIM_ENABLE 1u         /* HARD行修整开关 */
#define RA_HARD_LINE_VALID_ROWS  42u        /* HARD行有效行数 */
#define RA_HARD_LINE_ERR_KP      2.0f      /* HARD行误差比例系数 */
#define RA_HARD_LINE_LA_KP       1.2f       /* HARD行前瞻比例系数 */
#define RA_HARD_LINE_TRIM_MAX    160.0f     /* HARD行修整最大值 */
#define RA_HARD_OUTER_PCT        62         /* HARD外侧百分比 */
#define RA_FAST_HARD_OUTER_PCT   68         /* 高速HARD外侧百分比 */
#define RA_COMPLEX_INNER_REVERSE_PCT 70     /* 复杂内侧反转百分比 */
#define RA_COMPLEX_INNER_REVERSE_MIN_DUTY 160.0f /* 复杂内侧反转最小duty */
#define RA_COMPLEX_INNER_REVERSE_MAX_DUTY 320.0f /* 复杂内侧反转最大duty */
#define RA_COMPLEX_PIVOT_OUTER_MAX_DUTY 1850.0f /* 复杂枢轴外侧最大duty */
#define RA_FIXED_HARD_ROW_ENABLE 1u         /* 固定HARD行开关 */
#define RA_FIXED_HARD_ROW        30u        /* 固定HARD行（直角） */
#define RA_FIXED_COMPLEX_HARD_ROW 68u       /* 固定复杂HARD行 */
#define RA_DIRECT_TURN_ROW_OFFSET 0u        /* 直接转弯行偏移 */
#define RA_COMPLEX_TURN_ROW_OFFSET 12u      /* 复杂转弯行偏移 */
#define RA_FAST_TURN_ROW_ADVANCE 0u        /* 高速转弯行提前 */
#define RA_FAST_TURN_ROW_ADVANCE_MAX 0u    /* 高速转弯行提前最大值 */
#define RA_FAST_TURN_ROW_ADVANCE_SPEED_END 1700
#define RA_TURN_ROW_MIN         18u         /* 转弯行最小值 */
#define RA_PRE_DIRECT_IMMEDIATE_HARD_SPEED_MAX 0 /* 高速预识别直角不立刻HARD */
#define RA_ROUTE_PRE_HARD_ENABLE 0u         /* 路线预HARD开关 */
#define RA_ROUTE_PRE_HARD_VALID_ROWS 35u    /* 路线预HARD有效行数 */
#define RA_ROUTE_PRE_HARD_LOOKAHEAD_MIN 18  /* 路线预HARD最小前瞻误差 */
#define RA_ROUTE_PRE_HARD_IP_ROW 55u        /* 路线预HARD拐点行 */
#define RA_ROUTE_DIRECT_EARLY_ENABLE 0u
#define RA_ROUTE_DIRECT_EARLY_FIRST_ONLY 1u
#define RA_ROUTE_DIRECT_EARLY_VALID_ROWS_MIN 28u
#define RA_ROUTE_DIRECT_EARLY_VALID_ROWS_MAX 44u
#define RA_ROUTE_DIRECT_EARLY_ERR_MAX 18
#define RA_ROUTE_DIRECT_EARLY_LA_MIN 2
#define RA_ROUTE_DIRECT_EARLY_IP_ROW 56u
#define RA_ROUTE_DIRECT_EARLY_MIN_TICKS PID_MS_TO_TICKS(500u)
#define RA_PRE_DIRECT_HARD_IP_ROW 24u       /* 预直接HARD最小拐点行 */
#define RA_PRE_DIRECT_HARD_FRAMES 1u        /* 预直接HARD帧数 */
#define RA_PRE_DIRECT_NO_IP_ENABLE 0u       /* allow strong pre-turn without IP row */
#define RA_PRE_DIRECT_NO_IP_ROW 58u         /* equivalent row for no-IP pre-turn */
#define RA_PRE_DIRECT_NO_IP_VALID_ROWS 38u  /* no-IP pre-turn max valid rows */
#define RA_PRE_DIRECT_NO_IP_LA_MIN 2        /* no-IP pre-turn min lookahead */
#define RA_PRE_DIRECT_STRAIGHT_VALID_ROWS 42u
#define RA_PRE_DIRECT_STRAIGHT_ERR_MAX 12
#define RA_PRE_DIRECT_STRAIGHT_LA_MAX 14
#define RA_PRE_DIRECT_LEFT_STRAIGHT_VALID_ROWS 40u
#define RA_PRE_DIRECT_LEFT_STRAIGHT_ERR_MAX 18
#define RA_PRE_DIRECT_LEFT_STRAIGHT_LA_MAX 22
#define RA_LOW_TURN_ROW_DELAY    0u         /* 低速转弯行延迟 */
#define RA_LATE_APPROACH_SKIP_ROW_MARGIN 2u
#define RA_FAST_APPROACH_FRAMES  1u         /* 高速APPROACH帧数 */
#define RA_LOW_APPROACH_FRAMES   1u         /* 低速APPROACH帧数 */
#define RA_PRE_TURN_VALID_ROWS   12u        /* 预转弯有效行数 */
#define RA_PRE_TURN_ROW_ADVANCE  88u        /* 预转弯行提前 */
#define RA_PRE_TURN_ROW_SPAN     80u        /* 预转弯行跨度 */
#define RA_PRE_ROUTE_IP_ROW      70u        /* 路线预判无IP时使用的等效行 */
#define RA_PRE_TURN_SPEED_END    2200       /* 预转弯速度终点 */
#define RA_PRE_TURN_STEER_MAX    2200.0f    /* 预转弯最大转向 */
#define RA_PRE_TURN_SLEW_MAX     900.0f     /* 预转弯最大变化率 */
#define RA_PRE_TURN_ENABLE       0          /* 预转弯开关 */
#define RA_PRE_TURN_STEER_GUARD_ENABLE 1u   /* 预转弯转向保护开关 */
#define RA_PRE_TURN_NORMAL_STEER_PCT 100    /* 预转弯正常转向百分比 */
#define RA_PRE_TURN_GUARD_STEER_MIN 0.0f    /* 预转弯保护最小转向 */
#define RA_PRE_TURN_GUARD_STEER_MAX 1400.0f /* 预转弯保护最大转向 */
#define RA_PRE_TURN_GUARD_START_ROW 8u      /* 预转弯保护起始行 */
#define RA_PRE_TURN_GUARD_ERR_MAX 58        /* 预转弯保护最大误差 */
#define RA_PRE_TURN_GUARD_LA_MAX 76         /* 预转弯保护最大前瞻误差 */
#define RA_SINGLE_EDGE_PRE_DIRECT_IP_ROW 50u
#define RA_SINGLE_EDGE_PRE_DIRECT_MIN_TICKS PID_MS_TO_TICKS(360u)
#define RA_SINGLE_EDGE_PRE_DIRECT_LONG_FRAMES 14u
#define RA_COMPLEX_DUTY_PCT       100       /* 复杂duty百分比 */
#define RA_STRAIGHT_FRAMES       24u        /* 直行帧数 */
#define RA_STRAIGHT_SPEED_PCT    100        /* 直行速度百分比 */
#define RA_POST_RECOVER_FRAMES   3u         /* 后恢复帧数 */
#define RA_POST_RECOVER_COMPLEX_FRAMES 12u  /* 复杂后恢复帧数 */
#define RA_POST_RECOVER_COMPLEX_SPEED_PCT 76 /* 复杂后恢复速度百分比 */
#define RA_POST_RECOVER_SPEED_PCT 95        /* 后恢复速度百分比 */
#define RA_POST_RECOVER_STEER_PCT 100       /* 后恢复转向百分比 */
#define RA_RECOVER_CHAIN_COMPLEX_MIN_FRAMES 1u /* 连续复杂路口最小恢复帧 */
#define RA_LOST_GUARD_FRAMES 16u            /* 丢线保护帧数 */
#define RA_INTER_START_VALID_ROWS 35u       /* 路口起始有效行数 */
#define RA_INTER_ROUTE_VALID_ROWS 16u       /* 路口路线有效行数 */
#define RA_INTER_START_ERR_MAX    40        /* 路口起始最大误差 */
#define RA_INTER_START_LA_MAX     48        /* 路口起始最大前瞻误差 */
#define RA_INTER_START_TREND_MAX  48        /* 路口起始最大趋势误差 */
#define RA_INTER_ROUTE_LA_MAX     28        /* 路口路线最大前瞻误差 */
#define RA_INTER_ROUTE_TREND_MAX  34        /* 路口路线最大趋势误差 */
#define RA_INTER_COMPLEX_ROUTE_VALID_ROWS 12u
#define RA_INTER_COMPLEX_ROUTE_ERR_MAX 42
#define RA_INTER_COMPLEX_ROUTE_LA_MAX 42
#define RA_INTER_COMPLEX_ROUTE_TREND_MAX 46
#define RA_INTER_COMPLEX_LAST_CHANCE_ROW 42u
#define RA_INTER_COMPLEX_LAST_ERR_MAX 56
#define RA_INTER_COMPLEX_LAST_LA_MAX 38
#define RA_INTER_COMPLEX_LAST_TREND_MAX 58
#define RA_INTER_COMPLEX_LOST_FALLBACK_ENABLE 1u
#define RA_INTER_COMPLEX_LOST_IP_ROW 70u
#define RA_INTER_COMPLEX_LOST_FRAMES 1u
#define RA_INTER_COMPLEX_CONF_MIN 60u       /* 复杂路口最小置信度 */
#define RA_INTER_COMPLEX_CONF_LAST_MIN 48u
#define RA_INTER_COMPLEX_CONF_CUTOFF_MIN 38u
#define RA_INTER_COMPLEX_CONF_PENDING_MIN 45u
#define RA_INTER_COMPLEX_START_VALID_ROWS 18u
#define RA_INTER_COMPLEX_START_ERR_MAX 56
#define RA_INTER_COMPLEX_START_LA_MAX 58
#define RA_INTER_COMPLEX_START_TREND_MAX 80
#define RA_INTER_COMPLEX_RECENT_LOST_FRAMES 8u
#define RA_INTER_COMPLEX_RECOVER_VALID_ROWS 36u
#define RA_INTER_COMPLEX_RECOVER_ERR_MAX 36
#define RA_INTER_COMPLEX_RECOVER_LA_MAX 40
#define RA_INTER_COMPLEX_RECOVER_TREND_MAX 48
#define RA_ROUTE_COMPLEX_PRE_ENABLE 0u
#define RA_ROUTE_COMPLEX_PRE_VALID_ROWS 42u
#define RA_ROUTE_COMPLEX_PRE_ERR_MAX 36
#define RA_ROUTE_COMPLEX_PRE_LA_MIN 18
#define RA_ROUTE_COMPLEX_PRE_TREND_MIN 12
#define RA_ROUTE_COMPLEX_PRE_IP_ROW 52u
#define RA_PENDING_COMPLEX_HOLD_FRAMES 20u  /* 待定复杂保持帧数 */
#define RA_PENDING_COMPLEX_BRIDGE_RECOVER_FRAMES 1u
#define RA_PENDING_COMPLEX_IP_ROW 60u       /* 待定复杂拐点行 */
#define RA_PENDING_COMPLEX_VALID_ROWS 18u   /* 待定复杂有效行数 */
#define RA_PENDING_COMPLEX_ERR_MAX 56       /* 待定复杂最大误差 */
#define RA_PENDING_COMPLEX_LA_MAX 70        /* 待定复杂最大前瞻误差 */
#define RA_PENDING_COMPLEX_TREND_MAX 58     /* 待定复杂最大趋势误差 */
#define RA_KEEP_COMPLEX_VALID_ROWS 24u     /* 保持复杂有效行数 */
#define RA_KEEP_COMPLEX_ERR_MAX 56         /* 保持复杂最大误差 */
#define RA_KEEP_COMPLEX_LA_MAX 70          /* 保持复杂最大前瞻误差 */
#define RA_KEEP_COMPLEX_TREND_MAX 58       /* 保持复杂最大趋势误差 */
#define RULES_DONE_DELAY         136u       /* 路线完成延迟帧数 */

/* ========================================================================
 * 直角和丢线保底参数
 * ======================================================================== */

#define RA_FALLBACK_DIRECT_ENABLE 0u        /* 直接回退开关 */

/* ========================================================================
 * 丢线搜索参数
 * ======================================================================== */

#define LOST_SEARCH_ENTER_FRAMES 60u        /* 丢线搜索进入帧数：连续丢线60帧后启动 */
#define LOST_SEARCH_SWITCH_FRAMES 18u       /* 搜索方向切换帧数：每18帧切换方向 */
#define LOST_SEARCH_EXIT_VALID_ROWS 15u     /* 退出有效行数：找回15行有效行则退出搜索 */
#define LOST_SEARCH_DUTY 100.0f             /* 搜索差速duty */
#define LOST_SEARCH_FORWARD_DUTY 850.0f     /* 搜索前进duty */
#define LOST_SEARCH_ERR_DEADZONE 4          /* 搜索误差死区 */

/* ========================================================================
 * 单边巡线参数
 * ======================================================================== */

#define EDGE_BOTH  0u                       /* 双边模式（正常巡线） */
#define EDGE_LEFT  1u                       /* 单边左侧模式（仅用左边缘） */
#define EDGE_RIGHT 2u                       /* 单边右侧模式（仅用右边缘） */
#define EDGE_AUTO  3u                       /* 自动模式（由RA方向决定） */
#define SINGLE_EDGE_POST_TURN_MS 500u       /* 转弯后单边持续时间（毫秒） */
#define SINGLE_EDGE_UNTIL_NEXT_TURN 0xFFFFu /* 特殊值：保持到下一次真正转弯 */

extern uint8 g_post_edge_side;              /* 当前单边巡线方向（全局变量） */

/* ========================================================================
 * 串级PID参数（Cascade PID）
 * ========================================================================
 * 串级PID结构：
 *   外环：图像位置误差 -> 目标yaw速率
 *   内环：目标yaw速率 - 实际yaw_rate -> 电机duty差速
 * 优点：内环yaw_rate响应快，外环位置控制稳定。
 * ======================================================================== */

#define CAS_POS_KP          3.0f            /* 外环位置比例系数 */
#define CAS_TREND_KD        0.86f           /* 外环趋势微分系数 */
#define CAS_LA_K            0.78f           /* 外环前瞻系数 */
#define CAS_TARGET_MAX      180.0f          /* 外环目标yaw速率限幅（度/秒） */
#define CAS_TARGET_FILTER   0.42f           /* 外环目标低通滤波系数 */
#define CAS_YAW_KP_SCALE    0.25f           /* 内环比例缩放 */
#define CAS_YAW_KD_SCALE    0.12f           /* 内环微分缩放 */
#define CAS_DEADZONE_DPS    4.0f            /* 内环yaw速率死区（度/秒） */
#define CAS_INTEGRAL_GAIN   0.030f
#define CAS_INTEGRAL_LIMIT  80.0f
#define CAS_INTEGRAL_DECAY  0.92f
#define CAS_INTEGRAL_VALID_ROWS 28u
#define CAS_INTEGRAL_ERR_MAX 42.0f
#define CAS_INTEGRAL_LA_MAX 48
#define YAW_DAMP_SCALE      0.18f           /* 独立yaw阻尼缩放 */

/* ========================================================================
 * 接口函数声明
 * ======================================================================== */

void line_pid_init(void);                   /* PID控制器初始化 */
void line_pid_control(void);                /* PID控制主函数（8ms中断调用） */
void line_pid_reset_derivative(void);       /* 完全重置微分状态 */
void start_single_edge(uint8 side, uint16 duration_ms); /* 启动单边巡线 */
uint8 route_next_flag_is(uint8 flag);       /* 检查下一个路线flag */
uint8 route_next_expected_flag(void);       /* 获取下一个期望flag */
uint8 route_next_turn_dir(uint8 flag);      /* 获取flag对应的转弯方向 */
uint8 ra_curve_turn_dir(void);              /* 获取弯道转弯方向 */
uint8 ra_curve_yaw_takeover_ready(void);    /* 检查yaw接管就绪 */
uint8 ra_curve_ip_row(void);                /* 获取弯道拐点行 */
uint8 ra_exit_bezier_turn_dir(void);        /* 获取贝塞尔退出转弯方向 */
uint8 ra_exit_bezier_ip_row(void);          /* 获取贝塞尔退出拐点行 */

#endif /* CODE_PID_H_ */
