#include "Pid.h"                /* PID控制器头文件，包含所有PID相关常量和变量声�?*/
#include "Menu.h"               /* 菜单系统头文件，提供菜单变量（如motor_speed等） */
#include "Track_funsion.h"      /* 视觉融合头文件，提供g_tf视觉状态结构体 */
#include "IMU.h"                /* IMU头文件，提供yaw_angle、yaw_rate等IMU数据 */
#include "AutoTuneLog.h"

/* ======================== 全局调试变量 ======================== */

/* 基础速度，由 plan_base_speed() 规划后写入，单位为菜单设定的速度档位�?*/
int16 base_speed = 0;           /* 基础速度值，PID控制的核心速度输入 */
/* 速度环调试输出值（含转向叠加后的左右轮差速中间量�?*/
int16 speed_dbg_out = 0;        /* 速度调试输出，供TFT显示�?*/
/* 转向调试输出值（纯转向分量） */
int16 steer_dbg_out = 0;        /* 转向调试输出，供TFT显示�?*/
int16 duty_dbg_left = 0;        /* Last left duty sent to motor driver */
int16 duty_dbg_right = 0;       /* Last right duty sent to motor driver */
/* 视觉质量降速百分比�?00=无降速），用于TFT调试显示 */
uint8 speed_dbg_vq_pct = 100u;  /* 视觉质量降速百分比�?00表示无降�?*/
/* 预检测锁定标志，用于TFT调试显示�?=正在预减速） */
uint8 speed_dbg_pre_lock = 0u;  /* 预减速锁定标志，1表示正在预减�?*/
/* 规划前的原始目标速度，用于TFT调试显示 */
int16 speed_dbg_raw = 0;        /* 原始目标速度，速度规划前的输入�?*/
/* 规划后的目标速度，用于TFT调试显示 */
int16 speed_dbg_plan = 0;       /* 规划后目标速度，速度规划输出�?*/
/* 速度规划原因编号，用于TFT调试显示�?=RA 2=丢线 3=视觉�?4=对称加�?5=前瞻 6=直道加�?7=质量 8=正常 9=单边加速） */
uint8 speed_dbg_reason = 0u;    /* 速度变化原因编号，用于调�?*/
/* 后转弯单边巡线方向，�?start_single_edge() 写入，Track_funsion.c 读取
 * EDGE_BOTH=双边, EDGE_LEFT=只用左边�? EDGE_RIGHT=只用右边�?*/
uint8 g_post_edge_side = EDGE_BOTH; /* 单边巡线方向，全局变量供视觉模块读�?*/

/* ======================== RA（直�?路口）调试变�?======================== */

/* RA状态机当前状态（0=空闲 1=活跃�?*/
uint8 ra_dbg_state = 0u;        /* RA状态调试变�?*/
/* RA当前阶段�?=WAIT 1=SLOW 2=APPROACH 3=HARD 4=RECOVER�?*/
uint8 ra_dbg_phase = 0u;        /* RA阶段调试变量 */
/* RA转弯方向�?=直行 1=右转 2=左转�?*/
uint8 ra_dbg_dir = 0u;          /* RA方向调试变量 */
/* RA记录的最大拐点行�?*/
uint8 ra_dbg_ip_row = 0u;       /* RA拐点行调试变�?*/
/* RA全局计时器（帧数），每PID周期+1 */
uint16 ra_dbg_timer = 0u;       /* RA全局计时器调试变�?*/
/* RA HARD阶段帧计�?*/
uint16 ra_dbg_hard_cnt = 0u;    /* RA硬转弯计数调试变�?*/
/* RA退出条件满足的连续帧数（camera_done判断用） */
uint8 ra_dbg_exit_good_cnt = 0u;/* RA退出计数调试变�?*/
/* RA yaw进度 x10，用于TFT显示（避免浮点） */
int16 ra_dbg_yaw10 = 0;         /* RA偏航进度调试变量（乘10避免浮点�?*/

/* ======================== 转向PD控制静态变�?======================== */

/* 上一次的位置误差（滤波后），用于D项计�?*/
static float s_steer_last_pos_err = 0.0f;   /* 上一周期滤波后误�?*/
/* 上一次的原始位置误差（未滤波），用于D项计�?*/
static float s_steer_last_raw_err = 0.0f;   /* 上一周期原始误差 */
/* 低通滤波后的位置误差，一阶IIR滤波器输�?*/
static float s_filtered_err = 0.0f;          /* 滤波后误差�?*/
/* 上一次的转向输出值，用于变化率限制（slew rate limiter�?*/
static float s_prev_steer_output = 0.0f;     /* 上一周期转向输出 */
/* 前馈信号的低通滤波值，避免前瞻突变导致转向抖动 */
static float s_steer_ff_filtered = 0.0f;     /* 滤波后的前馈信号 */
static float s_ra_pre_turn_ff = 0.0f;
/* D项重置标志，�?后下一个周期跳过D输出（防止切换时微分冲击�?*/
static uint8 s_steer_d_reset_flag = 0u;      /* 微分重置标志 */

/* ======================== 速度PI控制静态变�?======================== */

/* 速度PI控制器的积分累积�?*/
static float s_speed_integral = 0.0f;        /* 速度积分累积�?*/
/* 上一周期的目标速度，用于加速度前馈计算 */
static float s_prev_target_speed = 0.0f;     /* 上一周期目标速度 */
/* 速度前馈就绪标志，首周期跳过前馈（避免启动突变） */
static uint8 s_speed_ff_ready = 0u;          /* 前馈就绪标志 */
/* 电机运行帧计数器，用�?motor_run_time 超时停机 */
static uint32 s_motor_run_counter = 0;       /* 电机运行帧计�?*/
static uint8 s_vacuum_on = 0u;
static uint32 s_vacuum_duty = 0u;
static uint16 s_vacuum_prearm_ticks = 0u;
static uint8 s_vacuum_prearm_timeout = 0u;
volatile uint8 vacuum_enable = 0u;           /* 负压实际开启状态，供TFT显示/关屏逻辑使用 */

static void vacuum_apply_duty(uint32 duty)
{
    if (s_vacuum_duty != duty)
    {
        pwm_set_duty(VAC_PWM_CH, duty);
        s_vacuum_duty = duty;
    }

    s_vacuum_on = (duty != 0u) ? 1u : 0u;
    vacuum_enable = s_vacuum_on;
}

static void vacuum_stop(void)
{
    s_vacuum_prearm_ticks = 0u;
    s_vacuum_prearm_timeout = 0u;
    vacuum_apply_duty(0u);
}

static void vacuum_control_update(uint8 request_on, uint8 run_active)
{
#if VACUUM_RUN_ENABLE
    uint32 duty;

    if (request_on == 0u)
    {
        vacuum_stop();
        return;
    }

    if (run_active != 0u)
    {
        s_vacuum_prearm_ticks = 0u;
        s_vacuum_prearm_timeout = 0u;
    }
    else
    {
        if (s_vacuum_prearm_timeout != 0u)
        {
            vacuum_apply_duty(0u);
            return;
        }

        if (s_vacuum_prearm_ticks >= VAC_PREARM_TIMEOUT_TICKS)
        {
            s_vacuum_prearm_timeout = 1u;
            vacuum_apply_duty(0u);
            return;
        }

        s_vacuum_prearm_ticks++;
    }

    if (s_vacuum_duty == 0u)
    {
        duty = (uint32)VAC_PWM_RAMP_START_DUTY;
    }
    else if (s_vacuum_duty < (uint32)VAC_PWM_DUTY)
    {
        duty = s_vacuum_duty + (uint32)VAC_PWM_RAMP_STEP;
        if (duty > (uint32)VAC_PWM_DUTY)
            duty = (uint32)VAC_PWM_DUTY;
    }
    else
    {
        duty = (uint32)VAC_PWM_DUTY;
    }

    vacuum_apply_duty(duty);
#else
    (void)request_on;
    (void)run_active;
    vacuum_stop();
#endif
}

uint8 vacuum_ready_to_run(void)
{
#if VACUUM_RUN_ENABLE
    if (s_vacuum_prearm_timeout != 0u)
        return 0u;
    return (s_vacuum_duty >= (uint32)VAC_PWM_DUTY) ? 1u : 0u;
#else
    return 1u;
#endif
}

/* ======================== 速度规划静态变�?======================== */

/* 经过斜坡处理后的基础速度，防止速度突变 */
static int16 s_base_speed_ramped = 0;        /* 斜坡处理后的速度 */
/* 直道条件满足的连续帧计数，用于直道加速确�?*/
static uint8 s_straight_cnt = 0u;            /* 直道确认帧计�?*/
/* 直道加速激活标志（1=当前处于直道加速模式） */
static uint8 s_straight_active = 0u;         /* 直道加速激活标�?*/
/* 上一帧的误差值，用于误差跳变检测（视觉质量降速） */
static int16 s_prev_quality_err = 0;         /* 上一帧误差�?*/
/* 上一帧误差值有效标�?*/
static uint8 s_prev_quality_err_valid = 0u;  /* 上一帧误差有效标�?*/
static uint8 s_straight_hold = 0u;
static uint8 s_pre_lock = 0u;
static uint8 s_pre_timeout = 0u;
static int16 speed_ramp_apply_reason(int16 target, uint8 reason);

/* ======================== 转弯屏蔽（turn shield）静态变�?======================== */

/* 转弯屏蔽剩余帧数�?0时屏蔽RA检测，防止出弯后误触发 */

/* ======================== 单边巡线静态变�?======================== */

/* 单边巡线激活标�?*/
static uint8 s_edge_active = 0u;             /* 单边巡线激活标�?*/
/* 当前单边巡线方向（EDGE_LEFT / EDGE_RIGHT�?*/
static uint8 s_edge_side = EDGE_BOTH;        /* 单边巡线方向 */
/* 单边巡线已运行帧计数 */
static uint16 s_edge_cnt = 0u;               /* 单边巡线当前帧计�?*/
/* 单边巡线目标帧数（由时间ms转换�?*/
static uint16 s_edge_target = 0u;            /* 单边巡线目标帧数 */
/* 单边巡线是否保持到下一次真正转弯RA启动 */
static uint8 s_edge_until_next_turn = 0u;    /* 1=不按时间结束，等下一次非直行RA完成 */
/* 保持模式已遇到下一次真正转弯，等该RA结束后关闭单�?*/
static uint8 s_edge_release_after_turn = 0u; /* 1=当前非直行RA结束后恢复双�?*/
static uint8 s_single_edge_fast_hold = 0u;
static uint8 s_completed_right_ra_count = 0u;

/* ======================== 丢线搜索静态变�?======================== */

/* 丢线搜索激活标志（1=正在执行原地转向搜索�?*/
static uint8 s_lost_search_active = 0u;      /* 丢线搜索激活标�?*/
/* 连续丢线帧计数，达到阈值后启动搜索 */
static uint8 s_lost_line_cnt = 0u;           /* 连续丢线帧计�?*/
/* 丢线搜索已运行帧计数 */
static uint16 s_lost_search_cnt = 0u;        /* 丢线搜索运行帧计�?*/
/* 丢线搜索方向�?=向右�?2=向左转） */
static uint8 s_lost_search_dir = 1u;         /* 丢线搜索方向 */
/* 丢线前的最后误差值，用于选择搜索方向 */
static int16 s_lost_last_err = 0;            /* 丢线前最后误�?*/

/* ======================== 串级PID（cascade）内环状�?======================== */

/* Cascade外环输出的目标角速度，经低通滤波后送给内环 */
static float s_cas_target_filtered = 0.0f;   /* 串级PID目标角速度（滤波后�?*/
/* 内环上一帧的角速度误差，用于内环D�?*/
static float s_cas_last_yaw_err    = 0.0f;   /* 内环上一帧角速度误差 */

/* Cascade菜单变量（在Menu.c中可调） */
/* 串级PID使能标志�?=普通PD 1=串级IMU角速度环） */
int16 cascade_en    = 1;                     /* 串级PID使能标志 */
/* 内环角速度D增益（菜单可调） */
int16 yaw_kd        = 3;                     /* 内环角速度微分增益 */
/* 独立yaw阻尼增益（cascade_en=0时的角速度阻尼�?=关闭�?*/
int16 yaw_damp_gain = 0;                     /* 独立偏航阻尼增益 */

/* ======================== RA状态机枚举 ======================== */

/* RA状态：无活�?/ 有活�?*/
typedef enum { RA_ST_NONE, RA_ST_ACTIVE } RaState;  /* RA状态枚�?*/
/* RA阶段：等待拐点接�?�?减�?�?接近 �?硬转�?�?恢复 */
typedef enum { RA_PH_WAIT, RA_PH_SLOW, RA_PH_APPROACH, RA_PH_HARD, RA_PH_RECOVER } RaPhase; /* RA阶段枚举 */

/* RA状态机当前状�?*/
static RaState s_ra_state = RA_ST_NONE;      /* RA当前状态，初始为空�?*/
/* RA状态机当前阶段 */
static RaPhase s_ra_phase = RA_PH_WAIT;      /* RA当前阶段，初始为等待 */
/* RA转弯方向�?=直行通过 1=右转 2=左转�?*/
static uint8 s_ra_dir = 0u;                  /* RA转弯方向 */
/* 触发RA的原始flag值（1~5），用于查路线表和超时配�?*/
static uint8 s_ra_orig_flag = 0u;            /* 原始触发flag */
static int16 s_ra_speed_ref_latched = 0;
/* RA记录的拐点最大行号，用于阶段转换判断 */
static uint8 s_ra_ip_row = 0u;               /* 拐点最大行�?*/
/* 直行通过标志�?=该路口直行不转弯�?*/
static uint8 s_ra_straight = 0u;             /* 直行通过标志 */
/* RA结束后启用的单边巡线方向 */
static uint8 s_ra_post_edge_side = EDGE_BOTH;/* 结束后单边巡线方�?*/
/* RA结束后单边巡线持续时�?ms) */
static uint16 s_ra_post_edge_ms = 0u;        /* 结束后单边巡线时�?*/
/* HARD阶段中满足退出条件的连续帧计�?*/
static uint8 s_ra_exit_good_cnt = 0u;        /* HARD退出条件满足帧计数 */
/* RECOVER阶段中满足恢复完成条件的连续帧计�?*/
static uint8 s_ra_recover_good_cnt = 0u;     /* RECOVER完成条件满足帧计�?*/
/* APPROACH阶段帧计�?*/
static uint16 s_ra_approach_cnt = 0u;        /* APPROACH阶段帧计�?*/
/* RA全局计时器，每PID周期+1，用于超时保�?*/
static uint16 s_ra_timer = 0u;               /* RA全局计时�?*/
/* HARD阶段帧计�?*/
static uint16 s_ra_hard_cnt = 0u;            /* HARD阶段帧计�?*/
/* RECOVER阶段帧计�?*/
static uint16 s_ra_recover_cnt = 0u;         /* RECOVER阶段帧计�?*/
/* 当前阶段内的帧计数（WAIT/SLOW用） */
static uint16 s_ra_phase_cnt = 0u;           /* 当前阶段帧计�?*/
/* HARD阶段的速度种子值，用于RECOVER阶段的速度平滑过渡 */
static float s_ra_hard_speed_seed = 0.0f;    /* HARD阶段速度种子 */
/* HARD阶段的转向种子值，用于RECOVER阶段的转向平滑过�?*/
static float s_ra_hard_steer_seed = 0.0f;    /* HARD阶段转向种子 */

/* 前向声明：int16取绝对�?*/
static int16 abs_i16(int16 v)
{
    if (v == (int16)(-32767 - 1))
        return 32767;
    return (v < 0) ? (int16)(-v) : v;
}

/* ======================== 路线规则定义 ======================== */

/* 路口动作类型 */
#define ACT_STRAIGHT 0u  /* 直行通过 */
#define ACT_RIGHT    1u  /* 右转 */
#define ACT_LEFT     2u  /* 左转 */
#define ACT_AUTO     3u  /* 自动（根据flag推断方向，用于直角） */

/* 路口规则结构�?*/
typedef struct
{
    uint8 count;            /* 第几次出现该类型flag时匹配此规则 */
    uint8 flag;             /* 路口类型flag�?=右直�?2=左直�?3/4/5=普通路口） */
    uint8 action;           /* 执行动作（ACT_STRAIGHT/RIGHT/LEFT/AUTO�?*/
    uint8 post_edge_side;   /* 转弯后单边巡线方向（EDGE_BOTH=不启用） */
    uint16 post_edge_ms;    /* 转弯后单边巡线持续时�?ms) */
} IntersectionRule;         /* 路口规则结构体定�?*/

/* 规则构造宏 */
#define RULE(count, flag, action) \
    { (count), (flag), (action), EDGE_BOTH, 0u }           /* 普通规则，无单�?*/
#define RULE_EDGE(count, flag, action, edge_side, edge_ms) \
    { (count), (flag), (action), (edge_side), (edge_ms) }   /* 指定单边方向 */
#define RULE_AUTO(count, flag, action, edge_ms) \
    { (count), (flag), (action), EDGE_AUTO, (edge_ms) }     /* 自动选单边方�?*/
#define RULE_RA(count, flag) \
    { (count), (flag), ACT_AUTO, EDGE_BOTH, 0u }            /* 直角自动，无单边 */
#define RULE_RA_AUTO(count, flag, edge_ms) \
    { (count), (flag), ACT_AUTO, EDGE_AUTO, (edge_ms) }     /* 直角自动+自动单边 */
#define RULE_RA_EDGE(count, flag, edge_side, edge_ms) \
    { (count), (flag), ACT_AUTO, (edge_side), (edge_ms) }   /* 直角自动+指定单边 */

/* 路线表：按图中黑线走向填写�? * RULE：执行指定动作，不开启单边巡线�? * RULE_AUTO：执行指定动作，转弯结束后自动选单边�? * RULE_EDGE：执行指定动作，结束后强制指定单边�? * RULE_RA：直角方向自动，不开启单边�? * RULE_RA_AUTO：直角方向自动，转完后自动选单边�? * RULE_RA_EDGE：直角方向自动，转完后强制指定单边�? * 直角类型�?=右直角，2=左直角。普通路口类型：3/4/5�?*/
static const IntersectionRule user_rules[] = {
    /* 如果某个直角出弯后需要单边，就在这里插入�?     * RULE_RA_AUTO(第几�? 1u�?u, 持续时间),
     * 例如：RULE_RA_AUTO(2u, 1u, 500u), */

    /* 当前最快路线：
     * 右直�?-> 右直�?-> 左直�?-> 4�?-> 5�?-> 5�?-> 4�?-> 4�?     * -> 5�?-> 3�?-> 3直行 -> 5�?-> 右直�?-> 右直角后单边�?*/
    RULE_RA(  1u, 1u),    /* �?个flag=1（右直角）：自动方向 */
    RULE(     1u, 4u, ACT_RIGHT),   /* �?个flag=4（T右）：右�?*/
    RULE_RA(  1u, 2u),    /* �?个flag=2（左直角）：自动方向 */
    RULE(     1u, 5u, ACT_LEFT),    /* �?个flag=5（十字）：左�?*/
    RULE(     2u, 5u, ACT_RIGHT),   /* �?个flag=5（十字）：右�?*/
    RULE(     2u, 4u, ACT_RIGHT),   /* �?个flag=4（T右）：右�?*/
    RULE(     3u, 4u, ACT_RIGHT),   /* �?个flag=4（T右）：右�?*/
    RULE(     3u, 5u, ACT_LEFT),    /* �?个flag=5（十字）：左�?*/
    RULE(     4u, 5u, ACT_LEFT),    /* �?个flag=5（十字）：左�?*/
    RULE(     1u, 3u, ACT_LEFT),    /* �?个flag=3（T左）：左�?*/
    RULE(     2u, 3u, ACT_STRAIGHT),/* �?个flag=3（T左）：直�?*/
    RULE(     5u, 5u, ACT_RIGHT),   /* �?个flag=5（十字）：右�?*/
    RULE_RA_EDGE(2u, 1u, EDGE_LEFT, SINGLE_EDGE_UNTIL_NEXT_TURN), /* �?个flag=1（右直角）：结束后靠左单�?*/
    RULE_RA_EDGE(3u, 1u, EDGE_LEFT, SINGLE_EDGE_UNTIL_NEXT_TURN), /* �?个flag=1（右直角）：结束后靠左单边直到下个真转弯 */
    RULE_RA(  4u, 1u),    /* �?个flag=1（右直角）：自动方向 */
};
/* 路线表总条目数 */
#define USER_RULE_COUNT (sizeof(user_rules) / sizeof(user_rules[0])) /* 路线表条目数 */

/* ======================== 路线匹配状�?======================== */

/* 各类型flag�?~6）的已匹配计数，用于按序匹配路线�?*/
static uint8 s_inter_count[7] = {0u};  /* 各flag类型已匹配次�?*/
/* 路线全部完成标志，置1后延迟停�?*/
static uint8 s_rules_done = 0u;        /* 路线完成标志 */
/* 路线完成后的延迟停机计时器（帧数�?*/
static uint16 s_rules_done_timer = 0u; /* 路线完成延迟停机计时�?*/
/* 路线调试：当前执行到第几步（�?开始） */
uint8 route_dbg_step = 0u;             /* 路线调试当前步数 */
/* 路线调试：路线表总步�?*/
uint8 route_dbg_total = (uint8)USER_RULE_COUNT; /* 路线调试总步�?*/
/* 路线调试：当前匹配的flag类型 */
uint8 route_dbg_flag = 0u;             /* 路线调试当前flag */
/* 路线调试：当前匹配的count�?*/
uint8 route_dbg_count = 0u;            /* 路线调试当前count */
/* 路线调试：当前执行的动作 */
uint8 route_dbg_action = ACT_STRAIGHT; /* 路线调试当前动作 */
/* 待提交的路线匹配结果有效标志（延迟一帧提交，避免影响当前帧RA�?*/
static uint8 s_route_pending_valid = 0u;   /* 待提交有效标�?*/
/* 待提交的flag类型 */
static uint8 s_route_pending_flag = 0u;    /* 待提交flag */
/* 待提交的count�?*/
static uint8 s_route_pending_count = 0u;   /* 待提交count */
/* 待提交的动作 */
static uint8 s_route_pending_action = ACT_STRAIGHT; /* 待提交动�?*/

/* 前向声明：更新RA调试信息 */
static void ra_debug_update(void);         /* 前向声明：更新RA调试变量 */

/* ======================== RA状态机返回结构�?======================== */

/* RA状态机每帧返回的结�?*/
typedef struct
{
    uint8 need_pid;     /* 1=需要继续执行PID控制�?=跳过PID */
    uint8 should_return;/* 1=RA已直接输出电机，本帧PID不执�?*/
    float speed_scale;  /* 速度缩放因子�?.0~1.0），用于降�?*/
} RaResult;             /* RA状态机返回结构�?*/

/* ======================== 路线决策结构�?======================== */

/* 路口决策结果 */
typedef struct
{
    uint8 action;           /* 动作类型（ACT_STRAIGHT/RIGHT/LEFT�?*/
    uint8 post_edge_side;   /* 转弯后单边方�?*/
    uint16 post_edge_ms;    /* 转弯后单边持续时�?*/
    uint8 valid;            /* 决策有效标志�?=未匹配路线表�?*/
} RouteDecision;            /* 路口决策结构�?*/

/* ======================== 转向调度结构�?======================== */

/* 根据速度和曲率计算的转向PD参数缩放 */
typedef struct
{
    float kp_scale;   /* 比例增益缩放因子 */
    float kd_scale;   /* 微分增益缩放因子 */
    float ff_scale;   /* 前馈增益缩放因子 */
    float slew_max;   /* 输出变化率限�?*/
} SteerSchedule;      /* 转向增益调度结构�?*/

/* ======================== 工具函数 ======================== */

/* abs_f - 浮点数取绝对�? * @v: 输入浮点�? * 返回: |v| */
static float abs_f(float v)                 /* 浮点绝对值函�?*/
{
    return (v >= 0.0f) ? v : -v;            /* 非负返回原值，负数取反 */
}

/* clamp_f - 浮点数限�? * @v: 输入�? * @min_v: 下限
 * @max_v: 上限
 * 返回: 限幅后的�?*/
static float clamp_f(float v, float min_v, float max_v) /* 浮点限幅函数 */
{
    if (v < min_v) return min_v;            /* 低于下限，返回下�?*/
    if (v > max_v) return max_v;            /* 高于上限，返回上�?*/
    return v;                               /* 在范围内，返回原�?*/
}

/* lerp_f - 线性插�? * @a: 起始值（t=0时返回）
 * @b: 终止值（t=1时返回）
 * @t: 插值系数（0.0~1.0�? * 返回: a + (b-a)*t */
static float lerp_f(float a, float b, float t) /* 线性插值函�?*/
{
    return a + (b - a) * t;                 /* 标准线性插值公�?*/
}

/* range_ratio_i16 - 将int16值映射到[start, end]区间的比例（0.0~1.0�? * @value: 输入�? * @start: 区间起点（返�?.0�? * @end: 区间终点（返�?.0�? * 返回: 0.0~1.0之间的比例�? * 用途：增益调度中的归一�?*/
static float range_ratio_i16(int16 value, int16 start, int16 end) /* 区间比例映射 */
{
    if (end <= start)                       /* 区间无效或单�?*/
        return (value >= end) ? 1.0f : 0.0f; /* 超过终点返回1，否�? */

    if (value <= start) return 0.0f;        /* 小于起点，返�? */
    if (value >= end) return 1.0f;          /* 大于终点，返�? */

    return (float)(value - start) / (float)(end - start); /* 线性比例计�?*/
}

/* ======================== 转向增益调度 ======================== */

/* steer_schedule_calc - 根据当前速度和弯道程度计算转向PD的增益缩放参�? * @pos_err_abs: 位置误差绝对�? * 返回: 包含kp_scale/kd_scale/ff_scale/slew_max的调度结�? *
 * 调度逻辑�? *   1. 取误�?前瞻/趋势中最大的作为弯道信号
 *   2. 速度越高，kp/kd适当降低（防高速振荡）
 *   3. 弯道越急，kp/kd适当增大（加强响应）
 *   4. 前馈在高速时才启�? *   5. 变化率限制随弯道程度略增 */
static SteerSchedule steer_schedule_calc(int16 pos_err_abs) /* 转向增益调度计算 */
{
    SteerSchedule s;                        /* 调度结果结构�?*/
    int16 la_abs = abs_i16(g_tf.lookahead_error);  /* 前瞻误差绝对�?*/
    int16 trend_abs = abs_i16(g_tf.error_trend);    /* 趋势误差绝对�?*/
    int16 curve_signal = pos_err_abs;       /* 初始弯道信号 = 位置误差 */

    /* 取三者最大值作为弯道信�?*/
    if (la_abs > curve_signal) curve_signal = la_abs;   /* 前瞻更大则替�?*/
    if (trend_abs > curve_signal) curve_signal = trend_abs; /* 趋势更大则替�?*/

    /* 速度因子：低速→0，高速→1 */
    float speed_t = range_ratio_i16((int16)base_speed,  /* 当前基础速度 */
                                    STEER_GAIN_SPEED_START, /* 速度下限180 */
                                    STEER_GAIN_SPEED_END);  /* 速度上限800 */
    /* 弯道因子：直道→0，急弯�? */
    float curve_t = range_ratio_i16(curve_signal,       /* 弯道信号 */
                                    STEER_GAIN_CURVE_T1, /* 弯道下限10 */
                                    STEER_GAIN_CURVE_T2);/* 弯道上限38 */
    /* 高速时的kp/kd缩放（通常kp略降，kd略增�?*/
    float kp_fast = lerp_f(1.0f, (float)STEER_FAST_KP_PCT * 0.01f, speed_t); /* 高速kp缩放 */
    float kd_fast = lerp_f(1.0f, (float)STEER_FAST_KD_PCT * 0.01f, speed_t); /* 高速kd缩放 */

    /* 最终kp = 高速kp线性过渡到弯道kp */
    s.kp_scale = lerp_f(kp_fast,                        /* 高速kp基准 */
                        (float)STEER_CURVE_KP_PCT * 0.01f, /* 弯道kp */
                        curve_t);                        /* 弯道因子插�?*/
    /* 最终kd = 高速kd线性过渡到弯道kd */
    s.kd_scale = lerp_f(kd_fast,                        /* 高速kd基准 */
                        (float)STEER_CURVE_KD_PCT * 0.01f, /* 弯道kd */
                        curve_t);                        /* 弯道因子插�?*/
    /* 前馈缩放：高速时才启用，乘以菜单可调系数 */
    s.ff_scale = range_ratio_i16((int16)base_speed,     /* 当前基础速度 */
                                 STEER_FF_SPEED_START,   /* 前馈启用速度下限 */
                                 STEER_FF_SPEED_END) *   /* 前馈启用速度上限 */
                 ((float)steer_ff_k * 0.01f);            /* 菜单可调前馈系数 */
    /* 变化率限制：弯道时略增（允许更快响应�?*/
    s.slew_max = STEER_SLEW_MAX * PID_DT_SCALE * lerp_f(0.85f, 1.20f, curve_t); /* 弯道时变化率略增 */

    if (s_straight_active)
    {
        s.kp_scale *= (float)STEER_STRAIGHT_KP_PCT * 0.01f;
        s.kd_scale *= (float)STEER_STRAIGHT_KD_PCT * 0.01f;
        s.ff_scale *= (float)STEER_STRAIGHT_FF_PCT * 0.01f;
        s.slew_max *= (float)STEER_STRAIGHT_SLEW_PCT * 0.01f;
    }

    return s;                               /* 返回调度结果 */
}


static int16 ra_speed_ref(void)
{
    int32 actual = ((int32)abs_i16(motor_value.receive_left_speed_data) +
                    (int32)abs_i16(motor_value.receive_right_speed_data)) / 2;
    int32 cmd = (int32)motor_speed * 8;
    int16 ref;

    if (cmd < 0) cmd = 0;
    if (cmd > 32767) cmd = 32767;
    if (actual > 32767) actual = 32767;

    ref = (int16)cmd;
    if (actual > (int32)ref) ref = (int16)actual;

    if (s_ra_state == RA_ST_ACTIVE && s_ra_speed_ref_latched > 0)
        ref = s_ra_speed_ref_latched;

    return ref;
}

static uint8 ra_turn_row_for_speed(void)
{
    int16 row = ra_turn_row;

    if (row < 0) row = 0;
    if (row > (int16)(TF_IMG_H - 1u)) row = (int16)(TF_IMG_H - 1u);
    return (uint8)row;
}

static uint8 ra_turn_trigger_row(void)
{
    uint8 turn_row = ra_turn_row_for_speed();

    if (s_ra_orig_flag >= 3u &&
        turn_row <= (uint8)(255u - RA_COMPLEX_TURN_ROW_OFFSET))
    {
        turn_row = (uint8)(turn_row + RA_COMPLEX_TURN_ROW_OFFSET);
    }
    else if (s_ra_orig_flag < 3u &&
             turn_row <= (uint8)(255u - RA_DIRECT_TURN_ROW_OFFSET))
    {
        turn_row = (uint8)(turn_row + RA_DIRECT_TURN_ROW_OFFSET);
    }

    return turn_row;
}

static uint8 ra_slow_trigger_row(void)
{
    uint8 turn_row = ra_turn_trigger_row();
    uint8 slow_row = (turn_row > RA_SLOW_BEFORE_TURN_ROWS) ?
                     (uint8)(turn_row - RA_SLOW_BEFORE_TURN_ROWS) :
                     0u;
    uint8 menu_slow_row = (ra_slow_row < 0) ? 0u : (uint8)ra_slow_row;

    if (slow_row < menu_slow_row)
        slow_row = menu_slow_row;

    return slow_row;
}

static uint16 ra_slow_limit_for_speed(void)
{
    return RA_SLOW_TIMEOUT;
}
static uint16 ra_approach_frames_for_speed(uint8 turn_row)
{
    uint16 frames = (ra_approach_frames < 1) ? 1u : (uint16)ra_approach_frames;
    (void)turn_row;
    return frames;
}
static float ra_pre_turn_steer_ff(void)
{
#if RA_PRE_TURN_ENABLE
    uint8 dir = 0u;
    uint8 ip_row = g_ip_max_row;
    int16 speed_ref;
    int16 turn_row;
    int16 start_row;
    int16 end_row;
    float row_t;
    float speed_t;
    float target = 0.0f;
    float delta;

    if (s_ra_state == RA_ST_NONE)
    {
        if (g_ra_flag == 0u && g_ra_pre_flag != 0u &&
            (g_ra_pre_dir == 1u || g_ra_pre_dir == 2u))
        {
            dir = g_ra_pre_dir;
        }
    }
    else if (s_ra_orig_flag < 3u &&
             (s_ra_phase == RA_PH_SLOW || s_ra_phase == RA_PH_APPROACH) &&
             (s_ra_dir == 1u || s_ra_dir == 2u))
    {
        dir = s_ra_dir;
        ip_row = s_ra_ip_row;
    }

    if (dir == 0u ||
        g_sym_component_flag != 0u ||
        g_tf.line_lost != 0u ||
        g_tf.valid_row_count < RA_PRE_TURN_VALID_ROWS)
    {
        target = 0.0f;
        goto slew_out;
    }

    speed_ref = base_speed;
    if (speed_ref < 0)
        speed_ref = 0;
    if (speed_ref < RA_FAST_SPEED_START)
    {
        target = 0.0f;
        goto slew_out;
    }

    turn_row = (int16)ra_turn_row_for_speed();
    start_row = turn_row - (int16)RA_PRE_TURN_ROW_ADVANCE;
    if (start_row < 0)
        start_row = 0;
    end_row = start_row + (int16)RA_PRE_TURN_ROW_SPAN;
    if (end_row <= start_row)
        end_row = start_row + 1;

    row_t = range_ratio_i16((int16)ip_row, start_row, end_row);
    if (row_t > 0.0f)
    {
        speed_t = range_ratio_i16(speed_ref,
                                  RA_FAST_SPEED_START,
                                  RA_PRE_TURN_SPEED_END);
        target = RA_PRE_TURN_STEER_MAX * row_t * speed_t;
        if (dir == 1u)
            target = -target;
    }

slew_out:
    delta = target - s_ra_pre_turn_ff;
    if (delta > RA_PRE_TURN_SLEW_MAX)
        delta = RA_PRE_TURN_SLEW_MAX;
    else if (delta < -RA_PRE_TURN_SLEW_MAX)
        delta = -RA_PRE_TURN_SLEW_MAX;

    s_ra_pre_turn_ff += delta;
    return s_ra_pre_turn_ff;
#else
    s_ra_pre_turn_ff = 0.0f;
    return 0.0f;
#endif
}

/* normalize_angle - 角度归一化到[-180, 180]范围
 * @angle: 输入角度（度�? * 返回: 归一化后的角�?*/
static float normalize_angle(float angle)   /* 角度归一化函�?*/
{
    while (angle > 180.0f) angle -= 360.0f; /* 大于180则减360 */
    while (angle < -180.0f) angle += 360.0f;/* 小于-180则加360 */
    return angle;                           /* 返回归一化角�?*/
}

/* ======================== RA yaw进度计算 ======================== */

/* ra_yaw_progress - 计算RA HARD阶段的yaw转动进度（正值，单位：度�? * 右转时直接用yaw_angle（正=右转），左转时取�? * 只返回正值（负值表示还没开始转或方向错误，视为0�? * 返回: yaw转动进度（正角度�?*/
static float ra_yaw_progress(void)          /* 计算RA偏航进度 */
{
    float delta = normalize_angle(yaw_angle); /* 获取归一化后的yaw�?*/

    /* 右转方向：yaw_angle为负值，取反得到正进�?*/
    if (s_ra_dir == 1u)                     /* 右转方向 */
        delta = -delta;                     /* 取反得到正进�?*/

    return (delta > 0.0f) ? delta : 0.0f;  /* 返回正值，负值视�? */
}

static float ra_yaw_progress_rate(void)
{
    float rate = (float)yaw_rate;

    if (s_ra_dir == 1u)
        rate = -rate;

    return (rate > 0.0f) ? rate : 0.0f;
}

/* ======================== RA调试信息更新 ======================== */

/* ra_debug_update - 将RA内部状态复制到全局调试变量，供TFT显示
 * 在RA状态机每个返回点调�? * 无参数，无返回�?*/
static void ra_debug_update(void)           /* 更新RA调试变量 */
{
    float yaw_progress = 0.0f;              /* yaw进度，初始为0 */

    if (s_ra_state == RA_ST_ACTIVE && s_ra_dir != 0u) /* RA活跃且有方向 */
        yaw_progress = ra_yaw_progress();   /* 计算yaw进度 */

    ra_dbg_state = (uint8)s_ra_state;       /* 复制RA状�?*/
    ra_dbg_phase = (uint8)s_ra_phase;       /* 复制RA阶段 */
    ra_dbg_dir = s_ra_dir;                  /* 复制RA方向 */
    ra_dbg_ip_row = s_ra_ip_row;            /* 复制拐点行号 */
    ra_dbg_timer = s_ra_timer;              /* 复制全局计时�?*/
    ra_dbg_hard_cnt = s_ra_hard_cnt;        /* 复制HARD计数 */
    /* RECOVER阶段显示recover计数，否则显示exit计数 */
    ra_dbg_exit_good_cnt = (s_ra_phase == RA_PH_RECOVER) ? /* 判断当前阶段 */
                           s_ra_recover_good_cnt : /* RECOVER阶段显示恢复计数 */
                           s_ra_exit_good_cnt;     /* 其他阶段显示退出计�?*/
    /* yaw进度x10，避免浮点显�?*/
    ra_dbg_yaw10 = (int16)(yaw_progress * 10.0f); /* �?0转int16供TFT显示 */
}

/* ======================== 电机输出限幅 ======================== */

/* clamp_duty - 将浮点duty值限幅到[-MAX_DUTY, MAX_DUTY]并转为int16
 * @val: 浮点duty�? * 返回: 限幅后的int16 duty�? * NaN检查：若val!=val为假（NaN），返回0保护电机 */
static int16 clamp_duty(float val)          /* 电机duty限幅函数 */
{
    if (val != val) return 0;               /* NaN检查：NaN不等于自身，返回0保护电机 */
    if (val > MAX_DUTY) val = MAX_DUTY;     /* 超过正最大值，限幅 */
    else if (val < -MAX_DUTY) val = -MAX_DUTY; /* 超过负最大值，限幅 */
    return (int16)val;                      /* 浮点转int16返回 */
}

static void pid_set_duty(int16 left, int16 right)
{
    duty_dbg_left = left;
    duty_dbg_right = right;
    small_driver_set_duty(left, right);
}

/* ======================== RA状态机复位 ======================== */

/* ra_reset - 复位RA状态机所有变量到初始状�? * 无参数，无返回�? * 调用时机：RA结束、电机使能关闭、初始化�?*/
static void ra_reset(void)                  /* RA状态机全复�?*/
{
    s_ra_state = RA_ST_NONE;                /* 状态重置为空闲 */
    s_ra_phase = RA_PH_WAIT;                /* 阶段重置为等�?*/
    s_ra_dir = 0u;                          /* 方向重置为直�?*/
    s_ra_orig_flag = 0u;                    /* 清除原始flag */
    s_ra_speed_ref_latched = 0;
    s_ra_ip_row = 0u;                       /* 清除拐点行号 */
    s_ra_straight = 0u;                     /* 清除直行标志 */
    s_ra_post_edge_side = EDGE_BOTH;        /* 单边方向重置为双�?*/
    s_ra_post_edge_ms = 0u;                 /* 单边时间清零 */
    s_ra_exit_good_cnt = 0u;                /* 退出计数清�?*/
    s_ra_recover_good_cnt = 0u;             /* 恢复计数清零 */
    s_ra_approach_cnt = 0u;                 /* 接近计数清零 */
    s_ra_timer = 0u;                        /* 全局计时器清�?*/
    s_ra_hard_cnt = 0u;                     /* HARD计数清零 */
    s_ra_recover_cnt = 0u;                  /* RECOVER计数清零 */
    s_ra_phase_cnt = 0u;                    /* 阶段计数清零 */
    s_ra_hard_speed_seed = 0.0f;            /* 速度种子清零 */
    s_ra_hard_steer_seed = 0.0f;            /* 转向种子清零 */
    s_ra_pre_turn_ff = 0.0f;
    s_route_pending_valid = 0u;             /* 待提交标志清�?*/
    ra_debug_update();                      /* 更新调试变量 */
}

/* ======================== 路线完成检�?======================== */

/* update_rules_done - 检查路线表中所有规则是否都已匹配完�? * 遍历user_rules[]，每条规则要求对应flag的计�?=规则的count
 * 全部满足则置 s_rules_done=1，触发延迟停�? * 无参数，无返回�?*/
static void update_rules_done(void)         /* 检查路线完成状�?*/
{
    uint8 all_done = 1u;                    /* 假设全部完成 */

    for (uint8 i = 0u; i < USER_RULE_COUNT; i++) /* 遍历路线表每条规�?*/
    {
        uint8 flag = user_rules[i].flag;    /* 获取规则的flag类型 */
        if (flag >= 7u || s_inter_count[flag] < user_rules[i].count) /* 计数不足 */
        {
            all_done = 0u;                  /* 标记未完�?*/
            break;                          /* 提前退出循�?*/
        }
    }

    if (all_done)                           /* 如果全部规则都满�?*/
        s_rules_done = 1u;                  /* 设置路线完成标志 */
}

/* route_debug_reset - 复位路线调试状�? * 无参数，无返回�?*/
static void route_debug_reset(void)         /* 复位路线调试变量 */
{
    route_dbg_step = 0u;                    /* 当前步数清零 */
    route_dbg_total = (uint8)USER_RULE_COUNT; /* 总步数设为规则总数 */
    route_dbg_flag = 0u;                    /* flag清零 */
    route_dbg_count = 0u;                   /* count清零 */
    route_dbg_action = ACT_STRAIGHT;        /* 动作清零为直�?*/
    s_route_pending_valid = 0u;             /* 待提交有效标志清�?*/
    s_route_pending_flag = 0u;              /* 待提交flag清零 */
    s_route_pending_count = 0u;             /* 待提交count清零 */
    s_route_pending_action = ACT_STRAIGHT;  /* 待提交动作清�?*/
}

/* route_debug_commit - 提交待确认的路线匹配结果到调试变�? * 延迟一帧提交，避免匹配结果影响当帧的RA启动
 * 无参数，无返回�?*/
static void route_debug_commit(void)        /* 提交路线调试信息 */
{
    if (!s_route_pending_valid)             /* 无待提交数据 */
        return;                             /* 直接返回 */

    if (route_dbg_step < route_dbg_total)   /* 未超过总步�?*/
        route_dbg_step++;                   /* 步数�? */
    route_dbg_flag = s_route_pending_flag;  /* 提交flag */
    route_dbg_count = s_route_pending_count;/* 提交count */
    route_dbg_action = s_route_pending_action; /* 提交动作 */

    s_route_pending_valid = 0u;             /* 清除待提交标�?*/
}

/* ======================== PD微分项重�?======================== */

/* line_pid_reset_derivative - 完全重置转向PD的微分状态（清零所有历史值）
 * 用于RA开�?结束、停机等场景
 * 无参数，无返回�?*/
void line_pid_reset_derivative(void)        /* 完全重置微分状�?*/
{
    s_steer_d_reset_flag = 1u;              /* 设置微分重置标志，下一周期跳过D�?*/
    s_filtered_err = 0.0f;                  /* 滤波误差清零 */
    s_prev_steer_output = 0.0f;             /* 上次转向输出清零 */
    s_steer_ff_filtered = 0.0f;             /* 前馈滤波值清�?*/
    s_cas_target_filtered = 0.0f;           /* 串级目标滤波值清�?*/
    s_cas_last_yaw_err = 0.0f;             /* 串级内环误差清零 */
}

/* line_pid_reset_derivative_keep_output - 部分重置微分状态（保留输出值的50%�? * 用于RECOVER阶段退出，避免转向突变
 * 无参数，无返回�?*/
static void line_pid_reset_derivative_keep_output(void) /* 部分重置微分 */
{
    s_steer_d_reset_flag = 1u;              /* 设置微分重置标志 */
    s_steer_ff_filtered *= 0.5f;            /* 前馈值保�?0% */
    s_cas_target_filtered = 0.0f;           /* 串级目标清零 */
    s_cas_last_yaw_err = 0.0f;             /* 串级内环误差清零 */
}

/* ======================== 速度规划器复�?======================== */

/* reset_speed_planner - 复位速度规划相关的所有状�? * 无参数，无返回�?*/
static void reset_speed_planner(void)       /* 复位速度规划�?*/
{
    s_base_speed_ramped = 0;                /* 斜坡速度清零 */
    s_straight_cnt = 0u;                    /* 直道计数清零 */
    s_straight_hold = 0u;
    s_straight_active = 0u;                 /* 直道激活标志清�?*/
    s_prev_quality_err = 0;                 /* 上帧误差清零 */
    s_prev_quality_err_valid = 0u;          /* 上帧误差有效标志清零 */
    speed_dbg_vq_pct = 100u;               /* 视觉质量百分比重置为100 */
    speed_dbg_pre_lock = 0u;               /* 预减速锁标志清零 */
    speed_dbg_raw = 0;                      /* 原始速度清零 */
    speed_dbg_plan = 0;                     /* 规划速度清零 */
    speed_dbg_reason = 0u;                  /* 速度原因清零 */
}

/* reset_speed_ff_state - 复位速度前馈状�? * 无参数，无返回�?*/
static void reset_speed_ff_state(void)      /* 复位速度前馈 */
{
    s_prev_target_speed = 0.0f;             /* 上一目标速度清零 */
    s_speed_ff_ready = 0u;                  /* 前馈就绪标志清零 */
}

/* ======================== 路线匹配辅助 ======================== */

/* route_has_next_match - 检查指定flag的下一个count是否能在路线表中找到匹配
 * @flag: 路口类型flag�?~5�? * 返回: 1=有下一个匹配，0=�?*/
static uint8 route_has_next_match(uint8 flag) /* 检查路线表是否有下一条匹�?*/
{
    uint8 next_count;                       /* 下一个期望的count�?*/

    if (flag >= 7u)                         /* flag越界检�?*/
        return 0u;                          /* 无效flag，无匹配 */

    next_count = s_inter_count[flag] + 1u;  /* 当前计数+1 = 下一个期望�?*/

    for (uint8 i = 0u; i < USER_RULE_COUNT; i++) /* 遍历路线�?*/
    {
        if (user_rules[i].flag == flag &&   /* flag类型匹配 */
            user_rules[i].count == next_count) /* count值匹�?*/
        {
            return 1u;                      /* 找到匹配，返�? */
        }
    }

    return 0u;                              /* 遍历完未找到，返�? */
}

uint8 route_next_flag_is(uint8 flag)
{
    return route_has_next_match(flag);
}

/* ra_fallback_direct_enabled - 检查RA保底直接转弯是否启用
 * @flag: 路口类型flag
 * 返回: 1=允许保底转弯�?=不允�? * 当RA_FALLBACK_DIRECT_ENABLE=1时，直角flag(1/2)允许保底转弯
 * 即使路线表中没有匹配到对应规�?*/
static uint8 ra_fallback_direct_enabled(uint8 flag) /* 检查RA保底转弯 */
{
#if RA_FALLBACK_DIRECT_ENABLE               /* 编译时开关：启用保底 */
    return (flag == 1u || flag == 2u) ? 1u : 0u; /* 直角flag(1/2)允许保底 */
#else                                       /* 保底关闭 */
    (void)flag;                             /* 消除未使用参数警�?*/
    return 0u;                              /* 不允许保�?*/
#endif
}

static void ra_clear_pre_flags(void)
{
    g_ra_pre_flag = 0u;
    g_ra_pre_dir = 0u;
    g_ra_pre_slow_flag = 0u;
    s_pre_lock = 0u;
    s_pre_timeout = 0u;
}

static void ra_clear_all_flags(void)
{
    g_ra_flag = 0u;
    ra_clear_pre_flags();
}

/* ======================== 丢线搜索 ======================== */

/* lost_search_reset - 复位丢线搜索状�? * 无参数，无返回�?*/
static void lost_search_reset(void)         /* 复位丢线搜索 */
{
    s_lost_search_active = 0u;              /* 搜索激活标志清�?*/
    s_lost_line_cnt = 0u;                   /* 丢线帧计数清�?*/
    s_lost_search_cnt = 0u;                 /* 搜索运行帧计数清�?*/
}

/* lost_search_pick_dir - 根据丢线前的最后误差选择搜索方向
 * @err: 丢线前的最后位置误�? * 返回: 1=向右搜索�?=向左搜索
 * err > 死区 �?向右搜索(1)
 * err < -死区 �?向左搜索(2)
 * 在死区内 �?保持上次搜索方向 */
static uint8 lost_search_pick_dir(int16 err) /* 选择丢线搜索方向 */
{
    if (err > LOST_SEARCH_ERR_DEADZONE)      /* 误差偏右，向右搜�?*/
        return 1u;                           /* 返回右转方向 */
    if (err < -LOST_SEARCH_ERR_DEADZONE)     /* 误差偏左，向左搜�?*/
        return 2u;                           /* 返回左转方向 */

    return (s_lost_search_dir == 2u) ? 2u : 1u; /* 在死区内，保持上次方�?*/
}

/* lost_search_step - 丢线搜索主逻辑，每PID周期调用
 * @pos_err: 当前位置误差
 * 返回: 1=正在搜索（本帧已直接输出电机），0=未搜索（继续正常PID�? *
 * 流程�? *   1. 若已找回线（line_lost=0且有效行足够）→ 退出搜�? *   2. 若RA活跃 �?不搜�? *   3. 若RA flag存在 �?不搜索（让RA处理�? *   4. 连续丢线达到阈�?�?启动搜索
 *   5. 搜索中定期切换方向，原地左右旋转寻线
 *   6. 搜索时速度清零，只输出差�?*/
static uint8 lost_search_step(int16 pos_err) /* 丢线搜索主逻辑 */
{
    /* 退出条件：找回线且有效行足�?*/
    if (g_tf.line_lost == 0u &&             /* 未丢�?*/
        g_tf.valid_row_count >= LOST_SEARCH_EXIT_VALID_ROWS) /* 有效行足�?*/
    {
        s_lost_last_err = pos_err;          /* 记录找回线时的误�?*/
        lost_search_reset();                /* 复位搜索状�?*/
        return 0u;                          /* 退出搜索，继续正常PID */
    }

    /* RA活跃时不搜索 */
    if (s_ra_state != RA_ST_NONE)           /* RA正在执行 */
    {
        lost_search_reset();                /* 复位搜索状�?*/
        return 0u;                          /* 不搜�?*/
    }

    /* 有RA flag时不搜索（让RA处理�?*/
    if (g_ra_flag != 0u || g_ra_pre_flag != 0u) /* 有RA标志 */
        return 0u;                          /* 不搜索，交给RA处理 */

    /* 未丢线时重置计数 */
    if (g_tf.line_lost == 0u)              /* 当前未丢�?*/
    {
        s_lost_line_cnt = 0u;              /* 丢线计数清零 */
        return 0u;                          /* 不搜�?*/
    }

    /* 累计丢线帧数 */
    if (s_lost_line_cnt < 255u)            /* 防溢�?*/
        s_lost_line_cnt++;                 /* 丢线帧计数加1 */

    /* 未达到启动阈�?*/
    if (s_lost_line_cnt < LOST_SEARCH_ENTER_FRAMES) /* 丢线帧数不足 */
        return 0u;                          /* 不启动搜�?*/

    /* 首次进入搜索：选择方向 */
    if (!s_lost_search_active)              /* 尚未激活搜�?*/
    {
        s_lost_search_active = 1u;          /* 激活搜�?*/
        s_lost_search_cnt = 0u;             /* 搜索计数清零 */
        s_lost_search_dir = lost_search_pick_dir(s_lost_last_err); /* 根据最后误差选方�?*/
    }

    /* 定期切换搜索方向（防止单方向转过头） */
    s_lost_search_cnt++;                    /* 搜索帧计数加1 */
    if (s_lost_search_cnt >= LOST_SEARCH_SWITCH_FRAMES) /* 达到切换阈�?*/
    {
        s_lost_search_cnt = 0u;             /* 计数清零 */
        s_lost_search_dir = (s_lost_search_dir == 1u) ? 2u : 1u; /* 左右切换 */
    }

    /* 搜索时清零速度和积分，重置PID状�?*/
    base_speed = 0;                         /* 基础速度清零 */
    s_speed_integral = 0.0f;                /* 速度积分清零 */
    reset_speed_planner();                  /* 复位速度规划�?*/
    reset_speed_ff_state();                 /* 复位速度前馈 */
    line_pid_reset_derivative();            /* 重置微分状�?*/

    /* 输出差速：forward recovery, no wheel reverse while running fast */
    if (s_lost_search_dir == 1u)            /* 向右搜索 */
    {
        pid_set_duty(clamp_duty(LOST_SEARCH_FORWARD_DUTY + LOST_SEARCH_DUTY),
                              clamp_duty(LOST_SEARCH_FORWARD_DUTY - LOST_SEARCH_DUTY));
    }
    else                                    /* 向左搜索 */
    {
        pid_set_duty(clamp_duty(LOST_SEARCH_FORWARD_DUTY - LOST_SEARCH_DUTY),
                              clamp_duty(LOST_SEARCH_FORWARD_DUTY + LOST_SEARCH_DUTY));
    }

    return 1u;                              /* 正在搜索，本帧已输出电机 */
}

/* ======================== 单边巡线 ======================== */

/* single_edge_reset - 复位单边巡线状态，恢复双边模式
 * 无参数，无返回�?*/
static void single_edge_reset(void)         /* 复位单边巡线 */
{
    uint8 was_single_edge = (s_edge_active != 0u || g_post_edge_side != EDGE_BOTH) ? 1u : 0u;
    s_edge_active = 0u;                     /* 激活标志清�?*/
    s_edge_side = EDGE_BOTH;                /* 方向恢复双边 */
    s_edge_cnt = 0u;                        /* 帧计数清�?*/
    s_edge_target = 0u;                     /* 目标帧数清零 */
    s_edge_until_next_turn = 0u;            /* 清除保持模式 */
    s_edge_release_after_turn = 0u;         /* 清除转弯后释放标�?*/
    s_single_edge_fast_hold = 0u;
    g_post_edge_side = EDGE_BOTH;           /* 全局方向恢复双边 */
    if (was_single_edge)
        turn_right_led_off();
}

/* start_single_edge - 启动单边巡线模式
 * @side: EDGE_LEFT �?EDGE_RIGHT
 * @duration_ms: 持续时间（毫秒），会被转换为PID周期帧数
 * 非法参数时复位为双边模式 */
void start_single_edge(uint8 side, uint16 duration_ms) /* 启动单边巡线 */
{
    if ((side != EDGE_LEFT && side != EDGE_RIGHT) || duration_ms == 0u) /* 参数合法性检�?*/
    {
        single_edge_reset();                /* 参数非法，复位为双边 */
        return;                             /* 返回 */
    }

    s_edge_active = 1u;                     /* 激活单边巡�?*/
    s_edge_side = side;                     /* 设置巡线方向 */
    s_edge_cnt = 0u;                        /* 帧计数清�?*/

    if (duration_ms == SINGLE_EDGE_UNTIL_NEXT_TURN)
    {
        s_edge_until_next_turn = 1u;        /* 保持到下一次真正转�?*/
        s_edge_release_after_turn = 0u;     /* 尚未遇到需要释放的转弯 */
        s_edge_target = 0u;                 /* 不使用时间退�?*/
    }
    else
    {
        s_edge_until_next_turn = 0u;        /* 普通定时单�?*/
        s_edge_release_after_turn = 0u;     /* 普通定时模式不使用转弯释放 */
        /* Convert ms to PID ticks, rounded up. */
        s_edge_target = PID_MS_TO_TICKS(duration_ms);
        if (s_edge_target == 0u)            /* 防止转换后为0 */
            s_edge_target = 1u;             /* 保证至少1�?*/
    }

    g_post_edge_side = side;                /* 设置全局单边方向 */
    turn_right_led_on();
}

/* single_edge_tick - 单边巡线每帧tick，由 line_pid_control() 调用
 * 确保 g_post_edge_side 与设定方向一致，计时到期后自动复�? * 无参数，无返回�?*/
static void single_edge_tick(void)          /* 单边巡线帧更�?*/
{
    if (!s_edge_active)                     /* 未激�?*/
        return;                             /* 直接返回 */

    /* 防止被外部意外修�?*/
    if (g_post_edge_side != s_edge_side)    /* 全局方向与设定不一�?*/
        g_post_edge_side = s_edge_side;     /* 恢复为设定方�?*/

    if (s_edge_until_next_turn)             /* 保持模式不按时间结束 */
        return;

    s_edge_cnt++;                           /* 帧计数加1 */
    if (s_edge_cnt >= s_edge_target)        /* 达到目标帧数 */
        single_edge_reset();                /* 复位单边巡线 */
}

/* ======================== 转弯屏蔽（Turn Shield�?======================== */

/* ra_finish_ex - RA结束的扩展版�? * @keep_flag: 结束后保留的RA flag�?=清除，非0=保留给下一个RA�? * @use_shield: 是否启动转弯屏蔽
 *
 * 逻辑�? *   1. 关闭LED
 *   2. 重置速度积分和PID微分
 *   3. 提交路线调试信息
 *   4. 检查路线完成状�? *   5. 根据post_edge配置启动单边巡线
 *   6. 启动转弯屏蔽（可选）
 *   7. 复位RA状态机 */
static void ra_finish_ex(uint8 keep_flag) /* RA结束扩展 */
{
    uint8 edge_side = EDGE_BOTH;            /* 单边巡线方向，初始为双边 */
    uint8 force_left_single_edge = 0u;
    uint8 from_recover = (s_ra_phase == RA_PH_RECOVER) ? 1u : 0u; /* 是否从RECOVER退�?*/

    turn_right_led_off();                   /* 关闭右转指示LED */
    g_ra_flag = keep_flag;                  /* 设置保留的flag�?=清除�?*/
    if (from_recover)
        s_speed_integral *= 0.50f;
    else
        s_speed_integral = 0.0f;
    /* RECOVER退出时保留部分输出，避免突�?*/
    if (from_recover)                       /* 从RECOVER阶段退�?*/
        line_pid_reset_derivative_keep_output(); /* 部分重置，保�?0%输出 */
    else                                    /* 从其他阶段退�?*/
        line_pid_reset_derivative();        /* 完全重置微分状�?*/
    route_debug_commit();                   /* 提交路线调试信息 */
    update_rules_done();                    /* 检查路线完成状�?*/

    if (s_ra_orig_flag == 1u && s_ra_straight == 0u)
    {
        if (s_completed_right_ra_count < 255u)
            s_completed_right_ra_count++;
        if (s_completed_right_ra_count >= 2u && s_edge_release_after_turn == 0u)
            force_left_single_edge = 1u;
    }

    if (s_edge_release_after_turn)          /* 已经完成下一次真正转�?*/
        single_edge_reset();                /* 单边任务结束，恢复双�?*/

    /* 确定单边巡线方向 */
    if (force_left_single_edge)
    {
        edge_side = EDGE_LEFT;
        s_ra_post_edge_ms = SINGLE_EDGE_UNTIL_NEXT_TURN;
    }
    else if (s_ra_post_edge_side == EDGE_LEFT || s_ra_post_edge_side == EDGE_RIGHT) /* 规则指定了具体方�?*/
    {
        /* 规则指定了具体方�?*/
        edge_side = s_ra_post_edge_side;    /* 使用规则指定的方�?*/
    }
    else if (s_ra_post_edge_side == EDGE_AUTO) /* 自动模式 */
    {
        /* 自动模式：右转后用左边线，左转后用右边线 */
        if (s_ra_dir == 1u)                 /* 右转 */
            edge_side = EDGE_LEFT;          /* 用左边线 */
        else if (s_ra_dir == 2u)            /* 左转 */
            edge_side = EDGE_RIGHT;         /* 用右边线 */
    }

    /* 启动单边巡线 */
    if (edge_side != EDGE_BOTH && s_ra_post_edge_ms > 0u) /* 需要单边且时间>0 */
        start_single_edge(edge_side, s_ra_post_edge_ms); /* 启动单边巡线 */

    g_ra_pre_flag = 0u;
    g_ra_pre_dir = 0u;
    g_ra_pre_slow_flag = 0u;
    s_pre_lock = 0u;
    s_pre_timeout = 0u;
    if (keep_flag == 0u && (from_recover || s_ra_straight != 0u))
        track_intersection_suppress_after_turn();
    ra_reset();                             /* 复位RA状态机 */
}

/* ra_finish - RA正常结束：清除flag + 启动转弯屏蔽
 * 无参数，无返回�?*/
static void ra_finish(void)                 /* RA正常结束 */
{
    ra_finish_ex(0u);                   /* 清除flag，启动转弯屏�?*/
}

/* ra_enter_recover - 进入RECOVER阶段
 * 特点�? *   - 使用HARD阶段的速度/转向种子值平滑过�? *   - 重新初始化PD控制器（以当前误差为初始值）
 *   - 速度前馈就绪（避免启动突变）
 * 无参数，无返回�?*/
static void ra_enter_recover(void)          /* 进入RECOVER阶段 */
{
    turn_right_led_off();                   /* 关闭右转指示LED */
    g_ra_flag = 0u;                         /* 清除RA flag */
    g_ra_pre_dir = 0u;
    g_ra_pre_slow_flag = 0u;
    s_ra_phase = RA_PH_RECOVER;             /* 切换到RECOVER阶段 */
    s_ra_phase_cnt = 0u;                    /* 阶段计数清零 */
    s_ra_recover_cnt = 0u;                  /* RECOVER帧计数清�?*/
    s_ra_recover_good_cnt = 0u;             /* RECOVER确认计数清零 */
    s_speed_integral *= 0.60f;
    s_steer_d_reset_flag = 1u;              /* 设置微分重置标志 */
    /* 以当前视觉误差初始化滤波器，避免恢复时跳�?*/
    s_filtered_err = (float)g_tf.error;     /* 用当前误差初始化滤波�?*/
    s_steer_last_pos_err = s_filtered_err;  /* 上次滤波误差 = 当前滤波�?*/
    s_steer_last_raw_err = (float)g_tf.error; /* 上次原始误差 = 当前误差 */
    s_steer_ff_filtered = 0.0f;             /* 前馈滤波值清�?*/
    s_cas_target_filtered = 0.0f;           /* 串级目标滤波值清�?*/
    s_cas_last_yaw_err = 0.0f;             /* 串级内环误差清零 */
    /* 从HARD种子值按比例过渡 */
    s_prev_steer_output = s_ra_hard_steer_seed * /* HARD阶段转向种子 */
                          ((float)RA_RECOVER_SEED_STEER_PCT * 0.01f); /* RECOVER比例过渡 */
    s_base_speed_ramped = (int16)((float)motor_speed * 8.0f *
                                  ((float)RA_RECOVER_SPEED_PCT * 0.01f));
    s_prev_target_speed = (float)s_base_speed_ramped; /* 上一目标速度 = 种子�?*/
    s_speed_ff_ready = 1u;                  /* 前馈就绪 */
    ra_debug_update();                      /* 更新调试变量 */
}

/* ra_start - 启动RA状态机
 * @dir: 方向�?=直行 1=�?2=左）
 * @orig_flag: 原始flag�? * @straight: 是否直行通过
 * @post_edge_side: 结束后单边方�? * @post_edge_ms: 结束后单边时�? * 无返回�?*/
static void ra_start(uint8 dir, uint8 orig_flag, uint8 straight,
                     uint8 post_edge_side, uint16 post_edge_ms) /* 启动RA */
{
    if (!straight && s_edge_until_next_turn)/* 下一个真正转弯已开�?*/
        s_edge_release_after_turn = 1u;     /* 等这个RA结束后恢复双�?*/

    s_ra_dir = dir;                         /* 设置转弯方向 */
    s_ra_orig_flag = orig_flag;             /* 保存原始flag */
    s_ra_speed_ref_latched = ra_speed_ref();
    s_ra_state = RA_ST_ACTIVE;              /* 状态切换为活跃 */
    /* 直行通过不需要等转点；真正转弯先 WAIT，避免远场路口刚识别就硬转。 */
    s_ra_phase = straight ? RA_PH_SLOW : RA_PH_WAIT;
    s_ra_ip_row = g_ip_max_row;             /* 记录当前拐点行号 */
    s_ra_straight = straight;               /* 设置直行标志 */
    s_ra_post_edge_side = post_edge_side;   /* 设置结束后单边方�?*/
    s_ra_post_edge_ms = post_edge_ms;       /* 设置结束后单边时�?*/
    s_ra_exit_good_cnt = 0u;                /* 退出确认计数清�?*/
    s_ra_recover_good_cnt = 0u;             /* 恢复确认计数清零 */
    s_ra_approach_cnt = 0u;                 /* 接近计数清零 */
    s_ra_timer = 0u;                        /* 全局计时器清�?*/
    s_ra_hard_cnt = 0u;                     /* HARD计数清零 */
    s_ra_recover_cnt = 0u;                  /* RECOVER计数清零 */
    s_ra_phase_cnt = 0u;                    /* 阶段计数清零 */
    s_ra_hard_speed_seed = 0.0f;            /* 速度种子清零 */
    s_ra_hard_steer_seed = 0.0f;            /* 转向种子清零 */
    s_speed_integral *= 0.70f;
    reset_speed_planner();                  /* 复位速度规划�?*/
    lost_search_reset();                    /* 复位丢线搜索 */

    /* 非直行模式点亮LED指示 */
    if (!straight)                          /* 非直行（需要转弯） */
        turn_right_led_on();                /* 点亮右转指示LED */

    ra_debug_update();                      /* 更新调试变量 */
}

/* ra_enter_hard - 进入HARD阶段（硬转弯�? * 重置yaw角，准备用固定duty驱动转弯
 * 无参数，无返回�?*/
static void ra_enter_hard(void)             /* 进入HARD硬转弯阶�?*/
{
    if (s_ra_phase == RA_PH_HARD)           /* 已经在HARD阶段 */
        return;                             /* 避免重复进入 */

    s_ra_phase = RA_PH_HARD;                /* 切换到HARD阶段 */
    s_ra_phase_cnt = 0u;                    /* 阶段计数清零 */
    s_ra_hard_cnt = 0u;                     /* HARD帧计数清�?*/
    s_ra_exit_good_cnt = 0u;                /* 退出确认计数清�?*/
    s_ra_recover_good_cnt = 0u;             /* 恢复确认计数清零 */
    s_ra_recover_cnt = 0u;                  /* RECOVER计数清零 */
    s_speed_integral *= 0.50f;
    s_ra_pre_turn_ff = 0.0f;
    line_pid_reset_derivative();            /* 重置微分状�?*/

    /* 重置yaw角为0，用于HARD阶段的yaw进度判断 */
    if (imu_ready && !imu_error)            /* IMU正常工作 */
        imu_reset_yaw();                    /* 重置yaw角为0 */

    ra_debug_update();                      /* 更新调试变量 */
}

/* ======================== PID初始�?======================== */

/* line_pid_init - 初始化所有PID相关状态（系统启动或重新使能时调用�? * 复位：转向PD、速度PI、RA状态机、单边巡线、丢线搜索、路线计数等
 * 无参数，无返回�?*/
void line_pid_init(void)                    /* PID控制器初始化 */
{
    s_steer_last_pos_err = 0.0f;            /* 上次滤波误差清零 */
    s_steer_last_raw_err = 0.0f;            /* 上次原始误差清零 */
    s_filtered_err = 0.0f;                  /* 滤波误差清零 */
    s_prev_steer_output = 0.0f;             /* 上次转向输出清零 */
    s_steer_ff_filtered = 0.0f;             /* 前馈滤波值清�?*/
    s_cas_target_filtered = 0.0f;           /* 串级目标滤波值清�?*/
    s_cas_last_yaw_err = 0.0f;             /* 串级内环误差清零 */
    s_steer_d_reset_flag = 1u;              /* 设置微分重置标志 */
    s_speed_integral = 0.0f;               /* 速度积分清零 */
    s_motor_run_counter = 0u;               /* 电机运行计数清零 */
    s_vacuum_on = 0u;                       /* 负压状态清�?*/
    s_vacuum_duty = 0u;
    s_vacuum_prearm_ticks = 0u;
    s_vacuum_prearm_timeout = 0u;
    vacuum_enable = 0u;                     /* 负压显示状态清�?*/
    s_rules_done = 0u;                      /* 路线完成标志清零 */
    s_rules_done_timer = 0u;                /* 路线完成计时器清�?*/
    g_ra_pre_slow_flag = 0u;                /* 清除预减速专用标�?*/
    s_pre_lock = 0u;
    s_pre_timeout = 0u;
    single_edge_reset();                    /* 复位单边巡线 */
    s_completed_right_ra_count = 0u;
    lost_search_reset();                    /* 复位丢线搜索 */
    s_lost_last_err = 0;                    /* 丢线最后误差清�?*/
    s_lost_search_dir = 1u;                /* 搜索方向默认�?*/
    reset_speed_planner();                  /* 复位速度规划�?*/
    reset_speed_ff_state();                 /* 复位速度前馈 */
    route_debug_reset();                    /* 复位路线调试 */

    ra_reset();                             /* 复位RA状态机 */

    /* 清零所有路口类型计�?*/
    for (uint8 i = 0u; i < 7u; i++)         /* 遍历所有flag类型�?~6�?*/
        s_inter_count[i] = 0u;             /* 计数清零 */
}

/* ======================== 串级PID转向计算 ======================== */

/* cascade_steer_calc - 串级PID转向计算：图像位置外�?+ IMU角速度内环
 * @pos_err: 位置误差
 * @slew_max: 输出变化率限�? * 返回: 转向输出�? *
 * 外环�? *   - 对位置误差做软死区处理（死区内p_scale=0，软区内二次平滑�? *   - 组合：位置项 + 趋势前馈 + 前瞻前馈 �?目标角速度
 *   - 路口附近抑制趋势/前瞻前馈
 *   - 目标角速度低通滤�? * 内环�? *   - 角速度误差 = 目标角速度 - 实际yaw_rate
 *   - 内环PD控制（kp/kd由菜单调节）
 *   - 输出限幅 + 变化率限�?*/
static float cascade_steer_calc(int16 pos_err, float kp_scale, float kd_scale, float slew_max) /* 串级PID转向 */
{
    float filter_alpha = s_straight_active ?
                         ERROR_FILTER_STRAIGHT_ALPHA :
                         ERROR_FILTER_ALPHA;
    float raw_err = (float)pos_err;

    s_filtered_err = s_filtered_err * filter_alpha +
                     raw_err * (1.0f - filter_alpha);

    float err     = s_filtered_err;         /* 位置误差转浮�?*/
    float err_abs = abs_f(err);             /* 位置误差绝对�?*/

    /* Outer loop: soft deadzone on position error. 外环：软死区处理 */
    float p_scale = 1.0f;                   /* 比例缩放因子，初始为1 */
    if (err_abs <= (float)STEER_DEADZONE)   /* 误差在死区内 */
    {
        p_scale = 0.0f;                     /* 比例项归�?*/
    }
    else if (err_abs < (float)STEER_SOFT_END) /* 误差在软死区范围�?*/
    {
        float t = (err_abs - (float)STEER_DEADZONE) / /* 当前距离死区起点 */
                  ((float)STEER_SOFT_END - (float)STEER_DEADZONE); /* 除以软区总宽�?*/
        p_scale = t * t;                    /* 二次曲线平滑过渡 */
    }

    /* 计算目标角速度 = 位置�?+ 趋势前馈 + 前瞻前馈 */
    float yaw_target = CAS_POS_KP  * kp_scale * err * p_scale          /* 位置项：误差×比例 */
                     + CAS_TREND_KD * (float)g_tf.error_trend /* 趋势前馈 */
                     + CAS_LA_K    * (float)g_tf.lookahead_error; /* 前瞻前馈 */

    /* Suppress trend/LA feedforward near intersections. 路口附近抑制前馈 */
    if (g_ra_pre_flag != 0u || g_ra_flag != 0u) /* 有RA标志 */
        yaw_target = CAS_POS_KP * kp_scale * err * p_scale; /* 仅保留位置项 */

    /* Low-pass filter + clamp. 低通滤�?+ 限幅 */
    yaw_target = clamp_f(yaw_target, -CAS_TARGET_MAX, CAS_TARGET_MAX); /* 限幅到�?20 */
    s_cas_target_filtered = s_cas_target_filtered * CAS_TARGET_FILTER /* 上一帧�?.55 */
                          + yaw_target * (1.0f - CAS_TARGET_FILTER);  /* 新值�?.45 */

    /* Inner loop: IMU yaw_rate closed loop. 内环：IMU角速度闭环 */
    float yaw_err = s_cas_target_filtered - (float)yaw_rate; /* 角速度误差 */
    if (abs_f(yaw_err) < CAS_DEADZONE_DPS)  /* 误差在死区内 */
        yaw_err = 0.0f;                     /* 归零，防�?*/

    float kp_val = (float)yaw_kp * CAS_YAW_KP_SCALE * kd_scale; /* 内环比例增益 */
    float kd_val = (float)yaw_kd * CAS_YAW_KD_SCALE * kd_scale; /* 内环微分增益 */

    float p_out  = kp_val * yaw_err;        /* 内环P�?*/
    float d_out = 0.0f;
    if (s_steer_d_reset_flag == 0u)
        d_out = kd_val * (yaw_err - s_cas_last_yaw_err);
    else
        s_steer_d_reset_flag = 0u;
    s_cas_last_yaw_err = yaw_err;           /* 保存当前误差供下帧D项使�?*/

    float steer = p_out + d_out;            /* 内环总输�?*/
    steer = clamp_f(steer, -STEER_MAX, STEER_MAX); /* 限幅到�?000 */

    /* Slew rate limit (consistent with original PD). 变化率限�?*/
    float delta = steer - s_prev_steer_output; /* 输出变化�?*/
    if      (delta >  slew_max) steer = s_prev_steer_output + slew_max; /* 正向限幅 */
    else if (delta < -slew_max) steer = s_prev_steer_output - slew_max; /* 负向限幅 */

    s_prev_steer_output = steer;            /* 保存当前输出 */
    s_steer_last_pos_err = err;
    s_steer_last_raw_err = raw_err;
    return steer;                           /* 返回转向�?*/
}

/* ======================== 转向PD控制 ======================== */

/* steer_pd_calc - 转向PD控制�? * @pos_err: 位置误差（负=偏左，正=偏右�? * @kp_scale: 比例增益缩放（由调度器计算）
 * @kd_scale: 微分增益缩放（由调度器计算）
 * @feedforward: 前馈信号（基于前瞻误差）
 * @slew_max: 输出变化率限�? * 返回: 转向输出值（叠加到速度上形成差速）
 *
 * 逻辑�? *   1. 一阶低通滤波位置误�? *   2. 死区判断：误差很小且无前馈时输出0
 *   3. 软死区：二次曲线平滑过渡
 *   4. P�?= KP * kp_scale * 滤波误差 * p_scale
 *   5. D�?= KD * kd_scale * (当前原始误差 - 上次原始误差)
 *   6. 输出 = P + D + 前馈
 *   7. 限幅 + 变化率限�?*/
static float steer_pd_calc(int16 pos_err,           /* 位置误差 */
                           float kp_scale,           /* 比例增益缩放 */
                           float kd_scale,           /* 微分增益缩放 */
                           float feedforward,        /* 前馈信号 */
                           float slew_max)           /* 变化率限�?*/
{
    float filter_alpha = s_straight_active ?
                         ERROR_FILTER_STRAIGHT_ALPHA :
                         ERROR_FILTER_ALPHA;

    s_filtered_err = s_filtered_err * filter_alpha +
                     (float)pos_err * (1.0f - filter_alpha);

    float err = s_filtered_err;             /* 滤波后的误差 */
    float err_abs = abs_f(err);             /* 滤波误差绝对�?*/
    float raw_err = (float)pos_err;         /* 原始误差（未滤波�?*/

    /* 死区：误差极小且前馈也很小时，直接返�?（防抖） */
    if (err_abs <= (float)STEER_DEADZONE && abs_f(feedforward) <= 1.0f) /* 死区+前馈极小 */
    {
        s_steer_last_pos_err = err;         /* 保存滤波误差 */
        s_steer_last_raw_err = raw_err;     /* 保存原始误差 */
        s_prev_steer_output *= 0.5f;        /* 衰减上次输出（防止积累） */
        return 0.0f;                        /* 输出0，防�?*/
    }

    /* 软死区：死区到软区间用二次曲线平�?*/
    float p_scale = 1.0f;                   /* 比例缩放因子，初始为1 */
    if (err_abs <= (float)STEER_DEADZONE)   /* 误差在死区内 */
    {
        p_scale = 0.0f;                     /* 比例项归�?*/
    }
    else if (err_abs < (float)STEER_SOFT_END) /* 误差在软死区范围�?*/
    {
        float t = (err_abs - (float)STEER_DEADZONE) / /* 当前距离死区起点 */
                  ((float)STEER_SOFT_END - (float)STEER_DEADZONE); /* 除以软区总宽�?*/
        p_scale = t * t;                    /* 二次曲线平滑 */
    }

    /* P项计�?*/
    float p_out = STEER_KP * kp_scale * err * p_scale; /* 比例输出 */
    float d_out = 0.0f;                     /* 微分输出，初始为0 */

    /* D项：若未被重置则正常计算，否则跳过一�?*/
    if (s_steer_d_reset_flag == 0u)         /* 微分未被重置 */
        d_out = STEER_KD * kd_scale * (err - s_steer_last_pos_err) * PID_D_SCALE; /* 微分输出 */
    else                                    /* 微分被重�?*/
        s_steer_d_reset_flag = 0u;          /* 清除重置标志，下帧恢复D�?*/

    s_steer_last_pos_err = err;             /* 保存当前滤波误差 */
    s_steer_last_raw_err = raw_err;         /* 保存当前原始误差 */

    /* 总输�?= P + D + 前馈 */
    float output = p_out + d_out + feedforward; /* 三项相加 */

    /* 输出限幅 */
    if (output > STEER_MAX) output = STEER_MAX;         /* 正向限幅 */
    else if (output < -STEER_MAX) output = -STEER_MAX;  /* 负向限幅 */

    /* 变化率限制（防止转向突变�?*/
    float delta = output - s_prev_steer_output; /* 输出变化�?*/
    if (delta > slew_max)                   /* 正向变化过大 */
        output = s_prev_steer_output + slew_max; /* 限制正向变化�?*/
    else if (delta < -slew_max)             /* 负向变化过大 */
        output = s_prev_steer_output - slew_max; /* 限制负向变化�?*/

    s_prev_steer_output = output;           /* 保存当前输出 */
    return output;                          /* 返回转向�?*/
}

/* ======================== 速度PI控制 ======================== */

/* speed_pi_calc - 速度PI控制�? * @target: 目标速度
 * @actual: 实际速度（左右轮平均�? * @integral: 积分指针
 * @pos_err_abs: 位置误差绝对值（用于积分分离�? * 返回: 速度PI输出
 *
 * 积分分离：位置误差大时不积分（防止弯道超调）
 * 积分限幅：防止积分饱�?*/
static float speed_pi_calc(float target, float actual, float *integral, int16 pos_err_abs) /* 速度PI */
{
    float speed_err = target - actual;      /* 速度误差 = 目标 - 实际 */

    /* 积分分离：位置误�?< 阈值时才积�?*/
    if (pos_err_abs < SPEED_I_SEPARATION)   /* 位置误差小于分离阈�?20) */
    {
        *integral += speed_err * PID_DT_SCALE; /* 累加积分�?*/

        /* 积分限幅 */
        if (*integral > SPEED_I_MAX) *integral = SPEED_I_MAX;       /* 正向积分限幅 */
        else if (*integral < -SPEED_I_MAX) *integral = -SPEED_I_MAX; /* 负向积分限幅 */
    }
    else
    {
        *integral *= 0.92f;
        if (*integral > -1.0f && *integral < 1.0f)
            *integral = 0.0f;
    }

    return SPEED_KP * speed_err + SPEED_KI * (*integral); /* P�?+ I�?*/
}

/* ======================== 误差自适应速度 ======================== */

/* calc_adapted_speed - 根据位置误差自适应调整速度
 * @base: 基础速度
 * @pos_err_abs: 位置误差绝对�? * 返回: 自适应后的速度
 *
 * 逻辑：误差小于t1时用ratio_1%，大于t2时用ratio_2%，中间线性插�? * 通常ratio_1 > ratio_2（直道快，弯道慢�?*/
static int16 calc_adapted_speed(int16 base, int16 pos_err_abs) /* 误差自适应速度 */
{
    int16 t1 = sp_err_t1;                  /* 误差阈值下限（菜单可调�?*/
    int16 t2 = sp_err_t2;                  /* 误差阈值上限（菜单可调�?*/
    int16 r1 = sp_ratio_1;                 /* 低误差速度百分比（菜单可调�?*/
    int16 r2 = sp_ratio_2;                 /* 高误差速度百分比（菜单可调�?*/

    if (t2 <= t1)                           /* 阈值无效（上限≤下限） */
        t2 = t1 + 1;                       /* 修正为下�?1 */

    if (pos_err_abs <= t1)                  /* 误差小于下限 */
        return (int16)((int32)base * r1 / 100); /* 用高百分比（直道快） */

    if (pos_err_abs >= t2)                  /* 误差大于上限 */
        return (int16)((int32)base * r2 / 100); /* 用低百分比（弯道慢） */

    /* 中间线性插�?*/
    int32 ratio = (int32)r1 + ((int32)(r2 - r1) * (pos_err_abs - t1)) / (t2 - t1); /* 线性插�?*/
    return (int16)((int32)base * ratio / 100); /* 用插值百分比 */
}

/* ======================== 直道判定 ======================== */

/* straight_speed_candidate - 判断当前是否为直道加速候�? * @pos_err_abs: 位置误差绝对�? * 条件：未丢线、有效行足够、误差小、前瞻小、趋势小、无RA
 * 返回 1=是直道候�?*/
static uint8 straight_speed_candidate(int16 pos_err_abs) /* 判断直道加速候�?*/
{
    if (g_tf.line_lost != 0u)               /* 丢线 */
        return 0u;                          /* 不是候�?*/
    if (g_tf.valid_row_count < SPEED_STRAIGHT_VALID_ROWS) /* 有效行不�?35) */
        return 0u;                          /* 不是候�?*/
    if (pos_err_abs > SPEED_STRAIGHT_ERR_MAX) /* 位置误差过大(>6) */
        return 0u;                          /* 不是候�?*/
    if (abs_i16(g_tf.lookahead_error) > SPEED_STRAIGHT_LOOKAHEAD_MAX) /* 前瞻误差过大(>8) */
        return 0u;                          /* 不是候�?*/
    if (abs_i16(g_tf.error_trend) > SPEED_STRAIGHT_TREND_MAX) /* 趋势误差过大(>8) */
        return 0u;                          /* 不是候�?*/
    if (g_ra_pre_flag != 0u || g_ra_flag != 0u) /* 有RA标志 */
        return 0u;                          /* 不是候�?*/

    return 1u;                              /* 满足所有条件，是直道候�?*/
}

/* sym_straight_speed_candidate - 判断当前是否为对称组件直道加速候�? * @pos_err_abs: 位置误差绝对�? * 对称组件（如三极管干扰区）被检测到时，说明赛道较直
 * 条件：有对称组件、未丢线、有效行足够、误差适中、无RA
 * 返回 1=是候�?*/

static uint8 straight_hold_safe(int16 pos_err_abs)
{
    if (g_tf.line_lost != 0u)
        return 0u;
    if (g_tf.valid_row_count < SPEED_STRAIGHT_HOLD_VALID_ROWS)
        return 0u;
    if (pos_err_abs > SPEED_STRAIGHT_HOLD_ERR_MAX)
        return 0u;
    if (abs_i16(g_tf.lookahead_error) > SPEED_STRAIGHT_HOLD_LOOKAHEAD_MAX)
        return 0u;
    if (abs_i16(g_tf.error_trend) > SPEED_STRAIGHT_HOLD_TREND_MAX)
        return 0u;
    if (g_ra_pre_flag != 0u || g_ra_flag != 0u)
        return 0u;

    return 1u;
}

static int16 straight_boost_speed(int16 target, uint8 reason)
{
    s_straight_cnt = SPEED_STRAIGHT_CONFIRM_FRAMES;
    s_straight_hold = SPEED_STRAIGHT_HOLD_FRAMES;
    s_straight_active = 1u;
    target = (int16)((int32)target * SPEED_STRAIGHT_BOOST_PCT / 100);
    return speed_ramp_apply_reason(target, reason);
}

static uint8 sym_straight_speed_candidate(int16 pos_err_abs) /* 对称组件直道候�?*/
{
    if (g_sym_component_flag == 0u)         /* 未检测到对称组件 */
        return 0u;                          /* 不是候�?*/
    if (g_tf.line_lost != 0u)               /* 丢线 */
        return 0u;                          /* 不是候�?*/
    if (g_tf.valid_row_count < SPEED_SYM_VALID_ROWS) /* 有效行不�?30) */
        return 0u;                          /* 不是候�?*/
    if (pos_err_abs > SPEED_SYM_ERR_MAX)   /* 位置误差过大(>12) */
        return 0u;                          /* 不是候�?*/
    if (g_ra_pre_flag != 0u || g_ra_flag != 0u) /* 有RA标志 */
        return 0u;                          /* 不是候�?*/

    return 1u;                              /* 满足所有条件，是候�?*/
}

static uint8 component_fast_speed_candidate(int16 pos_err_abs)
{
    if (g_sym_component_flag == 0u)
        return 0u;
    if (g_tf.line_lost != 0u)
        return 0u;
    if (g_tf.valid_row_count < SPEED_COMPONENT_VALID_ROWS)
        return 0u;
    if (pos_err_abs > SPEED_COMPONENT_ERR_MAX)
        return 0u;
    if (abs_i16(g_tf.lookahead_error) > SPEED_COMPONENT_LA_MAX)
        return 0u;
    if (abs_i16(g_tf.error_trend) > SPEED_COMPONENT_TREND_MAX)
        return 0u;
    if (g_ra_pre_flag != 0u || g_ra_flag != 0u)
        return 0u;

    return 1u;
}

/* single_edge_speed_candidate - 单边巡线稳定时允许快速通过
 * 单边用于压住右侧岔路干扰，稳定时应按直道逻辑提速；
 * 真正转弯RA或预减速阶段不加速�?*/
static uint8 single_edge_speed_candidate(int16 pos_err_abs)
{
    uint8 stable = 1u;

    if (s_edge_active == 0u)
    {
        s_single_edge_fast_hold = 0u;
        return 0u;
    }
    if (g_post_edge_side == EDGE_BOTH)
    {
        s_single_edge_fast_hold = 0u;
        return 0u;
    }
    if (s_ra_state == RA_ST_ACTIVE && s_ra_straight == 0u)
    {
        s_single_edge_fast_hold = 0u;
        return 0u;
    }
    if (s_ra_state == RA_ST_NONE && g_post_edge_side != EDGE_BOTH)
    {
        if (g_ra_flag >= 3u && g_ra_flag <= 5u)
            ra_clear_all_flags();
        else if (g_ra_flag != 0u && !route_next_flag_is((uint8)g_ra_flag))
            ra_clear_all_flags();

        if (g_ra_pre_flag != 0u &&
            (g_ra_pre_dir != 1u && g_ra_pre_dir != 2u))
            ra_clear_pre_flags();
        else if (g_ra_pre_flag != 0u &&
                 !route_next_flag_is((uint8)g_ra_pre_dir))
            ra_clear_pre_flags();
    }
    if (s_ra_state == RA_ST_NONE &&
        (g_ra_pre_flag != 0u || g_ra_flag != 0u))
    {
        s_single_edge_fast_hold = 0u;
        return 0u;
    }
    if (g_tf.line_lost != 0u)
    {
        s_single_edge_fast_hold = 0u;
        return 0u;
    }
    if (g_tf.valid_row_count < SPEED_VISION_BAD_VALID_ROWS)
    {
        s_single_edge_fast_hold = 0u;
        return 0u;
    }

    if (g_tf.valid_row_count < SPEED_SINGLE_EDGE_VALID_ROWS)
        stable = 0u;
    if (pos_err_abs > SPEED_SINGLE_EDGE_ERR_MAX)
        stable = 0u;
    if (abs_i16(g_tf.lookahead_error) > SPEED_SINGLE_EDGE_LOOKAHEAD_MAX)
        stable = 0u;
    if (abs_i16(g_tf.error_trend) > SPEED_SINGLE_EDGE_TREND_MAX)
        stable = 0u;

    if (stable)
    {
        s_single_edge_fast_hold = SPEED_SINGLE_EDGE_HOLD_FRAMES;
        return 1u;
    }

    if (s_single_edge_fast_hold > 0u)
    {
        s_single_edge_fast_hold--;
        return 1u;
    }

    return 0u;
}

/* ======================== 速度斜坡 ======================== */

/* speed_ramp_apply - 基础速度斜坡处理（无调试信息版）
 * 防止目标速度突变，按步长渐变
 * @target: 目标速度
 * 返回: 斜坡处理后的速度 */
static int16 speed_ramp_apply(int16 target) /* 速度斜坡处理 */
{
    int16 up_step = (int16)((float)SPEED_RAMP_UP_STEP * PID_DT_SCALE + 0.5f);
    int16 down_step = (int16)((float)SPEED_RAMP_DOWN_STEP * PID_DT_SCALE + 0.5f);

    if (up_step < 1)
        up_step = 1;
    if (down_step < 1)
        down_step = 1;

    if (target < 0)                         /* 目标速度为负 */
        target = 0;                         /* 限制�? */

    /* 首次调用（负值表示未初始化），直接跳到目�?*/
    if (s_base_speed_ramped < 0)            /* 首次调用 */
    {
        s_base_speed_ramped = target;       /* 直接设为目标 */
        return s_base_speed_ramped;         /* 返回 */
    }

    /* 按步长渐�?*/
    if (target > s_base_speed_ramped + up_step) /* 需要加�?*/
        s_base_speed_ramped += up_step;         /* 按上升步长增�?*/
    else if (target < s_base_speed_ramped - down_step) /* 需要减�?*/
        s_base_speed_ramped -= down_step;       /* 按下降步长减�?*/
    else                                    /* 差值在步长范围�?*/
        s_base_speed_ramped = target;       /* 直接设为目标 */

    return s_base_speed_ramped;             /* 返回斜坡处理后的速度 */
}

/* speed_ramp_apply_reason - 带调试信息的速度斜坡处理
 * @target: 目标速度
 * @reason: 速度变化原因编号
 * 直道/单边加速时使用更大的上升步�? * 返回: 斜坡处理后的速度 */
static int16 speed_ramp_apply_reason(int16 target, uint8 reason) /* 带原因的斜坡 */
{
    int16 up_step = SPEED_RAMP_UP_STEP;     /* 上升步长 */
    int16 down_step = SPEED_RAMP_DOWN_STEP;

    speed_dbg_plan = target;                /* 记录规划速度 */
    speed_dbg_reason = reason;              /* 记录原因编号 */

    if (target < 0)                         /* 目标速度为负 */
        target = 0;                         /* 限制�? */

    /* 直道/单边加速原�?4/6/9)使用更快的上升步�?*/
    if (reason == 4u || reason == 6u || reason == 9u)
        up_step = SPEED_RAMP_STRAIGHT_UP_STEP; /* 使用更大的上升步�?*/
    if (reason == 5u || reason == 7u || reason == 8u)
        down_step = SPEED_RAMP_SOFT_DOWN_STEP;

    up_step = (int16)((float)up_step * PID_DT_SCALE + 0.5f);
    down_step = (int16)((float)down_step * PID_DT_SCALE + 0.5f);
    if (up_step < 1)
        up_step = 1;
    if (down_step < 1)
        down_step = 1;

    /* 首次调用直接跳到目标 */
    if (s_base_speed_ramped < 0)            /* 首次调用 */
    {
        s_base_speed_ramped = target;       /* 直接设为目标 */
        return s_base_speed_ramped;         /* 返回 */
    }

    /* 按步长渐�?*/
    if (target > s_base_speed_ramped + up_step) /* 需要加�?*/
        s_base_speed_ramped += up_step;          /* 按上升步长增�?*/
    else if (target < s_base_speed_ramped - down_step) /* 需要减�?*/
        s_base_speed_ramped -= down_step; /* 按下降步长减�?*/
    else                                    /* 差值在步长范围�?*/
        s_base_speed_ramped = target;       /* 直接设为目标 */

    return s_base_speed_ramped;             /* 返回斜坡处理后的速度 */
}

/* ======================== 速度百分比计�?======================== */

/* apply_speed_pct - 将目标速度乘以百分比缩�? * @target: 目标速度
 * @pct: 百分比（0~120�?00=不变�? * 返回: 缩放后的速度 */
static int16 apply_speed_pct(int16 target, int16 pct) /* 速度百分比缩�?*/
{
    if (pct < 0) pct = 0;                  /* 百分比下限为0 */
    if (pct > 120) pct = 120;              /* 百分比上限为120 */

    return (int16)((int32)target * pct / 100); /* 计算百分�?*/
}

/* calc_signal_speed_pct - 根据信号值计算降速百分比
 * @signal: 信号值（如前瞻误差、趋势误差）
 * @t1: 信号阈值下限（低于此不降速）
 * @t2: 信号阈值上限（高于此最大降速）
 * @min_pct: 最大降速百分比
 * 返回: 速度百分比（100=不降，min_pct=最大降�?*/
static int16 calc_signal_speed_pct(int16 signal, int16 t1, int16 t2, int16 min_pct) /* 信号降�?*/
{
    if (t2 <= t1)                           /* 阈值无�?*/
        t2 = t1 + 1;                       /* 修正 */

    if (signal <= t1)                       /* 信号低于下限 */
        return 100;                         /* 不降�?*/

    if (signal >= t2)                       /* 信号高于上限 */
        return min_pct;                     /* 最大降�?*/

    /* 中间线性插�?*/
    return (int16)(100 - ((int32)(100 - min_pct) * (signal - t1)) / (t2 - t1)); /* 线性插�?*/
}

/* ======================== 前瞻/趋势降�?======================== */

/* calc_lookahead_speed_pct - 根据前瞻误差和趋势误差计算降速百分比
 * @pos_err_abs: 位置误差绝对�? * 返回: 速度百分比，100=不降�?*/
static int16 calc_lookahead_speed_pct(int16 pos_err_abs) /* 前瞻/趋势降速百分比 */
{
    int16 la_abs = abs_i16(g_tf.lookahead_error); /* 前瞻误差绝对�?*/
    int16 trend_abs = abs_i16(g_tf.error_trend);   /* 趋势误差绝对�?*/
    /* 前瞻降速百分比 */
    int16 la_pct = calc_signal_speed_pct(la_abs,          /* 前瞻信号 */
                                         SPEED_LOOKAHEAD_SLOW_T1, /* 前瞻降速下�?*/
                                         SPEED_LOOKAHEAD_SLOW_T2, /* 上限28 */
                                         SPEED_LOOKAHEAD_MIN_PCT); /* 前瞻最低速度百分�?*/
    /* 趋势降速百分比 */
    int16 trend_pct = calc_signal_speed_pct(trend_abs,     /* 趋势信号 */
                                            SPEED_TREND_SLOW_T1,   /* 趋势降速下�?*/
                                            SPEED_TREND_SLOW_T2,   /* 上限24 */
                                            SPEED_TREND_MIN_PCT);  /* 趋势最低速度百分�?*/
    /* 取较小值（降速更多） */
    int16 pct = (la_pct < trend_pct) ? la_pct : trend_pct; /* 取降速更多的 */

    /* 误差极大时叠加误差自适应降�?*/
    if (pos_err_abs > sp_err_t2)            /* 误差超过自适应上限 */
        pct = (pct < sp_ratio_2) ? pct : sp_ratio_2; /* 取更小�?*/

    return pct;
}

/* ======================== 视觉质量降�?======================== */

/* calc_quality_speed_pct - 根据视觉质量计算降速百分比
 * 降速因素：
 *   1. 有效行数�?�?降速（有效行越少，赛道信息越不可靠�? *   2. 误差跳变�?�?降速（帧间误差突变说明跟踪不稳定）
 * 取两者中降速更多的�? * 返回: 速度百分比，100=不降�?*/
static int16 calc_quality_speed_pct(void) /* 视觉质量降速百分比 */
{
    int16 pct = 100;                        /* 初始百分�?00%（不降速） */

    /* 有效行数不足时降�?*/
    if (g_tf.valid_row_count < SPEED_QUALITY_GOOD_ROWS) /* 有效行不�?35) */
    {
        int16 row_pct = SPEED_QUALITY_ROW_MIN_PCT; /* 最低百分比(70%) */
        uint16 span = SPEED_QUALITY_GOOD_ROWS - SPEED_VISION_BAD_VALID_ROWS; /* 区间宽度 */

        if (g_tf.valid_row_count <= SPEED_VISION_BAD_VALID_ROWS) /* 有效行极�?�?8) */
        {
            /* 有效行极少，最大降�?*/
            row_pct = SPEED_VISION_BAD_PCT; /* 最大降�?40%) */
        }
        else if (span > 0u)                 /* 区间有效 */
        {
            /* 线性插�?*/
            row_pct = (int16)(SPEED_VISION_BAD_PCT + /* 40% + */
                      ((int32)(SPEED_QUALITY_ROW_MIN_PCT - SPEED_VISION_BAD_PCT) *
                      (int32)(g_tf.valid_row_count - SPEED_VISION_BAD_VALID_ROWS)) /
                      (int32)span);          /* 线性插值计�?*/
        }

        if (row_pct < pct)                  /* 行降速更�?*/
            pct = row_pct;                  /* 更新百分�?*/
    }

    /* 误差跳变检测：当前帧与上帧误差差�?*/
    if (s_prev_quality_err_valid)            /* 上帧误差有效 */
    {
        int16 err_jump = abs_i16((int16)(g_tf.error - s_prev_quality_err)); /* 误差跳变�?*/
        int16 jump_pct = calc_signal_speed_pct(err_jump,   /* 跳变信号 */
                                               SPEED_ERR_JUMP_T1,   /* 下限10 */
                                               SPEED_ERR_JUMP_T2,   /* 上限28 */
                                               SPEED_ERR_JUMP_MIN_PCT); /* 最�?0% */
        if (jump_pct < pct)                 /* 跳变降速更�?*/
            pct = jump_pct;                 /* 更新百分�?*/
    }

    s_prev_quality_err = g_tf.error;        /* 保存当前误差供下帧比�?*/
    s_prev_quality_err_valid = 1u;          /* 标记上帧误差有效 */
    speed_dbg_vq_pct = (uint8)pct;          /* 记录视觉质量百分�?*/

    return pct;
}

/* ======================== 转向差速限�?======================== */

/* limit_normal_steer - 限制转向输出（差速）的幅�? * @steer: 转向�? * @speed_out: 速度输出�? * 差速限�?= 速度输出 * 百分比，再限幅到 [min, max]
 * 不同模式使用不同百分比：正常95%、直�?5%、RECOVER 120%
 * 返回: 限幅后的转向�?*/
static float limit_normal_steer(float steer, float speed_out) /* 转向差速限�?*/
{
    int16 pct = STEER_DIFF_NORMAL_PCT;      /* 默认百分�?*/

    if (s_straight_active)                  /* 直道模式 */
        pct = STEER_DIFF_STRAIGHT_PCT;      /* 直道百分�?5% */

    if (s_ra_state == RA_ST_ACTIVE && s_ra_phase == RA_PH_RECOVER) /* RECOVER阶段 */
        pct = STEER_DIFF_RECOVER_PCT;       /* RECOVER百分�?20% */

    uint8 recover_mode =
        (s_ra_state == RA_ST_ACTIVE && s_ra_phase == RA_PH_RECOVER) ? 1u : 0u;
    float speed_abs = abs_f(speed_out);
    float max_limit = STEER_DIFF_MAX_LIMIT;
    float limit = speed_abs * (float)pct * 0.01f; /* 差速限幅�?= 速度×百分�?*/

    if (!recover_mode && speed_out <= 0.0f)
        return 0.0f;

    if (!recover_mode && speed_out > 0.0f)
    {
        float no_reverse_limit = speed_out * 0.95f;
        if (max_limit > no_reverse_limit)
            max_limit = no_reverse_limit;
    }

    if (recover_mode)
    {
        limit = clamp_f(limit, STEER_DIFF_MIN_LIMIT, max_limit);
    }
    else
    {
        float min_limit = STEER_DIFF_MIN_LIMIT;
        if (min_limit > max_limit)
            min_limit = max_limit;
        limit = clamp_f(limit, min_limit, max_limit);
    }

    return clamp_f(steer, -limit, limit);   /* 将转向值限幅到±limit */
}

static float recover_steer_scale(void)
{
    return (float)RA_RECOVER_STEER_PCT * 0.01f;
}

static uint8 ra_hard_exit_ready(uint8 direct_fast,
                                uint8 min_hard,
                                uint8 hard_limit,
                                uint8 line_ok,
                                float hard_yaw_target,
                                float yaw_progress,
                                float yaw_progress_rate)
{
    (void)direct_fast;

    if (!imu_ready || imu_error || hard_yaw_target <= 0.0f)
    {
        if (s_ra_hard_cnt < min_hard)
            return 0u;
        if (line_ok && s_ra_exit_good_cnt >= RA_EXIT_CONFIRM_FRAMES)
            return 1u;
        return (s_ra_hard_cnt >=
                (uint16)((uint16)hard_limit + (uint16)RA_HARD_FORCE_TIMEOUT_EXTRA)) ? 1u : 0u;
    }

    if (s_ra_hard_cnt < min_hard)
        return 0u;

    if (yaw_progress >= hard_yaw_target)
        return 1u;

    if (yaw_progress_rate >= RA_HARD_COAST_YAW_RATE &&
        yaw_progress >= hard_yaw_target - RA_HARD_COAST_REMAIN_DEG)
        return 1u;

    if (s_ra_hard_cnt >=
        (uint16)((uint16)hard_limit + (uint16)RA_HARD_FORCE_TIMEOUT_EXTRA) &&
        yaw_progress >= hard_yaw_target - RA_HARD_TIMEOUT_EXIT_MARGIN)
        return 1u;

    if (s_ra_hard_cnt >=
        (uint16)((uint16)hard_limit +
                 (uint16)RA_HARD_FORCE_TIMEOUT_EXTRA +
                 (uint16)RA_HARD_EMERGENCY_TIMEOUT_EXTRA))
        return 1u;

    return 0u;
}

/* ======================== 速度规划主函�?======================== */

/* plan_base_speed - 基础速度规划（每PID周期调用�? * @target: 原始目标速度
 * @pos_err_abs: 位置误差绝对�? * @pre_slow_active: 预减速激活标�? * 返回: 规划后的基础速度（经斜坡处理�? *
 * 规划优先级：
 *   1. RA活跃或预减�?�?直接用目标速度
 *   2. 丢线 �?25%速度
 *   3. 有效行极�?�?40%速度
 *   4. 单边稳定通过 �?120%加速（用于最后T右直行）
 *   5. 元器件直�?�?120%加速（跳过质量/前瞻降速）
 *   6. 视觉质量降速（有效�?误差跳变�? *   7. 对称组件直道 �?120%加速（立即�? *   8. 前瞻/趋势降�? *   9. 直道确认 �?120%加速（需连续N帧确认） */
static int16 plan_base_speed(int16 target, int16 pos_err_abs, uint8 pre_slow_active) /* 速度规划 */
{
    s_straight_active = 0u;                 /* 直道激活标志清�?*/
    speed_dbg_vq_pct = 100u;               /* 视觉质量百分比重�?*/

    /* RA活跃或预减速期间，不做额外规划 */
    if (s_ra_state != RA_ST_NONE || pre_slow_active) /* RA活跃或预减�?*/
    {
        s_straight_cnt = 0u;                /* 直道计数清零 */

        s_straight_hold = 0u;

        if (!pre_slow_active && single_edge_speed_candidate(pos_err_abs))
        {
            s_straight_cnt = SPEED_STRAIGHT_CONFIRM_FRAMES;
            s_straight_active = 1u;
            target = (int16)((int32)target * SPEED_SINGLE_EDGE_BOOST_PCT / 100);
            return speed_ramp_apply_reason(target, 9u);
        }

        return speed_ramp_apply_reason(target, 1u); /* 直接用目标速度，原�?RA */
    }

    /* 丢线时大幅降�?*/
    if (g_tf.line_lost != 0u)              /* 丢线 */
    {
        s_straight_cnt = 0u;                /* 直道计数清零 */
        s_straight_hold = 0u;
        speed_dbg_vq_pct = SPEED_LINE_LOST_PCT; /* 记录降速百分比(25%) */
        target = (int16)((int32)target * SPEED_LINE_LOST_PCT / 100); /* 降速到25% */
        return speed_ramp_apply_reason(target, 2u); /* 返回，原�?丢线 */
    }

    /* 有效行极少时降�?*/
    if (g_tf.valid_row_count < SPEED_VISION_BAD_VALID_ROWS) /* 有效行极�?�?8) */
    {
        s_straight_cnt = 0u;                /* 直道计数清零 */
        s_straight_hold = 0u;
        speed_dbg_vq_pct = SPEED_VISION_BAD_PCT; /* 记录降速百分比(40%) */
        target = (int16)((int32)target * SPEED_VISION_BAD_PCT / 100); /* 降速到40% */
        return speed_ramp_apply_reason(target, 3u); /* 返回，原�?视觉�?*/
    }

    if (component_fast_speed_candidate(pos_err_abs))
    {
        return straight_boost_speed(target, 4u);
    }

    if (single_edge_speed_candidate(pos_err_abs))
    {
        s_straight_cnt = SPEED_STRAIGHT_CONFIRM_FRAMES;
        s_straight_active = 1u;
        target = (int16)((int32)target * SPEED_SINGLE_EDGE_BOOST_PCT / 100);
        return speed_ramp_apply_reason(target, 9u);
    }

    if (sym_straight_speed_candidate(pos_err_abs))
    {
        return straight_boost_speed(target, 4u);
    }

    if (straight_speed_candidate(pos_err_abs))
    {
        if (s_straight_cnt < 255u)
            s_straight_cnt++;

        if (s_straight_cnt >= SPEED_STRAIGHT_CONFIRM_FRAMES)
        {
            return straight_boost_speed(target, 6u);
        }
    }
    else
    {
        s_straight_cnt = 0u;
    }

    if (s_straight_hold > 0u && straight_hold_safe(pos_err_abs))
    {
        s_straight_hold--;
        s_straight_active = 1u;
        target = (int16)((int32)target * SPEED_STRAIGHT_BOOST_PCT / 100);
        return speed_ramp_apply_reason(target, 6u);
    }
    s_straight_hold = 0u;

    if (g_tf.valid_row_count >= SPEED_VALID_RUSH_ROWS &&
        g_tf.line_lost == 0u &&
        g_ra_pre_flag == 0u &&
        g_ra_flag == 0u)
    {
        s_straight_active = 1u;
        target = (int16)((int32)target * SPEED_VALID_RUSH_PCT / 100);
        return speed_ramp_apply_reason(target, 6u);
    }

    {
        int16 quality_pct = calc_quality_speed_pct();
        int16 curve_pct = calc_lookahead_speed_pct(pos_err_abs);
        int16 normal_pct = (quality_pct < curve_pct) ? quality_pct : curve_pct;
        uint8 reason = 8u;

        if (normal_pct < 100)
            reason = (curve_pct <= quality_pct) ? 5u : 7u;

        if (g_tf.valid_row_count >= SPEED_VALID_RUN_ROWS &&
            normal_pct < SPEED_VALID_RUN_MIN_PCT)
        {
            normal_pct = SPEED_VALID_RUN_MIN_PCT;
        }

        target = apply_speed_pct(target, normal_pct);
        return speed_ramp_apply_reason(target, reason);
    }
}
static uint8 route_action_from_flag(uint8 flag) /* 根据flag推断动作 */
{
    if (flag == 3u)                         /* T左路�?*/
        return ACT_LEFT;                    /* 左转 */
    if (flag == 4u)                         /* T右路�?*/
        return ACT_RIGHT;                   /* 右转 */
    if (flag == 1u)                         /* 右直�?*/
        return ACT_RIGHT;                   /* 右转 */
    if (flag == 2u)                         /* 左直�?*/
        return ACT_LEFT;                    /* 左转 */

    return ACT_STRAIGHT;                    /* 其他默认直行 */
}

/* fallback_intersection_decision - 保底路口决策（路线表未匹配时的默认行为）
 * @flag: 路口类型flag
 * 返回: 路口决策结构�?*/
static RouteDecision fallback_intersection_decision(uint8 flag) /* 保底路口决策 */
{
    RouteDecision d = { ACT_STRAIGHT, EDGE_BOTH, 0u, 0u }; /* 初始化为直行，无单边 */

    d.action = route_action_from_flag(flag); /* 根据flag推断动作 */
    return d;                               /* 返回决策 */
}

/* ======================== 路线表匹�?======================== */

/* select_intersection_decision - 根据当前flag从路线表中选择动作
 * @flag: 当前检测到的路口类�? * 返回: 路口决策（包含动作、单边配置、有效标志）
 *
 * 流程�? *   1. 先查路线表中flag+count匹配的条�? *   2. 匹配�?�?消耗该条目，返回对应动�? *   3. 未匹�?�?检查是否启用保底直�? *   4. 都没�?�?返回默认决策（flag=1/2不消耗，flag=3/4/5不动作） */
static RouteDecision select_intersection_decision(uint8 flag) /* 路线表匹�?*/
{
    RouteDecision d = fallback_intersection_decision(flag); /* 初始化保底决�?*/
    uint8 next_count;                       /* 下一个期望的count */

    if (flag >= 7u)                         /* flag越界 */
        return d;                           /* 返回保底决策 */

    next_count = s_inter_count[flag] + 1u;  /* 下一个期望的count�?*/

    /* 遍历路线表查找匹�?*/
    for (uint8 i = 0u; i < USER_RULE_COUNT; i++) /* 遍历路线�?*/
    {
        if (user_rules[i].flag == flag &&   /* flag类型匹配 */
            user_rules[i].count == next_count) /* count值匹�?*/
        {
            /* 消耗该条目 */
            s_inter_count[flag] = next_count; /* 更新已匹配计�?*/

            /* ACT_AUTO时根据flag推断动作 */
            d.action = (user_rules[i].action == ACT_AUTO) ? /* 自动模式 */
                       route_action_from_flag(flag) :        /* 根据flag推断 */
                       user_rules[i].action;                 /* 使用规则指定动作 */
            d.post_edge_side = user_rules[i].post_edge_side; /* 获取单边方向 */
            d.post_edge_ms = user_rules[i].post_edge_ms;    /* 获取单边时间 */
            d.valid = 1u;                   /* 标记决策有效 */

            /* 保存待提交的调试信息 */
            s_route_pending_valid = 1u;     /* 待提交有�?*/
            s_route_pending_flag = flag;    /* 待提交flag */
            s_route_pending_count = next_count; /* 待提交count */
            s_route_pending_action = d.action; /* 待提交动�?*/
            return d;                       /* 返回决策 */
        }
    }

    /* 路线表未匹配，检查保底直�?*/
    if (ra_fallback_direct_enabled(flag))   /* 保底直转已启�?*/
    {
        d.action = route_action_from_flag(flag); /* 根据flag推断动作 */
        d.post_edge_side = EDGE_BOTH;       /* 无单�?*/
        d.post_edge_ms = 0u;                /* 无单边时�?*/
        d.valid = 1u;                       /* 标记决策有效 */
        return d;                           /* 返回决策 */
    }

    /* 未匹配路线表时：直角1/2保底自动转，普通路口不消耗计数�?*/
    return d;                               /* 返回保底决策 */
}

static uint8 ra_try_start_direct_flag(void)
{
    if ((g_ra_flag != 1u && g_ra_flag != 2u) || s_ra_state != RA_ST_NONE)
        return 0u;

    RouteDecision d = select_intersection_decision((uint8)g_ra_flag);
    uint8 action = (d.action == ACT_RIGHT || d.action == ACT_LEFT) ?
                   d.action :
                   route_action_from_flag((uint8)g_ra_flag);

    if (!d.valid)
    {
        g_ra_flag = 0u;
        ra_debug_update();
        return 1u;
    }

    ra_start((action == ACT_RIGHT) ? 1u : 2u,
             (uint8)g_ra_flag,
             0u,
             d.post_edge_side,
             d.post_edge_ms);
    ra_clear_pre_flags();
    return 0u;
}

static uint8 ra_try_start_intersection_flag(void)
{
    if ((g_ra_flag < 3u || g_ra_flag > 5u) || s_ra_state != RA_ST_NONE)
        return 0u;

    if (g_post_edge_side != EDGE_BOTH)
    {
        ra_clear_all_flags();
        ra_debug_update();
        return 1u;
    }

    RouteDecision d = select_intersection_decision((uint8)g_ra_flag);

    if (!d.valid)
    {
        g_ra_flag = 0u;
        ra_debug_update();
        return 1u;
    }

    if (d.action == ACT_RIGHT || d.action == ACT_LEFT)
    {
        ra_start((d.action == ACT_RIGHT) ? 1u : 2u,
                 (uint8)g_ra_flag,
                 0u,
                 d.post_edge_side,
                 d.post_edge_ms);
        ra_clear_pre_flags();
    }
    else
    {
        ra_start(0u,
                 (uint8)g_ra_flag,
                 1u,
                 d.post_edge_side,
                 d.post_edge_ms);
        ra_clear_pre_flags();
    }

    return 0u;
}

static void ra_tick_active(void)
{
    s_ra_timer++;
    s_ra_phase_cnt++;

    if (g_ip_max_row > s_ra_ip_row)
        s_ra_ip_row = g_ip_max_row;
}

static uint8 ra_handle_timeout(void)
{
    if (s_ra_timer <= RA_TIMEOUT_FRAMES)
        return 0u;

    ra_finish();
    return 1u;
}

static uint8 ra_handle_straight_phase(RaResult *r)
{
    if (!s_ra_straight)
        return 0u;

    if (s_ra_timer >= RA_STRAIGHT_FRAMES)
    {
        ra_finish();
        return 1u;
    }

    if (g_post_edge_side != EDGE_BOTH)
        r->speed_scale = 1.0f;
    else
        r->speed_scale = (float)RA_STRAIGHT_SPEED_PCT * 0.01f;

    r->need_pid = 1u;
    ra_debug_update();
    return 1u;
}

static uint8 ra_handle_recover_phase(RaResult *r)
{
    uint8 recover_line_ok;

    if (s_ra_phase != RA_PH_RECOVER)
        return 0u;

    s_ra_recover_cnt++;
    ra_clear_all_flags();

    recover_line_ok =
        (g_tf.line_lost == 0u &&
         g_tf.valid_row_count >= RA_RECOVER_VALID_ROWS &&
         abs_i16(g_tf.error) <= RA_RECOVER_ERROR_MAX &&
         abs_i16(g_tf.lookahead_error) <= RA_RECOVER_LOOKAHEAD_MAX &&
         abs_i16(g_tf.error_trend) <= RA_RECOVER_TREND_MAX &&
         (!imu_ready || imu_error ||
          (abs_f((float)yaw_rate) <= RA_RECOVER_YAW_RATE_MAX &&
           abs_f((float)ra_hard_yaw - ra_yaw_progress()) <= RA_RECOVER_YAW_ERROR_MAX))) ? 1u : 0u;

    if (recover_line_ok)
    {
        if (s_ra_recover_good_cnt < 255u)
            s_ra_recover_good_cnt++;
    }
    else
    {
        s_ra_recover_good_cnt = 0u;
    }

    if (s_ra_recover_cnt >= RA_RECOVER_FIXED_FRAMES &&
        s_ra_recover_good_cnt >= RA_RECOVER_CONFIRM_FRAMES)
    {
        ra_finish();
        return 1u;
    }

    if (s_ra_recover_cnt >= RA_RECOVER_MAX_FRAMES &&
        (!imu_ready || imu_error ||
         (abs_f((float)yaw_rate) <= (RA_RECOVER_YAW_RATE_MAX * 2.0f) &&
          abs_f((float)ra_hard_yaw - ra_yaw_progress()) <=
          (RA_RECOVER_YAW_ERROR_MAX * 2.0f))))
    {
        ra_finish();
        return 1u;
    }

    r->speed_scale = (float)RA_RECOVER_SPEED_PCT * 0.01f;
    r->need_pid = 1u;
    ra_debug_update();
    return 1u;
}

static uint8 ra_step_wait_slow_approach(RaResult *r)
{
    if (s_ra_phase == RA_PH_WAIT)
    {
        uint8 slow_row = ra_slow_trigger_row();

        if (s_ra_ip_row >= slow_row)
        {
            s_ra_phase = RA_PH_SLOW;
            s_ra_phase_cnt = 0u;
            s_speed_integral = 0.0f;
        }
    }
    else if (s_ra_phase == RA_PH_SLOW)
    {
        uint8 turn_row = ra_turn_trigger_row();
        uint8 slow_row = ra_slow_trigger_row();

        if (s_ra_ip_row >= turn_row ||
            (s_ra_phase_cnt >= RA_SLOW_TO_HARD_FALLBACK_FRAMES &&
             s_ra_ip_row >= slow_row))
        {
            s_ra_phase = RA_PH_APPROACH;
            s_ra_approach_cnt = 0u;
            s_ra_phase_cnt = 0u;
            s_speed_integral = 0.0f;
        }
    }
    else if (s_ra_phase == RA_PH_APPROACH)
    {
        uint16 approach_frames =
            ra_approach_frames_for_speed(ra_turn_row_for_speed());

        if (s_ra_approach_cnt < approach_frames)
        {
            s_ra_approach_cnt++;
            r->speed_scale = (float)RA_APPROACH_SPEED_PCT * 0.01f;
            r->need_pid = 1u;
            ra_debug_update();
            return 1u;
        }

        ra_enter_hard();
    }

    return 0u;
}

static uint8 ra_handle_hard_phase(int16 pos_err_abs, RaResult *r)
{
    if (s_ra_phase != RA_PH_HARD)
        return 0u;

    uint8 direct_fast = (s_ra_orig_flag < 3u &&
                         ra_speed_ref() >= RA_FAST_SPEED_START) ? 1u : 0u;
    uint8 min_hard = (s_ra_orig_flag >= 3u) ?
                     RA_CROSS_HARD_MIN_FRAMES :
                     (direct_fast ? RA_FAST_HARD_MIN_FRAMES : RA_HARD_MIN_FRAMES);
    uint8 hard_limit = (s_ra_orig_flag >= 3u) ?
                       RA_CROSS_HARD_TIMEOUT :
                       (direct_fast ? RA_FAST_HARD_TIMEOUT : RA_HARD_TIMEOUT);
    uint8 line_ok = (g_tf.line_lost == 0u &&
                     g_tf.valid_row_count >= RA_EXIT_VALID_ROWS &&
                     pos_err_abs <= RA_EXIT_ERROR_MAX) ? 1u : 0u;
    float hard_yaw_target = (float)ra_hard_yaw +
                            (direct_fast ? RA_FAST_DIRECT_YAW_OFFSET : 0.0f);
    float yaw_progress;
    float outer;
    float inner;
    float out_l;
    float out_r;
    float yaw_progress_rate;
    uint8 ramp_frames;

    s_ra_hard_cnt++;

    if (line_ok)
        s_ra_exit_good_cnt++;
    else
        s_ra_exit_good_cnt = 0u;

    yaw_progress = ra_yaw_progress();
    yaw_progress_rate = ra_yaw_progress_rate();
    if (ra_hard_exit_ready(direct_fast,
                           min_hard,
                           hard_limit,
                           line_ok,
                           hard_yaw_target,
                           yaw_progress,
                           yaw_progress_rate))
    {
        ra_enter_recover();
        return 1u;
    }

    outer = (float)ra_hard_outer *
            (float)(direct_fast ? RA_FAST_HARD_OUTER_PCT : RA_HARD_OUTER_PCT) * 0.01f;
    inner = RA_HARD_INNER_DUTY;

    if (s_ra_orig_flag >= 3u)
    {
        outer = outer * (float)RA_COMPLEX_DUTY_PCT * 0.01f;
    }

    {
        float outer_min = (s_ra_orig_flag >= 3u) ?
                          RA_COMPLEX_OUTER_MIN_DUTY :
                          RA_HARD_OUTER_MIN_DUTY;
        if (outer < outer_min)
            outer = outer_min;
    }

    if (imu_ready && !imu_error && hard_yaw_target > 1.0f)
    {
        float yaw_ratio = yaw_progress / hard_yaw_target;
        float taper_start = direct_fast ?
                            RA_FAST_HARD_TAPER_START_RATIO :
                            RA_HARD_TAPER_START_RATIO;
        float taper_end = direct_fast ?
                          RA_FAST_HARD_TAPER_END_RATIO :
                          RA_HARD_TAPER_END_RATIO;
        float min_scale = (float)(direct_fast ?
                                  RA_FAST_HARD_TAPER_MIN_PCT :
                                  RA_HARD_TAPER_MIN_PCT) * 0.01f;

        if (yaw_ratio > taper_start)
        {
            float span = taper_end - taper_start;
            float t;
            float taper;

            if (span < 0.01f)
                span = 0.01f;
            t = (yaw_ratio - taper_start) / span;

            if (t > 1.0f)
                t = 1.0f;
            if (t < 0.0f)
                t = 0.0f;

            taper = 1.0f - t * (1.0f - min_scale);
            outer *= taper;
        }
    }

    if (imu_ready && !imu_error)
    {
        if (yaw_progress_rate > RA_HARD_YAW_RATE_SOFT_LIMIT)
        {
            float rate_scale =
                1.0f - (yaw_progress_rate - RA_HARD_YAW_RATE_SOFT_LIMIT) /
                RA_HARD_YAW_RATE_SOFT_LIMIT;
            float rate_min =
                (float)RA_HARD_YAW_RATE_MIN_PCT * 0.01f;

            if (rate_scale < rate_min)
                rate_scale = rate_min;
            if (rate_scale > 1.0f)
                rate_scale = 1.0f;

            outer *= rate_scale;
        }
    }

    if (s_ra_dir == 1u)
    {
        out_l = inner;
        out_r = outer;
    }
    else
    {
        out_l = outer;
        out_r = inner;
    }

    ramp_frames = direct_fast ? RA_FAST_HARD_RAMP_FRAMES : RA_HARD_RAMP_FRAMES;
    if (ramp_frames < 1u)
        ramp_frames = 1u;

    if (s_ra_hard_cnt <= ramp_frames)
    {
        float ramp = (float)s_ra_hard_cnt / (float)ramp_frames;
        float ramp_min = direct_fast ? 0.35f : 0.20f;
        if (ramp < ramp_min)
            ramp = ramp_min;
        out_l *= ramp;
        out_r *= ramp;
    }

    s_ra_hard_speed_seed = (out_l + out_r) * 0.5f;
    s_ra_hard_steer_seed = (out_l - out_r) * 0.5f;
    speed_dbg_out = (int16)s_ra_hard_speed_seed;
    steer_dbg_out = (int16)s_ra_hard_steer_seed;

    pid_set_duty(clamp_duty(out_l), clamp_duty(out_r));
    r->should_return = 1u;
    ra_debug_update();
    return 1u;
}

static void ra_apply_slow_scale(RaResult *r)
{
    if (s_ra_phase == RA_PH_SLOW || s_ra_phase == RA_PH_APPROACH)
    {
        uint8 slow_pct = (uint8)ra_slow_pct;

        r->speed_scale = (float)slow_pct * 0.01f;
    }
}

/* ======================== RA状态机主逻辑 ======================== */

/* ra_state_machine_step - RA状态机每帧执行
 * @pos_err_abs: 位置误差绝对�? * 返回: RaResult（need_pid/should_return/speed_scale�? *
 * 状态转换：
 *   检测到flag(1/2) �?从路线表选动�?�?ra_start()
 *   检测到flag(3/4/5) �?从路线表选动�?�?ra_start()
 *   WAIT阶段 �?等待拐点行达到阈�?�?SLOW
 *   SLOW阶段 �?等待拐点行达到转弯阈�?�?APPROACH
 *   APPROACH阶段 �?等待N�?�?HARD
 *   HARD阶段 �?固定duty驱动 �?摄像头恢�?yaw达标/超时 �?RECOVER
 *   RECOVER阶段 �?PID恢复巡线 �?确认稳定 �?结束 */
static RaResult ra_state_machine_step(int16 pos_err_abs) /* RA状态机主逻辑 */
{
    RaResult r = { 0u, 0u, 1.0f };          /* 初始化：需要PID，不跳过，速度缩放100% */

    if (ra_try_start_direct_flag())
        return r;

    if (ra_try_start_intersection_flag())
        return r;

    /* RA未激�?�?返回默认�?*/
    if (s_ra_state != RA_ST_ACTIVE)          /* RA未激�?*/
    {
        ra_debug_update();                   /* 更新调试 */
        return r;                            /* 返回默认结果 */
    }

    ra_tick_active();

    if (ra_handle_timeout())
        return r;

    if (ra_handle_straight_phase(&r))
        return r;

    if (ra_handle_recover_phase(&r))
        return r;

    if (ra_step_wait_slow_approach(&r))
        return r;

    if (ra_handle_hard_phase(pos_err_abs, &r))
        return r;

    ra_apply_slow_scale(&r);

    r.need_pid = 1u;                         /* 需要PID控制 */
    ra_debug_update();                       /* 更新调试 */
    return r;                                /* 返回结果 */
}

/* ======================== 正常PID执行 ======================== */

/* normal_pid_step - 正常巡线PID控制（非HARD阶段时调用）
 * @pos_err: 位置误差
 * @pos_err_abs: 位置误差绝对�? *
 * 流程�? *   1. 计算转向增益调度参数
 *   2. 计算前馈信号（基于前瞻误差）
 *   3. 选择cascade或普通PD计算转向
 *   4. 计算自适应速度 + 速度PI + 速度前馈
 *   5. 速度因子缩放转向（高速时温和补偿�? *   6. 直道/RECOVER模式转向缩放
 *   7. 差速限�? *   8. 输出：左�?速度+转向，右�?速度-转向 */
static void normal_pid_step(int16 pos_err, int16 pos_err_abs) /* 正常PID控制 */
{
    /* 计算增益调度参数 */
    SteerSchedule sch = steer_schedule_calc(pos_err_abs); /* 根据速度和弯道计算增�?*/

    /* 前馈信号：KP * ff_scale * 前瞻误差 */
    float ff_raw = STEER_KP * sch.ff_scale * (float)g_tf.lookahead_error; /* 原始前馈 */
    float steer_ff = 0.0f;                  /* 有效前馈值，初始�? */

    ff_raw = clamp_f(ff_raw, -STEER_FF_MAX, STEER_FF_MAX); /* 前馈限幅 */
    /* 前馈低通滤�?*/
    s_steer_ff_filtered = s_steer_ff_filtered * STEER_FF_FILTER_ALPHA + /* 上一帧�?.55 */
                          ff_raw * (1.0f - STEER_FF_FILTER_ALPHA);      /* 新值�?.45 */

    /* 路口附近抑制前馈 */
    if (g_ra_pre_flag == 0u && g_ra_flag == 0u) /* 无RA标志 */
        steer_ff = s_steer_ff_filtered;     /* 使用滤波后的前馈 */

    float steer;                            /* 转向输出�?*/

    /* 选择转向控制模式：cascade（IMU内环）或普通PD */
    if (cascade_en && imu_ready && !imu_error && /* 串级启用且IMU正常 */
        !(s_ra_state == RA_ST_ACTIVE && s_ra_phase == RA_PH_RECOVER)) /* 且不在RECOVER阶段 */
    {
        steer = cascade_steer_calc(pos_err,
                                   sch.kp_scale,
                                   sch.kd_scale,
                                   sch.slew_max); /* 使用串级PID */
    }
    else                                    /* 使用普通PD */
    {
        steer = steer_pd_calc(pos_err,      /* 位置误差 */
                              sch.kp_scale, /* 比例增益缩放 */
                              sch.kd_scale, /* 微分增益缩放 */
                              steer_ff,     /* 前馈信号 */
                              sch.slew_max); /* 变化率限�?*/

#if YAW_COMP_ENABLE                         /* yaw角补偿（默认关闭�?*/
        /* yaw角补偿（默认关闭�?*/
        {
            float yaw_kp_val = (float)yaw_kp / 10.0f; /* yaw比例增益 */
            float yaw_abs = abs_f(yaw_angle);          /* yaw角绝对�?*/
            if (yaw_abs > YAW_DEADZONE)                 /* 超过死区(1�? */
                steer += yaw_kp_val * yaw_angle;        /* 添加yaw补偿 */
        }
#endif

        /* Independent yaw damping (cascade_en=0 only). 独立yaw阻尼 */
        if (yaw_damp_gain != 0 && imu_ready && !imu_error && /* 阻尼增益非零且IMU正常 */
            !(s_ra_state == RA_ST_ACTIVE && s_ra_phase == RA_PH_RECOVER)) /* 不在RECOVER */
        {
            steer -= (float)yaw_damp_gain * YAW_DAMP_SCALE * (float)yaw_rate; /* 减去阻尼�?*/
        }
    }

    /* ===== 速度计算 ===== */
    int16 trend_abs = abs_i16(g_tf.error_trend); /* 趋势误差绝对�?*/
    int16 speed_err = pos_err_abs;          /* 速度误差基准 = 位置误差 */
    uint8 component_fast = component_fast_speed_candidate(pos_err_abs);

    /* 高速时将趋势误差叠加到速度误差中（高速弯道更谨慎�?*/
    if (base_speed > 200)                   /* 速度超过200 */
    {
        float trend_factor = (float)(base_speed - 200) / 800.0f; /* 趋势因子 */
        if (trend_factor > 0.5f) trend_factor = 0.5f; /* 上限50% */
        speed_err = pos_err_abs + (int16)((float)trend_abs * trend_factor); /* 叠加趋势 */
    }

    /* 自适应速度目标 */
    uint8 valid_run_fast =
        (g_tf.line_lost == 0u &&
         g_tf.valid_row_count >= SPEED_VALID_RUN_ROWS &&
         g_ra_pre_flag == 0u &&
         g_ra_flag == 0u) ? 1u : 0u;

    uint8 ra_pid_speed_direct = (s_ra_state == RA_ST_ACTIVE) ? 1u : 0u;
    float target_speed = (ra_pid_speed_direct ||
                          component_fast || s_straight_active || valid_run_fast) ?
                         (float)base_speed :
                         (float)calc_adapted_speed(base_speed, speed_err); /* 误差自适应速度 */
    float actual_l = (float)motor_value.receive_left_speed_data;  /* 左轮实际速度 */
    float actual_r = (float)motor_value.receive_right_speed_data; /* 右轮实际速度 */
    float avg_actual = (actual_l + actual_r) * 0.5f; /* 左右轮平均速度 */

    /* 加速度前馈：目标速度变化�?* 增益 */
    float accel_ff = 0.0f;                  /* 加速度前馈，初始为0 */
    if (s_speed_ff_ready)                   /* 前馈就绪（非首周期） */
        accel_ff = (target_speed - s_prev_target_speed) * PID_D_SCALE * SPEED_ACCEL_FF_GAIN; /* 目标变化量前�?*/
    else                                    /* 首周�?*/
        s_speed_ff_ready = 1u;              /* 标记前馈就绪 */

    accel_ff = clamp_f(accel_ff, -SPEED_ACCEL_FF_LIMIT, SPEED_ACCEL_FF_LIMIT); /* 限幅加速度前馈 */
    s_prev_target_speed = target_speed;     /* 保存当前目标速度 */

    /* 速度输出 = 前馈 + PI */
    float speed_ff = target_speed * SPEED_FF_RATIO + accel_ff; /* 速度前馈 + 加速度前馈 */
    float speed_out = speed_ff + speed_pi_calc(target_speed,   /* 前馈 + 速度PI */
                                               avg_actual,     /* 实际速度 */
                                               &s_speed_integral, /* 积分指针 */
                                               pos_err_abs);  /* 位置误差（积分分离用�?*/
    if (target_speed > 0.0f && speed_out < 0.0f)
    {
        speed_out = 0.0f;
        if (s_speed_integral < 0.0f)
            s_speed_integral *= 0.5f;
    }

    /* 速度因子：高速时温和增加转向补偿 */
    int16 turn_signal = pos_err_abs;
    int16 la_abs_for_turn = abs_i16(g_tf.lookahead_error);
    if (la_abs_for_turn > turn_signal) turn_signal = la_abs_for_turn;
    if (trend_abs > turn_signal) turn_signal = trend_abs;

    float curve_boost = range_ratio_i16(turn_signal,
                                        STEER_GAIN_CURVE_T1,
                                        STEER_GAIN_CURVE_T2) * 0.20f;
    float speed_factor = 1.0f +
                         (float)base_speed * (float)steer_speed_k * 0.00025f +
                         curve_boost;       /* 速度因子 */
    if (speed_factor > SPEED_FACTOR_MAX)    /* 超过上限 */
        speed_factor = SPEED_FACTOR_MAX;    /* 限幅 */

    steer *= speed_factor;                  /* 转向乘以速度因子 */
    steer += ra_pre_turn_steer_ff();

    /* 直道时转向缩�?*/
    if (s_straight_active)                  /* 直道加速模�?*/
        steer *= (float)SPEED_STRAIGHT_STEER_PCT * 0.01f; /* 转向缩放(100%) */

    /* RECOVER阶段：转向缩�?+ yaw修正 */
    if (s_ra_state == RA_ST_ACTIVE && s_ra_phase == RA_PH_RECOVER) /* RECOVER阶段 */
    {
        steer *= recover_steer_scale();      /* 转向渐入，避免出弯抖�?*/
        if (imu_ready && !imu_error && s_ra_dir != 0u &&
            RA_RECOVER_YAW_KP > 0.0f)
        {
            float yaw_err = (float)ra_hard_yaw - ra_yaw_progress();
            if (abs_f(yaw_err) > RA_RECOVER_YAW_DEADZONE)
            {
                float yaw_corr =
                    clamp_f(yaw_err * RA_RECOVER_YAW_KP,
                            -RA_RECOVER_YAW_MAX,
                            RA_RECOVER_YAW_MAX);
                steer += (s_ra_dir == 1u) ? -yaw_corr : yaw_corr;
            }
            if (RA_RECOVER_YAW_RATE_KD > 0.0f)
            {
                float rate_corr =
                    clamp_f((float)yaw_rate * RA_RECOVER_YAW_RATE_KD,
                            -RA_RECOVER_YAW_RATE_MAX_CORR,
                            RA_RECOVER_YAW_RATE_MAX_CORR);
                steer -= rate_corr;
            }
        }
    }

    /* 差速限�?*/
    steer = limit_normal_steer(steer, speed_out); /* 限制差速幅�?*/

    /* yaw_rate 限幅：仅在转向与 yaw_rate 同号（即转向正在加剧旋转、有发散风险）
     * 时才衰减；转向在纠正旋转（反号）时不动，避免真弯道跟随被误砍。
     * 并保留下限，绝不把该转的转向清零。 */
    if (imu_ready && !imu_error)              /* IMU正常工作 */
    {
        float yaw_rate_f = (float)yaw_rate;
        float yaw_rate_abs = abs_f(yaw_rate_f); /* 角速度绝对值 */
        /* steer>0 左轮加/右轮减 → 右转，yaw_rate 符号与转向方向一致才算"加剧" */
        uint8 amplifying = ((steer > 0.0f && yaw_rate_f > 0.0f) ||
                            (steer < 0.0f && yaw_rate_f < 0.0f)) ? 1u : 0u;
        if (amplifying && yaw_rate_abs > YAW_RATE_LIMIT) /* 同向且超限才衰减 */
        {
            float scale = 1.0f - (yaw_rate_abs - YAW_RATE_LIMIT) / YAW_RATE_LIMIT; /* 按比例减弱 */
            float min_scale = (float)YAW_RATE_LIMIT_MIN_PCT * 0.01f;
            if (scale < min_scale) scale = min_scale; /* 保留下限，不清零 */
            steer *= scale;                   /* 转向输出按比例缩放 */
        }
    }

    speed_dbg_out = (int16)speed_out;       /* 记录速度调试�?*/
    steer_dbg_out = (int16)steer;           /* 记录转向调试�?*/

    /* 最终输出：左轮=速度+转向，右�?速度-转向 */
    pid_set_duty(clamp_duty(speed_out + steer), /* 左轮 = 速度+转向 */
                          clamp_duty(speed_out - steer)); /* 右轮 = 速度-转向 */
}

static void pid_stop_runtime_reset(uint8 clear_rules_done)
{
    pid_set_duty(0, 0);
    base_speed = 0;
    speed_dbg_out = 0;
    steer_dbg_out = 0;
    speed_dbg_raw = 0;
    speed_dbg_plan = 0;
    speed_dbg_reason = 0u;
    s_speed_integral = 0.0f;
    s_motor_run_counter = 0u;
    if (clear_rules_done)
    {
        s_rules_done = 0u;
        s_rules_done_timer = 0u;
    }
    single_edge_reset();
    s_completed_right_ra_count = 0u;
    lost_search_reset();
    reset_speed_planner();
    reset_speed_ff_state();
    g_ra_pre_slow_flag = 0u;
    s_pre_lock = 0u;
    s_pre_timeout = 0u;
    ra_reset();
}

/* line_pid_control - 主PID控制函数，由PID_PERIOD_MS PIT中断调用（isr.c:cc60_pit_ch0_isr�? * 这是整个控制系统的核心，每PID_PERIOD_MS执行一�? *
 * 执行流程�? *   1. 电机未使�?�?清零所有状态并返回
 *   2. 运行超时 �?停机
 *   3. 路线完成延迟停机
 *   4. 更新转弯屏蔽
 *   5. RA状态机（可能直接输出电机）
 *   6. 丢线搜索（可能直接输出电机）
 *   7. 速度规划（含预减速）
 *   8. 单边巡线tick
 *   9. 正常PID控制 */
void line_pid_control(void)                  /* 主PID控制入口 */
{
    if (run_quiet_active() && quiet_stop_key_pressed())
        motor_enable = 0;

    uint8 vacuum_run_active = (motor_enable != 0) ? 1u : 0u;
#if VACUUM_PREARM_ENABLE
    uint8 vacuum_request = (vacuum_run_active || race_state == RACE_STATE_ARMED) ? 1u : 0u;
#else
    uint8 vacuum_request = vacuum_run_active;
#endif
    vacuum_control_update(vacuum_request, vacuum_run_active);

    /* ===== 电机未使能：全部复位 ===== */
    if (motor_enable == 0)                   /* 电机未使�?*/
    {
        pid_stop_runtime_reset(0u);
        return;                              /* 返回 */
    }

    /* ===== 运行超时保护 ===== */
    s_motor_run_counter++;                   /* 运行帧计数加1 */
    if (s_motor_run_counter >= PID_SECONDS_TO_TICKS((uint32)motor_run_time)) /* 超过运行时间 */
    {
        motor_enable = 0;                    /* 关闭电机使能 */
        vacuum_stop();
        pid_stop_runtime_reset(0u);
        return;                              /* 返回 */
    }

    /* ===== 路线完成延迟停机 ===== */
    if (s_rules_done)                        /* 路线全部完成 */
    {
        s_rules_done_timer++;                /* 延迟计时器加1 */
        if (s_rules_done_timer >= RULES_DONE_DELAY) /* 达到延迟帧数(136=1.5�? */
        {
            motor_enable = 0;                /* 关闭电机使能 */
            vacuum_stop();
            pid_stop_runtime_reset(1u);
            return;                          /* 返回 */
        }
    }


    /* ===== 转弯屏蔽更新 ===== */

    int16 pos_err = g_tf.error;             /* 获取当前位置误差 */
    int16 pos_err_abs = abs_i16(pos_err);   /* 位置误差绝对�?*/

    /* ===== RA状态机 ===== */
    RaResult ra = ra_state_machine_step(pos_err_abs); /* 执行RA状态机 */
    /* HARD阶段RA已直接输出电机，本帧不再执行PID */
    if (ra.should_return)                    /* RA已直接输出电�?*/
    {
        auto_tune_log_pid_tick();
        return;                              /* 跳过本帧PID */
    }

    /* ===== 丢线搜索 ===== */
    /* 丢线搜索已直接输出电机，本帧不再执行PID */
    if (lost_search_step(pos_err))           /* 丢线搜索正在执行 */
    {
        auto_tune_log_pid_tick();
        return;                              /* 跳过本帧PID */
    }

    /* ===== 速度规划 ===== */
    /* 原始目标速度 = 菜单速度 * 8 * RA速度缩放 */
    int16 target_base_speed = (int16)((float)motor_speed * 8.0f * ra.speed_scale); /* 原始目标速度 */
    uint8 pre_slow_active = 0u;              /* 预减速激活标�?*/
    speed_dbg_pre_lock = 0u;                 /* 预减速锁调试标志清零 */
    speed_dbg_raw = target_base_speed;       /* 记录原始目标速度 */

    /* ===== 预减速逻辑 ===== */
    /* �?right_angle_pre_detect() 检测到直角但还未正式触发RA�?     * 提前降低速度，防止冲出赛�?*/
    if (s_ra_state == RA_ST_NONE)            /* RA未激�?*/
    {
        /* 检测到方向预判或远场预减速信号，且无正式flag �?锁定预减�?*/
        if (g_post_edge_side != EDGE_BOTH &&
            g_ra_flag == 0u &&
            (g_ra_pre_flag == 0u ||
             (g_ra_pre_dir != 1u && g_ra_pre_dir != 2u)))
        {
            s_pre_lock = 0u;
            s_pre_timeout = 0u;
            g_ra_pre_flag = 0u;
            g_ra_pre_dir = 0u;
            g_ra_pre_slow_flag = 0u;
        }

        if ((g_ra_pre_flag || g_ra_pre_slow_flag) && g_ra_flag == 0u)
        {
            s_pre_lock = 1u;                 /* 锁定预减�?*/
            s_pre_timeout = 0u;              /* 超时清零 */
        }

        /* 正式flag到来 �?解除预减速（让RA接管�?*/
        if (g_ra_flag != 0u)                 /* 有正式RA flag */
        {
            s_pre_lock = 0u;                 /* 解除锁定 */
            g_ra_pre_slow_flag = 0u;         /* 正式RA接管后清除预减速专用标�?*/
        }

        /* 对称组件（三极管干扰区） �?解除预减�?*/
        if (g_sym_component_flag)            /* 检测到对称组件 */
        {
            s_pre_lock = 0u;                 /* 解除锁定 */
            s_pre_timeout = 0u;              /* 超时清零 */
            g_ra_pre_slow_flag = 0u;         /* 防止元器件远场特征锁住预减�?*/
        }

        /* 锁定状态下，预减速信号消失后超时解除 */
        if (s_pre_lock)                      /* 处于锁定状�?*/
        {
            if (!g_ra_pre_flag && !g_ra_pre_slow_flag)
            {
                /* 恢复直道 �?解除 */
                if (straight_speed_candidate(pos_err_abs)) /* 满足直道条件 */
                {
                    s_pre_lock = 0u;         /* 解除锁定 */
                    s_pre_timeout = 0u;      /* 超时清零 */
                }
                else                         /* 不满足直道条�?*/
                {
                    /* 超时3帧后解除 */
                    s_pre_timeout++;         /* 超时计数�? */
                    if (s_pre_timeout > 3u)  /* 超过3�?*/
                    {
                        s_pre_lock = 0u;     /* 解除锁定 */
                        s_pre_timeout = 0u;  /* 超时清零 */
                    }
                }
            }

            /* 仍在锁定 �?降�?*/
            if (s_pre_lock)                  /* 仍在锁定 */
            {
                pre_slow_active = 1u;        /* 标记预减速激�?*/
                target_base_speed = (int16)((int32)target_base_speed * RA_PRE_SLOW_PCT / 100); /* 降�?*/
            }
        }

        speed_dbg_pre_lock = s_pre_lock;     /* 记录预减速锁状�?*/
    }

    /* 规划基础速度（含各种降�?加速策略） */
    base_speed = plan_base_speed(target_base_speed, pos_err_abs, pre_slow_active); /* 速度规划 */

    /* 单边巡线倒计�?*/
    single_edge_tick();                      /* 更新单边巡线状�?*/
    /* 执行正常PID控制（转�?速度�?*/
    normal_pid_step(pos_err, pos_err_abs);   /* 执行正常PID控制 */
    auto_tune_log_pid_tick();
}
