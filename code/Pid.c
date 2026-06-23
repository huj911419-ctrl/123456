#include "Pid.h"                /* PID控制器头文件，包��有PID相关常量和变量声�?*/
#include "Menu.h"               /* 菜单系统头文件，提供菜单变量（�motor_speed等） */
#include "Track_funsion.h"      /* 视�融合头文件，提供g_tf视�状态结构体 */
#include "IMU.h"                /* IMU头文件，提供yaw_angle、yaw_rate等IMU数据 */
#include "Battery.h"
#include "AutoTuneLog.h"

/* ======================== 全局调试变量 ======================== */

/* 基�速度，由 plan_base_speed() 规划后写入，单位为菜单�定的�度档位�?*/
int16 base_speed = 0;           /* 基�速度值，PID控制的核心�度输入 */
/* 速度�调试输出值（��向叠加后的左右轮�速中间量�?*/
int16 speed_dbg_out = 0;        /* 速度调试输出，供TFT显示�?*/
/* �向调试输出�（��向分量） */
int16 steer_dbg_out = 0;        /* �向调试输出，供TFT显示�?*/
int16 duty_dbg_left = 0;        /* Last left duty sent to motor driver */
int16 duty_dbg_right = 0;       /* Last right duty sent to motor driver */
/* 视�质量降速百分比�?00=无降速），用于TFT调试显示 */
uint8 speed_dbg_vq_pct = 100u;  /* 视�质量降速百分比�?00表示无降�?*/
/* 预�测锁定标志，用于TFT调试显示�?=正在预减速） */
uint8 speed_dbg_pre_lock = 0u;  /* 预减速锁定标志，1表示正在预减�?*/
/* 规划前的原�目标�度，用于TFT调试显示 */
int16 speed_dbg_raw = 0;        /* 原�目标�度，�度规划前的输入�?*/
/* 规划后的�标�度，用于TFT调试显示 */
int16 speed_dbg_plan = 0;       /* 规划后目标�度，�度规划输出�?*/
/* 速度规划原因编号，用于TFT调试显示�?=RA 2=丢线 3=视��?4=对称加�?5=前瞻 6=直道加�?7=质量 8=正常 9=单边加�） */
uint8 speed_dbg_reason = 0u;    /* 速度变化原因编号，用于调�?*/
/* 后转�单边巡线方向，�?start_single_edge() 写入，Track_funsion.c 读取
 * EDGE_BOTH=双边, EDGE_LEFT=�用左边�? EDGE_RIGHT=�用右边�?*/
uint8 g_post_edge_side = EDGE_BOTH; /* 单边巡线方向，全�变量供��模块��?*/

/* ======================== RA（直�?�口）调试变�?======================== */

/* RA状�机当前状�（0=空闲 1=活跃�?*/
uint8 ra_dbg_state = 0u;        /* RA状�调试变�?*/
/* RA当前阶��?=WAIT 1=SLOW 2=APPROACH 3=HARD 4=YAW_LOCK 5=RECOVER�?*/
uint8 ra_dbg_phase = 0u;        /* RA阶�调试变� */
/* RA��方向�?=直� 1=右转 2=左转�?*/
uint8 ra_dbg_dir = 0u;          /* RA方向调试变量 */
/* RA记录的最大拐点��?*/
uint8 ra_dbg_ip_row = 0u;       /* RA拐点行调试变�?*/
/* RA全局计时�（帧数），每PID周期+1 */
uint16 ra_dbg_timer = 0u;       /* RA全局计时器调试变�?*/
/* RA HARD阶�帧计�?*/
uint16 ra_dbg_hard_cnt = 0u;    /* RA���计数调试变�?*/
/* RA�出条件满足的连续帧数（camera_done判断�� */
uint8 ra_dbg_exit_good_cnt = 0u;/* RA�出�数调试变�?*/
/* RA yaw进度 x10，用于TFT显示（避免浮点） */
int16 ra_dbg_yaw10 = 0;         /* RA偏航进度调试变量（乘10避免�点�?*/
uint8 ra_dbg_slow_row = 0u;
uint8 ra_dbg_turn_row = 0u;
uint8 ra_dbg_exit_reason = 0u;
int16 ra_dbg_hard_target10 = 0;
int16 ra_dbg_outer_cmd = 0;
uint8 ra_dbg_line_takeover_ready = 0u;
uint8 ra_dbg_line_takeover_cnt = 0u;
uint8 ra_dbg_exit_boost_active = 0u;
uint8 ra_dbg_exit_boost_cnt = 0u;
int16 ra_dbg_hard_outer_dynamic = 0;
int16 ra_dbg_yaw_pred10 = 0;
int16 ra_dbg_yaw_remain10 = 0;
uint8 ra_dbg_outer_scale = 100u;
uint8 ra_dbg_takeover_valid_rows = 0u;
int16 ra_dbg_takeover_error = 0;
int16 ra_dbg_takeover_lookahead = 0;
int16 ra_dbg_takeover_trend = 0;

/* === New debug variables for improved direct turn === */
int16 ra_dbg_yaw_total_progress10 = 0;
int16 ra_dbg_yaw_hard_progress10 = 0;
uint8 ra_dbg_visual_exit_ready = 0u;
uint8 ra_dbg_visual_stable_cnt = 0u;
uint8 ra_dbg_yaw_guard_active = 0u;
uint8 ra_dbg_over_turn_guard = 0u;
uint8 ra_dbg_line_takeover_speed_cap = 0u;
uint8 ra_dbg_turn_assist_active = 0u;
uint8 ra_dbg_turn_assist_weight = 0u;
int16 ra_dbg_turn_assist_found_col = -1;
uint8 ra_dbg_inner_speed_pct = 100u;
uint8 ra_dbg_outer_boost_pct = 0u;
uint8 ra_dbg_continuous_turn_mode = 0u;
uint8 ra_dbg_exit_reason_verbose = 0u;
uint8 ra_dbg_inner_min_pct = 100u;


/* ======================== �向PD控制静�变�?======================== */

/* 上一次的位置��（滤波后），用于D项��?*/
static float s_steer_last_pos_err = 0.0f;   /* 上一周期滤波后��?*/
/* 上一次的原�位���（未滤波），用于D项��?*/
static float s_steer_last_raw_err = 0.0f;   /* 上一周期原��� */
/* 低�滤波后的位���，一阶IIR滤波器输�?*/
static float s_filtered_err = 0.0f;          /* 滤波后���?*/
/* 上一次的�向输出�，用于变化率限制（slew rate limiter�?*/
static float s_prev_steer_output = 0.0f;     /* 上一周期�向输� */
/* 前�信号的低�滤波�，避免前瞻突变导致�向抖� */
static float s_steer_ff_filtered = 0.0f;     /* 滤波后的前�信� */
static float s_ra_pre_turn_ff = 0.0f;
/* D项重�标志，�?后下��周期跳过D输出（防止切换时�分冲击�?*/
static uint8 s_steer_d_reset_flag = 0u;      /* �分重�标志 */

/* ======================== 速度PI控制静�变�?======================== */

/* 速度PI控制器的�分累��?*/
static float s_speed_integral = 0.0f;        /* 速度�分累��?*/
/* 上一周期的目标�度，用于加速度前��算 */
static float s_prev_target_speed = 0.0f;     /* 上一周期�标�度 */
/* 速度前�就�标志，�周期跳过前馈（避免�动突变） */
static uint8 s_speed_ff_ready = 0u;          /* 前�就�标志 */
/* 电机运�帧计数�，用�?motor_run_time 超时停机 */
static uint32 s_motor_run_counter = 0;       /* 电机运�帧计�?*/
static uint8 s_vacuum_on = 0u;
static uint32 s_vacuum_duty = 0u;
static uint16 s_vacuum_prearm_ticks = 0u;
static uint8 s_vacuum_prearm_timeout = 0u;
volatile uint8 vacuum_enable = 0u;           /* 负压实际��状�，供TFT显示/关屏逻辑使用 */

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

/* ======================== 速度规划静�变�?======================== */

/* 经过斜坡处理后的基�速度，防止�度突变 */
static int16 s_base_speed_ramped = 0;        /* 斜坡处理后的速度 */
/* 直道条件满足的连�帧�数，用于直道加速确�?*/
static uint8 s_straight_cnt = 0u;            /* 直道�认帧计�?*/
/* 直道加�激活标志（1=当前处于直道加�模式） */
static uint8 s_straight_active = 0u;         /* 直道加�激活标�?*/
/* 上一帧的��值，用于��跳变�测（视�质量降速） */
static int16 s_prev_quality_err = 0;         /* 上一帧���?*/
/* 上一帧��值有效标�?*/
static uint8 s_prev_quality_err_valid = 0u;  /* 上一帧��有效标�?*/
static uint8 s_straight_hold = 0u;
static uint8 s_pre_lock = 0u;
static uint8 s_pre_timeout = 0u;
static int16 speed_ramp_apply_reason(int16 target, uint8 reason);

/* ======================== ��屏蔽（turn shield）静态变�?======================== */

/* ��屏蔽剩余帧数�?0时屏蔽RA�测，防�出�后�触发 */

/* ======================== 单边巡线静�变�?======================== */

/* 单边巡线�活标�?*/
static uint8 s_edge_active = 0u;             /* 单边巡线�活标�?*/
/* 当前单边巡线方向（EDGE_LEFT / EDGE_RIGHT�?*/
static uint8 s_edge_side = EDGE_BOTH;        /* 单边巡线方向 */
/* 单边巡线已运行帧计数 */
static uint16 s_edge_cnt = 0u;               /* 单边巡线当前帧��?*/
static uint16 s_edge_age_cnt = 0u;
/* 单边巡线�标帧数（由时间ms���?*/
static uint16 s_edge_target = 0u;            /* 单边巡线�标帧� */
/* 单边巡线�否保持到下一次真正转弯RA�� */
static uint8 s_edge_until_next_turn = 0u;    /* 1=不按时间结束，等下一次非直�RA完成 */
/* 保持模式已遇到下�次真正转�，等�RA结束后关�单�?*/
static uint8 s_edge_release_after_turn = 0u; /* 1=当前非直行RA结束后恢复双�?*/
static uint8 s_single_edge_fast_hold = 0u;
static uint8 s_completed_right_ra_count = 0u;

/* ======================== 丢线搜索静�变�?======================== */

/* 丢线搜索�活标志（1=正在执�原地转向搜��?*/
static uint8 s_lost_search_active = 0u;      /* 丢线搜索�活标�?*/
/* 连续丢线帧�数，达到阈值后�动搜� */
static uint8 s_lost_line_cnt = 0u;           /* 连续丢线帧��?*/
/* 丢线搜索已运行帧计数 */
static uint16 s_lost_search_cnt = 0u;        /* 丢线搜索运�帧计�?*/
/* 丢线搜索方向�?=向右�?2=向左�� */
static uint8 s_lost_search_dir = 1u;         /* 丢线搜索方向 */
/* 丢线前的�后��值，用于选择搜索方向 */
static int16 s_lost_last_err = 0;            /* 丢线前最后��?*/

/* ======================== 串级PID（cascade）内�状�?======================== */

/* Cascade外环输出的目标��度，经低�滤波后送给内环 */
static float s_cas_target_filtered = 0.0f;   /* 串级PID�标��度（滤波后�?*/
/* 内环上一帧的角�度��，用于内环D�?*/
static float s_cas_last_yaw_err    = 0.0f;   /* 内环上一帧��度�� */
static float s_yaw_bias            = 0.0f;   /* IMU零漂动态补偿 */
static float s_cas_pos_integral   = 0.0f;   /* 串级外环位置误差积分 */

/* Cascade菜单变量（在Menu.c��调） */
/* 串级PID使能标志�?=�通PD 1=串级IMU角�度�� */
int16 cascade_en    = 1;                     /* 串级PID使能标志 */
/* 内环角�度D增益（菜单可调） */
int16 yaw_kd        = 3;                     /* 内环角�度�分�益 */
/* �立yaw阻尼增益（cascade_en=0时的角�度阻尼�?=关闭�?*/
int16 yaw_damp_gain = 0;                     /* �立偏�阻尼增益 */

/* ======================== RA状�机枚举 ======================== */

/* RA状�：无活�?/ 有活�?*/
typedef enum { RA_ST_NONE, RA_ST_ACTIVE } RaState;  /* RA状�枚�?*/

#define RA_EXIT_NONE      0u
#define RA_EXIT_LINE      1u
#define RA_EXIT_YAW       2u
#define RA_EXIT_COAST     3u
#define RA_EXIT_TIMEOUT   4u
#define RA_EXIT_EMERGENCY 5u
#define RA_EXIT_NO_IMU    6u
#define RA_EXIT_RA_TO     7u
#define RA_EXIT_RECOVER   8u
/* RA阶�：等待拐点接�?�?减�?�?接近 �?���?�?恢� */
typedef enum { RA_PH_WAIT, RA_PH_SLOW, RA_PH_APPROACH, RA_PH_HARD, RA_PH_YAW_LOCK, RA_PH_RECOVER } RaPhase; /* RA阶�枚� */

/* RA状�机当前状�?*/
static RaState s_ra_state = RA_ST_NONE;      /* RA当前状�，初�为空�?*/
/* RA状�机当前阶� */
static RaPhase s_ra_phase = RA_PH_WAIT;      /* RA当前阶�，初�为等待 */
/* RA��方向�?=直��过 1=右转 2=左转�?*/
static uint8 s_ra_dir = 0u;                  /* RA��方向 */
/* 触发RA的原始flag值（1~5），用于查路线表和超时配�?*/
static uint8 s_ra_orig_flag = 0u;            /* 原�触发flag */
static int16 s_ra_speed_ref_latched = 0;
/* RA记录的拐点最大�号，用于阶段转换判� */
static uint8 s_ra_ip_row = 0u;               /* 拐点�大��?*/
/* 直��过标志�?=该路口直行不���?*/
static uint8 s_ra_straight = 0u;             /* 直��过标志 */
/* RA结束后启用的单边巡线方向 */
static uint8 s_ra_post_edge_side = EDGE_BOTH;/* 结束后单边巡线方�?*/
/* RA结束后单边巡线持�时�?ms) */
static uint16 s_ra_post_edge_ms = 0u;        /* 结束后单边巡线时�?*/
/* HARD阶�中满足�出条件的连续帧��?*/
static uint8 s_ra_exit_good_cnt = 0u;        /* HARD�出条件满足帧计数 */
/* RECOVER阶�中满足恢�完成条件的连续帧��?*/
static uint8 s_ra_recover_good_cnt = 0u;     /* RECOVER完成条件满足帧��?*/
static uint8 s_ra_recover_seen_cnt = 0u;
/* APPROACH阶�帧计�?*/
static uint16 s_ra_approach_cnt = 0u;        /* APPROACH阶�帧计�?*/
/* RA全局计时�，每PID周期+1，用于超时保�?*/
static uint16 s_ra_timer = 0u;               /* RA全局计时�?*/
/* HARD阶�帧计�?*/
static uint16 s_ra_hard_cnt = 0u;            /* HARD阶�帧计�?*/
/* RECOVER阶�帧计�?*/
static uint16 s_ra_recover_cnt = 0u;         /* RECOVER阶�帧计�?*/
static uint16 s_ra_yaw_lock_cnt = 0u;       /* YAW_LOCK frame counter */
/* 当前阶�内的帧计数（WAIT/SLOW�� */
static uint16 s_ra_phase_cnt = 0u;           /* 当前阶�帧计�?*/
static float s_ra_yaw_base = 0.0f;
static float s_ra_hard_yaw_target = 0.0f;
static float s_ra_hard_yaw_peak = 0.0f;
static uint8 s_ra_post_recover_cnt = 0u;
static uint8 s_ra_post_recover_complex = 0u;
static uint8 s_ra_complex_lost_match_cnt = 0u;
static uint8 s_ra_pre_direct_match_cnt = 0u;
static uint8 s_ra_pending_complex_flag = 0u;
static uint8 s_ra_pending_complex_cnt = 0u;
static uint8 s_ra_pending_complex_ip_row = 0u;
static uint8 s_ra_pending_complex_conf = 0u;
static uint8 s_ra_pending_complex_bridge = 0u;
static uint8 s_ra_lost_guard_cnt = 0u;           /* HARD入口yaw基准 */
/* HARD阶�的速度种子值，用于RECOVER阶�的速度平滑过渡 */
static float s_ra_hard_speed_seed = 0.0f;    /* HARD阶��度种子 */
/* HARD阶�的�向�子值，用于RECOVER阶�的�向平滑过�?*/
static float s_ra_hard_steer_seed = 0.0f;    /* HARD阶�转向�子 */
static float s_ra_hard_diff_cmd = 0.0f;
static uint8 s_ra_hard_diff_ready = 0u;
static float s_ra_exit_last_err = 0.0f;
static float s_ra_exit_last_turn = 0.0f;
static uint8 s_ra_exit_pd_ready = 0u;
static uint8 s_ra_line_takeover_cnt = 0u;
static uint8 s_ra_exit_boost_cnt = 0u;
static uint8 s_ra_exit_boost_active = 0u;

/* === New static variables for improved direct turn === */
static float s_ra_total_yaw_base = 0.0f;     /* total yaw base, set in ra_start, never reset */

/* Turn assist (补线) state */
static uint8 s_turn_assist_active = 0u;
static uint8 s_turn_assist_weight = 0u;
static int16 s_turn_assist_found_col = -1;
static uint8 s_turn_assist_frame_cnt = 0u;

/* Visual exit stable count */
static uint8 s_visual_stable_cnt = 0u;
static uint8 s_visual_strict_cnt = 0u;

/* Yaw / over-turn guards */
static uint8 s_yaw_guard_active = 0u;
static uint8 s_over_turn_guard = 0u;

/* Line takeover speed cap */
static uint8 s_line_takeover_speed_cap_frames = 0u;

/* Consecutive turn mode */
static uint8 s_continuous_turn_mode = 0u;
static uint8 s_continuous_turn_remnant_frames = 0u;
static uint8 s_continuous_turn_post_dir = 0u;

/* 前向声明：int16取绝对�?*/
static int16 abs_i16(int16 v)
{
    if (v == (int16)(-32767 - 1))
        return 32767;
    return (v < 0) ? (int16)(-v) : v;
}

/* ======================== �线�则定义 ======================== */

/* �口动作类� */
#define ACT_STRAIGHT 0u  /* 直��过 */
#define ACT_RIGHT    1u  /* 右转 */
#define ACT_LEFT     2u  /* 左转 */
#define ACT_AUTO     3u  /* ��（根据flag推断方向，用于直角） */

/* �口�则结构�?*/
typedef struct
{
    uint8 count;            /* �几�出现�类型flag时匹配��则 */
    uint8 flag;             /* �口类型flag�?=右直�?2=左直�?3/4/5=�通路口） */
    uint8 action;           /* 执�动作（ACT_STRAIGHT/RIGHT/LEFT/AUTO�?*/
    uint8 post_edge_side;   /* ��后单边巡线方向（EDGE_BOTH=不启�� */
    uint16 post_edge_ms;    /* ��后单边巡线持�时�?ms) */
} IntersectionRule;         /* �口�则结构体定�?*/

/* 规则构�宏 */
#define RULE(count, flag, action) \
    { (count), (flag), (action), EDGE_BOTH, 0u }           /* �通�则，无单�?*/
#define RULE_EDGE(count, flag, action, edge_side, edge_ms) \
    { (count), (flag), (action), (edge_side), (edge_ms) }   /* 指定单边方向 */
#define RULE_AUTO(count, flag, action, edge_ms) \
    { (count), (flag), (action), EDGE_AUTO, (edge_ms) }     /* �动�单边方�?*/
#define RULE_RA(count, flag) \
    { (count), (flag), ACT_AUTO, EDGE_BOTH, 0u }            /* 直�自�，无单边 */
#define RULE_RA_AUTO(count, flag, edge_ms) \
    { (count), (flag), ACT_AUTO, EDGE_AUTO, (edge_ms) }     /* 直�自�+�动单� */
#define RULE_RA_EDGE(count, flag, edge_side, edge_ms) \
    { (count), (flag), ACT_AUTO, (edge_side), (edge_ms) }   /* 直�自�+指定单边 */

/* �线表：按图中黑线走向�写�? * RULE：执行指定动作，不开�单边巡线�? * RULE_AUTO：执行指定动作，��结束后自动�单边�? * RULE_EDGE：执行指定动作，结束后强制指定单边�? * RULE_RA：直角方向自�，不��单边�? * RULE_RA_AUTO：直角方向自�，转完后�动�单边�? * RULE_RA_EDGE：直角方向自�，转完后强制指定单边�? * 直�类型�?=右直角，2=左直角�普通路口类型：3/4/5�?*/
static const IntersectionRule user_rules[] = {
    /* 如果某个直�出�后需要单边，就在这里插入�?     * RULE_RA_AUTO(�几�? 1u�?u, 持续时间),
     * 例�：RULE_RA_AUTO(2u, 1u, 500u), */

    /* 当前���线：
     * 右直�?-> 右直�?-> 左直�?-> 4�?-> 5�?-> 5�?-> 4�?-> 4�?     * -> 5�?-> 3�?-> 3直� -> 5�?-> 右直�?-> 右直角后单边�?*/
    RULE_RA(  1u, 1u),    /* first flag=1: auto direct turn */
    RULE(     1u, 4u, ACT_RIGHT), /* first flag=4: right turn; no early single-edge */
    RULE_RA(  1u, 2u),    /* first flag=2: auto direct turn */
    RULE(     1u, 5u, ACT_LEFT),    /* first flag=5: left turn */
    RULE(     2u, 5u, ACT_RIGHT),   /* �?个flag=5（十字）：右�?*/
    RULE(     2u, 4u, ACT_RIGHT),   /* �?个flag=4（T右）：右�?*/
    RULE(     3u, 4u, ACT_RIGHT),   /* �?个flag=4（T右）：右�?*/
    RULE(     3u, 5u, ACT_LEFT),    /* �?个flag=5（十字）：左�?*/
    RULE(     4u, 5u, ACT_LEFT),    /* �?个flag=5（十字）：左�?*/
    RULE(     1u, 3u, ACT_LEFT),    /* �?个flag=3（T左）：左�?*/
    RULE(     2u, 3u, ACT_STRAIGHT),/* �?个flag=3（T左）：直�?*/
    RULE_EDGE(5u, 5u, ACT_RIGHT, EDGE_LEFT, SINGLE_EDGE_UNTIL_NEXT_TURN),   /* �?个flag=5（十字）：右�?*/
    RULE_RA_EDGE(2u, 1u, EDGE_LEFT, SINGLE_EDGE_UNTIL_NEXT_TURN), /* �?个flag=1（右直�）：结束后靠左单�?*/
    RULE_RA_EDGE(3u, 1u, EDGE_LEFT, SINGLE_EDGE_UNTIL_NEXT_TURN), /* �?个flag=1（右直�）：结束后靠左单边直到下个真转� */
    RULE_RA(  4u, 1u),    /* �?个flag=1（右直�）：自动方� */
};
/* �线表总条�� */
#define USER_RULE_COUNT (sizeof(user_rules) / sizeof(user_rules[0])) /* �线表条目� */

/* ======================== �线匹配状�?======================== */

/* 各类型flag�?~6）的已匹配�数，用于按序匹配路线�?*/
static uint8 s_inter_count[7] = {0u};  /* 各flag类型已匹配��?*/
/* �线全部完成标志，�1后延迟停�?*/
static uint8 s_rules_done = 0u;        /* �线完成标� */
/* �线完成后的延迟停机�时�（帧数�?*/
static uint16 s_rules_done_timer = 0u; /* �线完成延迟停机�时�?*/
/* �线调试：当前执�到�几�（�?�始） */
uint8 route_dbg_step = 0u;             /* �线调试当前�数 */
/* �线调试：�线表总��?*/
uint8 route_dbg_total = (uint8)USER_RULE_COUNT; /* �线调试���?*/
/* �线调试：当前匹配的flag类型 */
uint8 route_dbg_flag = 0u;             /* �线调试当前flag */
/* �线调试：当前匹配的count�?*/
uint8 route_dbg_count = 0u;            /* �线调试当前count */
/* �线调试：当前执�的动作 */
uint8 route_dbg_action = ACT_STRAIGHT; /* �线调试当前动� */
/* 待提交的�线匹配结果有效标志（延迟�帧提交，避免影响当前�RA�?*/
static uint8 s_route_pending_valid = 0u;   /* 待提交有效标�?*/
/* 待提交的flag类型 */
static uint8 s_route_pending_flag = 0u;    /* 待提�flag */
/* 待提交的count�?*/
static uint8 s_route_pending_count = 0u;   /* 待提�count */
/* 待提交的动作 */
static uint8 s_route_pending_action = ACT_STRAIGHT; /* 待提交动�?*/

/* 前向声明：更新RA调试信息 */
static void ra_debug_update(void);         /* 前向声明：更新RA调试变量 */
static void visual_reset_stable(void);     /* 前向声明：复位视觉稳定计数 */
static void turn_assist_reset(void);       /* 前向声明：复位补线模式 */

/* ======================== RA状�机返回结构�?======================== */

/* RA状�机每帧返回的结�?*/
typedef struct
{
    uint8 need_pid;     /* 1=�要继�执�PID控制�?=跳过PID */
    uint8 should_return;/* 1=RA已直接输出电机，��PID不执�?*/
    float speed_scale;  /* 速度缩放因子�?.0~1.0），用于降�?*/
} RaResult;             /* RA状�机返回结构�?*/

/* ======================== �线决策结构�?======================== */

/* �口决策结� */
typedef struct
{
    uint8 action;           /* 动作类型（ACT_STRAIGHT/RIGHT/LEFT�?*/
    uint8 post_edge_side;   /* ��后单边方�?*/
    uint16 post_edge_ms;    /* ��后单边持�时�?*/
    uint8 valid;            /* 决策有效标志�?=�匹配�线表�?*/
} RouteDecision;            /* �口决策结构�?*/

/* ======================== �向调度结构�?======================== */

/* 根据速度和曲率�算的转向PD参数缩放 */
typedef struct
{
    float kp_scale;   /* 比例增益缩放因子 */
    float kd_scale;   /* �分�益缩放因子 */
    float ff_scale;   /* 前��益缩放因子 */
    float slew_max;   /* 输出变化率限�?*/
} SteerSchedule;      /* �向�益调度结构�?*/

/* ======================== 工具函数 ======================== */

/* abs_f - �点数取绝对�? * @v: 输入�点�? * 返回: |v| */
static float abs_f(float v)                 /* �点绝对�函�?*/
{
    return (v >= 0.0f) ? v : -v;            /* 非负返回原�，负数取反 */
}

/* clamp_f - �点数限�? * @v: 输入�? * @min_v: 下限
 * @max_v: 上限
 * 返回: 限幅后的�?*/
static float clamp_f(float v, float min_v, float max_v) /* �点限幅函� */
{
    if (v < min_v) return min_v;            /* 低于下限，返回下�?*/
    if (v > max_v) return max_v;            /* 高于上限，返回上�?*/
    return v;                               /* 在范围内，返回原�?*/
}

/* lerp_f - 线�插�? * @a: 起��（t=0时返回）
 * @b: 终��（t=1时返回）
 * @t: 插�系数（0.0~1.0�? * 返回: a + (b-a)*t */
static float lerp_f(float a, float b, float t) /* 线�插值函�?*/
{
    return a + (b - a) * t;                 /* 标准线�插值公�?*/
}

static float ra_voltage_comp_scale(void)
{
#if RA_VOLT_COMP_ENABLE
    uint16 volt_x10 = battery_get_voltage_x10();
    float scale;

    if (volt_x10 < 90u || volt_x10 > 160u)
        return 1.0f;

    scale = (float)RA_VOLT_REF_X10 / (float)volt_x10;
    return clamp_f(scale,
                   (float)RA_VOLT_COMP_MIN_PCT * 0.01f,
                   (float)RA_VOLT_COMP_MAX_PCT * 0.01f);
#else
    return 1.0f;
#endif
}

/* range_ratio_i16 - 将int16值映射到[start, end]区间的比例（0.0~1.0�? * @value: 输入�? * @start: 区间起点（返�?.0�? * @end: 区间终点（返�?.0�? * 返回: 0.0~1.0之间的比例�? * 用�：增益调度�的归��?*/
static float range_ratio_i16(int16 value, int16 start, int16 end) /* 区间比例映射 */
{
    if (end <= start)                       /* 区间无效或单�?*/
        return (value >= end) ? 1.0f : 0.0f; /* 超过终点返回1，否�? */

    if (value <= start) return 0.0f;        /* 小于起点，返�? */
    if (value >= end) return 1.0f;          /* 大于终点，返�? */

    return (float)(value - start) / (float)(end - start); /* 线�比例��?*/
}

/* ======================== �向�益调度 ======================== */

/* steer_schedule_calc - 根据当前速度和弯道程度�算�向PD的�益缩放参�? * @pos_err_abs: 位置��绝��? * 返回: 包含kp_scale/kd_scale/ff_scale/slew_max的调度结�? *
 * 调度逻辑�? *   1. 取��?前瞻/趋势��大的作为�道信�
 *   2. 速度越高，kp/kd适当降低（防高�振荡）
 *   3. �道越急，kp/kd适当增大（加强响应）
 *   4. 前�在高�时才启�? *   5. 变化率限制随�道程度略� */
static SteerSchedule steer_schedule_calc(int16 pos_err_abs) /* �向�益调度计算 */
{
    SteerSchedule s;                        /* 调度结果结构�?*/
    int16 la_abs = abs_i16(g_tf.lookahead_error);  /* 前瞻��绝��?*/
    int16 trend_abs = abs_i16(g_tf.error_trend);    /* 趋势��绝��?*/
    int16 curve_signal = pos_err_abs;       /* 初�弯道信� = 位置�� */

    /* 取三者最大�作为弯道信�?*/
    if (la_abs > curve_signal) curve_signal = la_abs;   /* 前瞻更大则替�?*/
    if (trend_abs > curve_signal) curve_signal = trend_abs; /* 趋势更大则替�?*/

    /* 速度因子：低速→0，高速→1 */
    float speed_t = range_ratio_i16((int16)base_speed,  /* 当前基�速度 */
                                    STEER_GAIN_SPEED_START, /* 速度下限180 */
                                    STEER_GAIN_SPEED_END);  /* 速度上限800 */
    /* �道因子：直道�0，�弯�? */
    float curve_t = range_ratio_i16(curve_signal,       /* �道信� */
                                    STEER_GAIN_CURVE_T1, /* �道下�10 */
                                    STEER_GAIN_CURVE_T2);/* �道上�38 */
    /* 高�时的kp/kd缩放（�常kp略降，kd略��?*/
    float kp_fast = lerp_f(1.0f, (float)STEER_FAST_KP_PCT * 0.01f, speed_t); /* 高�kp缩放 */
    float kd_fast = lerp_f(1.0f, (float)STEER_FAST_KD_PCT * 0.01f, speed_t); /* 高�kd缩放 */

    /* �终kp = 高�kp线�过渡到�道kp */
    s.kp_scale = lerp_f(kp_fast,                        /* 高�kp基准 */
                        (float)STEER_CURVE_KP_PCT * 0.01f, /* �道kp */
                        curve_t);                        /* �道因子插�?*/
    /* �终kd = 高�kd线�过渡到�道kd */
    s.kd_scale = lerp_f(kd_fast,                        /* 高�kd基准 */
                        (float)STEER_CURVE_KD_PCT * 0.01f, /* �道kd */
                        curve_t);                        /* �道因子插�?*/
    /* 前�缩放：高�时才启�，乘以菜单可调系� */
    s.ff_scale = range_ratio_i16((int16)base_speed,     /* 当前基�速度 */
                                 STEER_FF_SPEED_START,   /* 前�启用�度下限 */
                                 STEER_FF_SPEED_END) *   /* 前�启用�度上限 */
                 ((float)steer_ff_k * 0.01f);            /* 菜单�调前馈系� */
    /* 变化率限制：�道时略�（允�更�响应�?*/
    s.slew_max = STEER_SLEW_MAX * PID_DT_SCALE * lerp_f(0.85f, 1.20f, curve_t); /* �道时变化率略� */

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
    int16 max_row = (int16)((uint16)(TF_IMG_H - 1u) * 2u);

    if (row < 0) row = 0;
    if (row > max_row) row = max_row;
    return (uint8)row;
}

static uint8 ra_turn_speed_advance(void)
{
    int16 ref = ra_speed_ref();
    uint16 base_adv = RA_FAST_TURN_ROW_ADVANCE;
    uint16 max_adv = RA_FAST_TURN_ROW_ADVANCE_MAX;
    uint16 adv;

    if (max_adv < base_adv)
        max_adv = base_adv;
    if (ref <= RA_FAST_SPEED_START)
        return 0u;
    if (RA_FAST_TURN_ROW_ADVANCE_SPEED_END <= RA_FAST_SPEED_START ||
        ref >= RA_FAST_TURN_ROW_ADVANCE_SPEED_END)
        return (uint8)max_adv;

    adv = (uint16)(base_adv +
          ((uint32)(max_adv - base_adv) *
           (uint32)(ref - RA_FAST_SPEED_START)) /
          (uint32)(RA_FAST_TURN_ROW_ADVANCE_SPEED_END - RA_FAST_SPEED_START));
    return (uint8)adv;
}

static uint8 ra_apply_turn_speed_advance(uint8 turn_row)
{
    uint8 advance = ra_turn_speed_advance();
    uint8 row;

    row = (turn_row > advance) ? (uint8)(turn_row - advance) : 0u;
    if (row < RA_TURN_ROW_MIN)
        row = RA_TURN_ROW_MIN;
    return row;
}

static uint8 ra_direct_fixed_turn_trigger_row(void)
{
    return ra_apply_turn_speed_advance(RA_FIXED_HARD_ROW);
}

static uint8 ra_turn_trigger_row(void)
{
    uint8 turn_row = ra_turn_row_for_speed();

#if RA_FIXED_HARD_ROW_ENABLE
    if (s_ra_state == RA_ST_ACTIVE && s_ra_straight == 0u)
    {
        if (s_ra_orig_flag >= 3u)
        {
            return RA_FIXED_COMPLEX_HARD_ROW;
        }
        return ra_direct_fixed_turn_trigger_row();
    }
#endif

    if (s_ra_orig_flag >= 3u &&
        turn_row <= (uint8)(255u - RA_COMPLEX_TURN_ROW_OFFSET))
    {
        turn_row = (uint8)(turn_row + RA_COMPLEX_TURN_ROW_OFFSET);

        if (ra_speed_ref() <= RA_LOW_SPEED_START &&
            turn_row <= (uint8)(255u - RA_LOW_TURN_ROW_DELAY))
        {
            turn_row = (uint8)(turn_row + RA_LOW_TURN_ROW_DELAY);
        }
    }
    else if (s_ra_orig_flag < 3u &&
             turn_row <= (uint8)(255u - RA_DIRECT_TURN_ROW_OFFSET))
    {
        turn_row = (uint8)(turn_row + RA_DIRECT_TURN_ROW_OFFSET);
    }

    turn_row = ra_apply_turn_speed_advance(turn_row);

    return turn_row;
}

static uint8 ra_slow_trigger_row(void)
{
    uint8 turn_row = ra_turn_trigger_row();
    uint8 slow_row = (turn_row > RA_SLOW_BEFORE_TURN_ROWS) ?
                     (uint8)(turn_row - RA_SLOW_BEFORE_TURN_ROWS) :
                     0u;
    uint8 menu_slow_row = (ra_slow_row < 0) ? 0u : (uint8)ra_slow_row;

    if (s_ra_orig_flag < 3u &&
        ra_speed_ref() >= RA_FAST_SPEED_START &&
        menu_slow_row > RA_FAST_SLOW_MENU_MAX_ROW)
    {
        menu_slow_row = RA_FAST_SLOW_MENU_MAX_ROW;
    }

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

#if defined(RA_FAST_SPEED_START) && defined(RA_FAST_APPROACH_FRAMES)
    if (s_ra_orig_flag < 3u && ra_speed_ref() >= RA_FAST_SPEED_START)
    {
        if (frames > RA_FAST_APPROACH_FRAMES)
            frames = RA_FAST_APPROACH_FRAMES;
    }
#endif

#if defined(RA_LOW_SPEED_START) && defined(RA_LOW_APPROACH_FRAMES)
    if (ra_speed_ref() <= RA_LOW_SPEED_START)
    {
        if (frames > RA_LOW_APPROACH_FRAMES)
            frames = RA_LOW_APPROACH_FRAMES;
    }
#endif

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
            if (route_next_flag_is((uint8)g_ra_pre_dir))
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

    if (ip_row == 0u)
        ip_row = RA_PRE_ROUTE_IP_ROW;

    speed_ref = base_speed;
    if (speed_ref < 0)
        speed_ref = 0;
    if (speed_ref < RA_FAST_SPEED_START)
    {
        target = 0.0f;
        goto slew_out;
    }

    turn_row = (int16)ra_direct_fixed_turn_trigger_row();
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

static uint8 ra_pre_turn_guard_dir(void)
{
#if RA_PRE_TURN_STEER_GUARD_ENABLE
    if (s_ra_state == RA_ST_NONE)
    {
        if (g_ra_flag == 0u &&
            g_ra_pre_flag != 0u &&
            (g_ra_pre_dir == 1u || g_ra_pre_dir == 2u) &&
            route_next_flag_is((uint8)g_ra_pre_dir))
        {
            return g_ra_pre_dir;
        }
    }
    else if (s_ra_orig_flag < 3u &&
             (s_ra_phase == RA_PH_WAIT ||
              s_ra_phase == RA_PH_SLOW ||
              s_ra_phase == RA_PH_APPROACH) &&
             (s_ra_dir == 1u || s_ra_dir == 2u))
    {
        return s_ra_dir;
    }
#endif

    return 0u;
}

static float ra_pre_turn_steer_guard(float steer)
{
#if RA_PRE_TURN_STEER_GUARD_ENABLE
    uint8 dir = ra_pre_turn_guard_dir();

    if (dir != 0u)
    {
        uint8 ip_row = (s_ra_state == RA_ST_ACTIVE) ? s_ra_ip_row : g_ip_max_row;
        int16 turn_row = (int16)((dir == 1u || dir == 2u) ?
                                 ra_direct_fixed_turn_trigger_row() :
                                 ra_turn_trigger_row());
        float row_t = range_ratio_i16((int16)ip_row,
                                      (int16)RA_PRE_TURN_GUARD_START_ROW,
                                      turn_row);
        float target =
            RA_PRE_TURN_GUARD_STEER_MIN +
            (RA_PRE_TURN_GUARD_STEER_MAX -
             RA_PRE_TURN_GUARD_STEER_MIN) * row_t;

        if (ip_row < RA_PRE_TURN_GUARD_START_ROW ||
            abs_i16(g_tf.error) > RA_PRE_TURN_GUARD_ERR_MAX ||
            abs_i16(g_tf.lookahead_error) > RA_PRE_TURN_GUARD_LA_MAX)
        {
            return steer;
        }

        if (dir == 1u)
            target = -target;

        steer = target +
                steer * ((float)RA_PRE_TURN_NORMAL_STEER_PCT * 0.01f);
        if (dir == 1u)
        {
            if (steer > 0.0f)
                steer = target;
        }
        else if (steer < 0.0f)
        {
            steer = target;
        }
        s_prev_steer_output = steer;
        if (RA_PRE_TURN_NORMAL_STEER_PCT == 0)
        {
            s_steer_ff_filtered = 0.0f;
        }
    }
#endif

    return steer;
}

/* normalize_angle - 角度归一化到[-180, 180]范围
 * @angle: 输入角度（度�? * 返回: 归一化后的��?*/
static float normalize_angle(float angle)   /* 角度归一化函�?*/
{
    while (angle > 180.0f) angle -= 360.0f; /* 大于180则减360 */
    while (angle < -180.0f) angle += 360.0f;/* 小于-180则加360 */
    return angle;                           /* 返回归一化��?*/
}

/* ======================== RA yaw进度计算 ======================== */

/* ra_yaw_progress - 计算RA HARD阶�的yaw�动进度（正�，单位：度�? * 右转时直接用yaw_angle（�=右转），左转时取�? * �返回正�（负�表示还没开始转或方向错�，�为0�? * 返回: yaw�动进度（正�度�?*/
static float ra_yaw_progress(void)          /* 计算RA偏航进度 */
{
    float delta = normalize_angle(yaw_angle - s_ra_yaw_base);

    /* 右转方向：yaw_angle为负值，取反得到正进�?*/
    if (s_ra_dir == 1u)                     /* 右转方向 */
        delta = -delta;                     /* 取反得到正进�?*/

    return (delta > 0.0f) ? delta : 0.0f;  /* 返回正�，负���? */
}

/* ra_total_yaw_progress - 从RA开始到现在的总转角（item 1）
 * 使用 s_ra_total_yaw_base（在ra_start中记录，不被ra_enter_hard覆盖）
 * 返回: 总转角（度，正数）
 */
static float ra_total_yaw_progress(void)
{
    float delta = normalize_angle(yaw_angle - s_ra_total_yaw_base);

    if (s_ra_dir == 1u)
        delta = -delta;

    return (delta > 0.0f) ? delta : 0.0f;
}

/* ra_hard_yaw_progress - HARD阶�的转角（item 1）
 * 使用 s_ra_yaw_base（在ra_enter_hard中记录），与原有ra_yaw_progress一致
 * 返回: HARD阶�转角（度，正数）
 */
static float ra_hard_yaw_progress(void)
{
    return ra_yaw_progress();
}

static float ra_yaw_progress_rate(void)
{
    float rate = (float)yaw_rate;

    if (s_ra_dir == 1u)
        rate = -rate;

    return (rate > 0.0f) ? rate : 0.0f;
}

static float ra_abs_yaw_progress(void)
{
    float yaw_now = normalize_angle(yaw_angle - s_ra_yaw_base);

    if (yaw_now < 0.0f)
        yaw_now = -yaw_now;

    return yaw_now;
}

static float ra_abs_yaw_rate(void)
{
    float rate = (float)yaw_rate;

    if (rate < 0.0f)
        rate = -rate;

    return rate;
}

static int16 ra_fast_hard_outer_cmd(void)
{
#if RA_HARD_PREDICT_EXIT_ENABLE
    float yaw_now;
    float yaw_rate_abs;
    float yaw_pred;
    float target;
    float remain;
    int32 outer;
    int32 scale;

    target = (float)ra_hard_yaw;
    if (target > RA_HARD_TARGET_YAW_MAX)
        target = RA_HARD_TARGET_YAW_MAX;

    yaw_now = ra_abs_yaw_progress();
    yaw_rate_abs = ra_abs_yaw_rate();
    yaw_pred = yaw_now + yaw_rate_abs * RA_HARD_PREDICT_TIME_S;
    remain = target - yaw_pred;

    if (remain > RA_HARD_REMAIN_FULL)
        scale = RA_HARD_OUTER_SCALE_FULL;
    else if (remain > RA_HARD_REMAIN_MID)
        scale = RA_HARD_OUTER_SCALE_MID;
    else if (remain > RA_HARD_REMAIN_LOW)
        scale = RA_HARD_OUTER_SCALE_LOW;
    else
        scale = RA_HARD_OUTER_SCALE_END;

    outer = ((int32)ra_hard_outer * scale) / 100;
    if (outer < RA_HARD_OUTER_MIN)
        outer = RA_HARD_OUTER_MIN;

    ra_dbg_hard_outer_dynamic = (int16)outer;
    ra_dbg_yaw_pred10 = (int16)(yaw_pred * 10.0f);
    ra_dbg_yaw_remain10 = (int16)(remain * 10.0f);
    ra_dbg_outer_scale = (uint8)scale;

    return (int16)outer;
#else
    ra_dbg_hard_outer_dynamic = ra_hard_outer;
    ra_dbg_outer_scale = 100u;
    return ra_hard_outer;
#endif
}

static float ra_hard_target_limit(float target)
{
    if (target > RA_HARD_YAW_MAX_DEG)
        target = RA_HARD_YAW_MAX_DEG;
    return target;
}

static float slew_to_f(float current, float target, float max_delta)
{
    float delta = target - current;

    if (delta > max_delta)
        target = current + max_delta;
    else if (delta < -max_delta)
        target = current - max_delta;

    return target;
}

static float ra_hard_target_rate(uint8 direct_fast,
                                 float hard_yaw_target,
                                 float yaw_progress)
{
    float remain = hard_yaw_target - yaw_progress;
    float max_rate = (float)ra_hard_rate;
    float target_rate;

    if (remain <= RA_HARD_RATE_STOP_REMAIN_DEG)
        return 0.0f;

    if (direct_fast != 0u)
        max_rate += RA_FAST_HARD_RATE_BOOST;
    if (s_ra_orig_flag >= 3u)
        max_rate *= (float)RA_COMPLEX_HARD_RATE_PCT * 0.01f;

    max_rate = clamp_f(max_rate, RA_HARD_RATE_MIN, RA_HARD_RATE_LIMIT);
    target_rate = max_rate;

    if (RA_HARD_RATE_TAPER_REMAIN_DEG > RA_HARD_RATE_STOP_REMAIN_DEG &&
        remain < RA_HARD_RATE_TAPER_REMAIN_DEG)
    {
        float span = RA_HARD_RATE_TAPER_REMAIN_DEG -
                     RA_HARD_RATE_STOP_REMAIN_DEG;
        float t = (remain - RA_HARD_RATE_STOP_REMAIN_DEG) / span;

        t = clamp_f(t, 0.0f, 1.0f);
        target_rate = RA_HARD_RATE_MIN +
                      (max_rate - RA_HARD_RATE_MIN) * t;
    }

    return target_rate;
}

static void ra_hard_apply_rate_control(uint8 direct_fast,
                                       float hard_yaw_target,
                                       float yaw_progress,
                                       float yaw_progress_rate,
                                       float *outer,
                                       float *inner)
{
    float inner_floor = *inner;
    float diff_base = *outer - inner_floor;
    float max_diff;
    float diff_cmd;
    float target_rate;

    if (diff_base < 0.0f)
        diff_base = 0.0f;

    if (!imu_ready || imu_error || hard_yaw_target <= 1.0f)
    {
        s_ra_hard_diff_ready = 0u;
        return;
    }

    target_rate = ra_hard_target_rate(direct_fast,
                                      hard_yaw_target,
                                      yaw_progress);
    diff_cmd = diff_base +
               (target_rate - yaw_progress_rate) * RA_HARD_RATE_KP;

    max_diff = diff_base * (float)RA_HARD_DIFF_MAX_PCT * 0.01f;
    if (max_diff < diff_base)
        max_diff = diff_base;

    diff_cmd = clamp_f(diff_cmd, 0.0f, max_diff);

    if (s_ra_hard_diff_ready == 0u)
    {
        s_ra_hard_diff_cmd = diff_base;
        s_ra_hard_diff_ready = 1u;
    }

    s_ra_hard_diff_cmd = slew_to_f(s_ra_hard_diff_cmd,
                                   diff_cmd,
                                   RA_HARD_DIFF_SLEW_MAX * PID_DT_SCALE);

    *inner = inner_floor;
    *outer = inner_floor + s_ra_hard_diff_cmd;
    if (*outer < 0.0f)
        *outer = 0.0f;
    if (*outer > MAX_DUTY)
        *outer = MAX_DUTY;
}

static uint8 ra_curve_line_takeover_active(void);

static float ra_curve_steer_assist(void)
{
#if RA_CURVE_ASSIST_ENABLE
    float target;
    float progress;
    float remain;
    float speed_t;
    float taper;
    float assist;
    float rate;

    if (s_ra_state != RA_ST_ACTIVE ||
        s_ra_straight != 0u ||
        s_ra_phase != RA_PH_APPROACH ||
        (s_ra_dir != 1u && s_ra_dir != 2u))
    {
        return 0.0f;
    }

    target = ra_hard_target_limit((float)ra_hard_yaw);
    progress = ra_yaw_progress();
    remain = target - progress;
    if (remain <= RA_CURVE_PID_EXIT_REMAIN_DEG)
        return 0.0f;

    speed_t = range_ratio_i16(ra_speed_ref(),
                              RA_FAST_SPEED_START,
                              RA_CURVE_ASSIST_SPEED_END);
    taper = remain / RA_CURVE_ASSIST_TAPER_DEG;
    if (taper < 0.0f) taper = 0.0f;
    if (taper > 1.0f) taper = 1.0f;

    assist = RA_CURVE_ASSIST_MIN +
             (RA_CURVE_ASSIST_MAX - RA_CURVE_ASSIST_MIN) * speed_t;
    if (s_ra_orig_flag < 3u && s_ra_hard_cnt <= RA_DIRECT_FAST_REVERSE_FRAMES)
    {
        inner = -RA_DIRECT_FAST_REVERSE_DUTY;
        if (outer > RA_DIRECT_ENTRY_OUTER_DUTY_MAX)
            outer = RA_DIRECT_ENTRY_OUTER_DUTY_MAX;
    }

    if (s_ra_orig_flag >= 3u)
    {
        float min_taper = (float)RA_COMPLEX_CURVE_ASSIST_LATE_MIN_PCT * 0.01f;
        assist *= min_taper + (1.0f - min_taper) * taper;
        assist *= (float)RA_COMPLEX_CURVE_ASSIST_PCT * 0.01f;
    }
    else
    {
        assist *= 0.35f + 0.65f * taper;
    }

    rate = ra_yaw_progress_rate();
    if (rate > RA_CURVE_ASSIST_RATE_LIMIT)
    {
        float scale = 1.0f -
            (rate - RA_CURVE_ASSIST_RATE_LIMIT) /
            RA_CURVE_ASSIST_RATE_LIMIT;
        float min_scale = (s_ra_orig_flag >= 3u) ?
                          (float)RA_COMPLEX_CURVE_ASSIST_RATE_MIN_PCT * 0.01f :
                          (float)RA_CURVE_ASSIST_RATE_MIN_PCT * 0.01f;
        if (scale < min_scale) scale = min_scale;
        if (scale > 1.0f) scale = 1.0f;
        assist *= scale;
    }

    if (s_ra_phase == RA_PH_SLOW)
        assist *= (float)RA_CURVE_SLOW_ASSIST_PCT * 0.01f;

    if (ra_curve_line_takeover_active())
        assist *= (float)RA_CURVE_PID_TAKEOVER_ASSIST_PCT * 0.01f;

    return (s_ra_dir == 1u) ? -assist : assist;
#else
    return 0.0f;
#endif
}

/* ======================== RA调试信息更新 ======================== */

/* ra_debug_update - 将RA内部状��制到全�调试变量，供TFT显示
 * 在RA状�机每个返回点调�? * 无参数，无返回�?*/
static void ra_debug_update(void)           /* 更新RA调试变量 */
{
    float yaw_progress = 0.0f;              /* yaw进度，初始为0 */

    if (s_ra_state == RA_ST_ACTIVE && s_ra_dir != 0u) /* RA活跃且有方向 */
        yaw_progress = ra_yaw_progress();   /* 计算yaw进度 */

    ra_dbg_state = (uint8)s_ra_state;       /* 复制RA状�?*/
    ra_dbg_phase = (uint8)s_ra_phase;       /* 复制RA阶� */
    ra_dbg_dir = s_ra_dir;                  /* 复制RA方向 */
    ra_dbg_ip_row = s_ra_ip_row;            /* 复制拐点行号 */
    ra_dbg_timer = s_ra_timer;              /* 复制全局计时�?*/
    ra_dbg_hard_cnt = s_ra_hard_cnt;        /* 复制HARD计数 */
    /* RECOVER阶�显示recover计数，否则显示exit计数 */
    ra_dbg_exit_good_cnt = (s_ra_phase == RA_PH_RECOVER) ? /* 判断当前阶� */
                           s_ra_recover_good_cnt : /* RECOVER阶�显示恢复�数 */
                           s_ra_exit_good_cnt;     /* 其他阶�显示�出��?*/
    /* yaw进度x10，避免浮点显�?*/
    ra_dbg_yaw10 = (int16)(yaw_progress * 10.0f); /* �?0转int16供TFT显示 */
    if (s_ra_state == RA_ST_ACTIVE)
    {
        ra_dbg_slow_row = ra_slow_trigger_row();
        ra_dbg_turn_row = ra_turn_trigger_row();
    }
    else
    {
        ra_dbg_slow_row = 0u;
        ra_dbg_turn_row = 0u;
    }
}

/* ======================== 电机输出限幅 ======================== */

/* clamp_duty - 将浮点duty值限幅到[-MAX_DUTY, MAX_DUTY]并转为int16
 * @val: �点duty�? * 返回: 限幅后的int16 duty�? * NaN�查：�val!=val为假（NaN），返回0保护电机 */
static int16 clamp_duty(float val)          /* 电机duty限幅函数 */
{
    if (val != val) return 0;               /* NaN�查：NaN不等于自�，返�0保护电机 */
    if (val > MAX_DUTY) val = MAX_DUTY;     /* 超过正最大�，限幅 */
    else if (val < -MAX_DUTY) val = -MAX_DUTY; /* 超过负最大�，限幅 */
    return (int16)val;                      /* �点转int16返回 */
}

static void pid_set_duty(int16 left, int16 right)
{
    duty_dbg_left = left;
    duty_dbg_right = right;
    small_driver_set_duty(left, right);
}

/* ======================== RA状�机复位 ======================== */

/* ra_reset - 复位RA状�机�有变量到初�状�? * 无参数，无返回�? * 调用时机：RA结束、电机使能关�、初始化�?*/
static float s_ra_complex_ip_f = 0.0f;
static float s_ra_complex_ip_v = 0.0f;
static uint8 s_ra_complex_ip_valid = 0u;

static void ra_complex_predict_reset(void)
{
    s_ra_complex_ip_f = 0.0f;
    s_ra_complex_ip_v = 0.0f;
    s_ra_complex_ip_valid = 0u;
}

static uint8 ra_complex_row_now(void)
{
    uint8 row = g_ip_max_row;

    if (s_ra_pending_complex_cnt > 0u &&
        s_ra_pending_complex_ip_row > row)
    {
        row = s_ra_pending_complex_ip_row;
    }

    return row;
}

static uint8 ra_complex_predict_ready(uint8 row_now)
{
#if RA_COMPLEX_PREDICT_ENABLE
    float ip;
    float v;
    float pred;

    if (row_now >= RA_COMPLEX_FORCE_ROW)
        return 1u;

    if (g_tf.line_lost != 0u && row_now >= RA_COMPLEX_COMMIT_ROW)
        return 1u;

    if (row_now < RA_COMPLEX_PREDICT_MIN_ROW)
        return 0u;

    ip = (float)row_now;

    if (s_ra_complex_ip_valid == 0u)
    {
        s_ra_complex_ip_f = ip;
        s_ra_complex_ip_v = 0.0f;
        s_ra_complex_ip_valid = 1u;
        return (row_now >= RA_COMPLEX_COMMIT_ROW) ? 1u : 0u;
    }

    v = ip - s_ra_complex_ip_f;
    if (v < 0.0f)
        v = 0.0f;
    if (v > RA_COMPLEX_IP_V_MAX)
        v = RA_COMPLEX_IP_V_MAX;

    s_ra_complex_ip_f = s_ra_complex_ip_f * 0.55f + ip * 0.45f;
    s_ra_complex_ip_v = s_ra_complex_ip_v * 0.65f + v * 0.35f;

    pred = s_ra_complex_ip_f + s_ra_complex_ip_v * RA_COMPLEX_LEAD_FRAMES;

    return (pred >= (float)RA_COMPLEX_COMMIT_ROW) ? 1u : 0u;
#else
    return (row_now >= RA_FIXED_COMPLEX_HARD_ROW) ? 1u : 0u;
#endif
}

static void ra_reset(void)                  /* RA状�机全��?*/
{
    ra_complex_predict_reset();
    s_ra_state = RA_ST_NONE;                /* 状�重�为空� */
    s_ra_phase = RA_PH_WAIT;                /* 阶�重�为等�?*/
    s_ra_dir = 0u;                          /* 方向重置为直�?*/
    s_ra_orig_flag = 0u;                    /* 清除原�flag */
    s_ra_speed_ref_latched = 0;
    s_ra_ip_row = 0u;                       /* 清除拐点行号 */
    s_ra_straight = 0u;                     /* 清除直�标� */
    s_ra_post_edge_side = EDGE_BOTH;        /* 单边方向重置为双�?*/
    s_ra_post_edge_ms = 0u;                 /* 单边时间清零 */
    s_ra_exit_good_cnt = 0u;                /* �出�数清�?*/
    s_ra_recover_good_cnt = 0u;             /* 恢��数清零 */
    s_ra_recover_seen_cnt = 0u;
    s_ra_approach_cnt = 0u;                 /* 接近计数清零 */
    s_ra_timer = 0u;                        /* 全局计时器清�?*/
    s_ra_hard_cnt = 0u;                     /* HARD计数清零 */
    s_ra_recover_cnt = 0u;                  /* RECOVER计数清零 */
    s_ra_yaw_lock_cnt = 0u;
    s_ra_lost_guard_cnt = 0u;
    s_ra_phase_cnt = 0u;                    /* 阶��数清零 */
    s_ra_yaw_base = 0.0f;
    s_ra_hard_yaw_target = 0.0f;
    s_ra_post_recover_cnt = 0u;
    s_ra_complex_lost_match_cnt = 0u;
    s_ra_pre_direct_match_cnt = 0u;
    s_ra_pending_complex_flag = 0u;
    s_ra_pending_complex_cnt = 0u;
    s_ra_pending_complex_ip_row = 0u;
    s_ra_pending_complex_conf = 0u;
    s_ra_pending_complex_bridge = 0u;
    s_ra_hard_speed_seed = 0.0f;            /* 速度种子清零 */
    s_ra_hard_steer_seed = 0.0f;            /* �向�子清零 */
    s_ra_hard_diff_cmd = 0.0f;
    s_ra_hard_diff_ready = 0u;
    s_ra_exit_last_err = 0.0f;
    s_ra_exit_last_turn = 0.0f;
    s_ra_exit_pd_ready = 0u;
    s_ra_line_takeover_cnt = 0u;
    s_ra_pre_turn_ff = 0.0f;
    ra_dbg_line_takeover_ready = 0u;
    ra_dbg_line_takeover_cnt = 0u;
    ra_dbg_exit_reason = RA_EXIT_NONE;
    ra_dbg_hard_target10 = 0;
    ra_dbg_outer_cmd = 0;
    s_route_pending_valid = 0u;             /* 待提交标志清�?*/
    visual_reset_stable();
    turn_assist_reset();
    s_yaw_guard_active = 0u;
    s_over_turn_guard = 0u;
    s_line_takeover_speed_cap_frames = 0u;
    s_continuous_turn_mode = 0u;
    s_continuous_turn_remnant_frames = 0u;
    ra_debug_update();                      /* 更新调试变量 */
}

/* ======================== �线完成��?======================== */

/* update_rules_done - �查路线表��有�则�否都已匹配完�? * 遍历user_rules[]，每条�则要求对应flag的��?=规则的count
 * 全部满足则置 s_rules_done=1，触发延迟停�? * 无参数，无返回�?*/
static void update_rules_done(void)
{
    if (route_dbg_step >= (uint8)USER_RULE_COUNT && !s_route_pending_valid)
        s_rules_done = 1u;
}

/* route_debug_reset - 复位�线调试状�? * 无参数，无返回�?*/
static void route_debug_reset(void)         /* 复位�线调试变� */
{
    route_dbg_step = 0u;                    /* 当前步数清零 */
    route_dbg_total = (uint8)USER_RULE_COUNT; /* 总�数设为规则总数 */
    route_dbg_flag = 0u;                    /* flag清零 */
    route_dbg_count = 0u;                   /* count清零 */
    route_dbg_action = ACT_STRAIGHT;
    s_route_pending_valid = 0u;             /* 待提交有效标志清�?*/
    s_route_pending_flag = 0u;              /* 待提�flag清零 */
    s_route_pending_count = 0u;             /* 待提�count清零 */
    s_route_pending_action = ACT_STRAIGHT;  /* 待提交动作清�?*/
}

/* route_debug_commit - 提交待确认的�线匹配结果到调试变�? * 延迟�帧提交，避免匹配结果影响当帧的RA��
 * 无参数，无返回�?*/
static void route_debug_commit(void)        /* 提交�线调试信� */
{
    if (!s_route_pending_valid)             /* 无待提交数据 */
        return;                             /* 直接返回 */

    if (route_dbg_step < route_dbg_total)
        route_dbg_step++;
    route_dbg_flag = s_route_pending_flag;  /* 提交flag */
    route_dbg_count = s_route_pending_count;/* 提交count */
    route_dbg_action = s_route_pending_action; /* 提交动作 */

    s_route_pending_valid = 0u;             /* 清除待提交标�?*/
}

/* ======================== PD�分项重�?======================== */

/* line_pid_reset_derivative - 完全重置�向PD的微分状态（清零�有历史�）
 * 用于RA��?结束、停机等场景
 * 无参数，无返回�?*/
void line_pid_reset_derivative(void)        /* 完全重置�分状�?*/
{
    s_steer_d_reset_flag = 1u;              /* 设置�分重�标志，下�周期跳过D�?*/
    s_filtered_err = 0.0f;                  /* 滤波��清零 */
    s_prev_steer_output = 0.0f;             /* 上�转向输出清� */
    s_steer_ff_filtered = 0.0f;             /* 前�滤波�清�?*/
    s_cas_target_filtered = 0.0f;           /* 串级�标滤波�清�?*/
    s_cas_last_yaw_err = 0.0f;             /* 串级内环��清零 */
    s_cas_pos_integral = 0.0f;
}

/* line_pid_reset_derivative_keep_output - 部分重置�分状态（保留输出值的50%�? * 用于RECOVER阶��出，避免�向突�
 * 无参数，无返回�?*/
static void line_pid_reset_derivative_keep_output(void) /* 部分重置�� */
{
    s_steer_d_reset_flag = 1u;              /* 设置�分重�标志 */
    s_steer_ff_filtered *= 0.5f;            /* 前��保�?0% */
    s_cas_target_filtered = 0.0f;           /* 串级�标清� */
    s_cas_last_yaw_err = 0.0f;             /* 串级内环��清零 */
    s_cas_pos_integral *= 0.5f;
}

/* ======================== 速度规划器��?======================== */

/* reset_speed_planner - 复位速度规划相关的所有状�? * 无参数，无返回�?*/
static void reset_speed_planner(void)       /* 复位速度规划�?*/
{
    s_base_speed_ramped = 0;                /* 斜坡速度清零 */
    s_straight_cnt = 0u;                    /* 直道计数清零 */
    s_straight_hold = 0u;
    s_straight_active = 0u;                 /* 直道�活标志清�?*/
    s_prev_quality_err = 0;                 /* 上帧��清零 */
    s_prev_quality_err_valid = 0u;          /* 上帧��有效标志清零 */
    speed_dbg_vq_pct = 100u;               /* 视�质量百分比重置�100 */
    speed_dbg_pre_lock = 0u;               /* 预减速锁标志清零 */
    speed_dbg_raw = 0;                      /* 原��度清零 */
    speed_dbg_plan = 0;                     /* 规划速度清零 */
    speed_dbg_reason = 0u;                  /* 速度原因清零 */
}

/* reset_speed_ff_state - 复位速度前�状�? * 无参数，无返回�?*/
static void reset_speed_ff_state(void)      /* 复位速度前� */
{
    s_prev_target_speed = 0.0f;             /* 上一�标�度清零 */
    s_speed_ff_ready = 0u;                  /* 前�就�标志清零 */
}

/* ======================== �线匹配辅� ======================== */

/* route_has_next_match - �查指定flag的下�个count�否能在路线表�找到匹配
 * @flag: �口类型flag�?~5�? * 返回: 1=有下��匹配�0=�?*/
static uint8 route_next_step_index(void)
{
    uint8 step = route_dbg_step;

    if (s_route_pending_valid && step < (uint8)USER_RULE_COUNT)
        step++;

    return step;
}

static uint8 route_has_next_match(uint8 flag)
{
    uint8 step = route_next_step_index();

    if (step >= (uint8)USER_RULE_COUNT)
        return 0u;

    return (user_rules[step].flag == flag) ? 1u : 0u;
}

uint8 route_next_flag_is(uint8 flag)
{
    return route_has_next_match(flag);
}

uint8 route_next_expected_flag(void)
{
    uint8 step = route_next_step_index();

    if (step >= (uint8)USER_RULE_COUNT)
        return 0u;

    return user_rules[step].flag;
}

static uint8 route_next_is_complex(void)
{
    uint8 step = route_next_step_index();

    if (step >= (uint8)USER_RULE_COUNT)
        return 0u;

    return (user_rules[step].flag >= 3u &&
            user_rules[step].flag <= 5u) ? 1u : 0u;
}

static uint8 route_immediate_flag_is(uint8 flag);

static void ra_pending_complex_clear(void)
{
    s_ra_pending_complex_flag = 0u;
    s_ra_pending_complex_cnt = 0u;
    s_ra_pending_complex_ip_row = 0u;
    s_ra_pending_complex_conf = 0u;
    s_ra_pending_complex_bridge = 0u;
}

static uint8 ra_complex_start_quality_ready(void)
{
    if (g_tf.line_lost != 0u)
        return 0u;
    if (g_tf.valid_row_count < RA_INTER_COMPLEX_START_VALID_ROWS)
        return 0u;
    if (abs_i16(g_tf.error) > RA_INTER_COMPLEX_START_ERR_MAX)
        return 0u;
    if (abs_i16(g_tf.lookahead_error) > RA_INTER_COMPLEX_START_LA_MAX)
        return 0u;
    if (abs_i16(g_tf.error_trend) > RA_INTER_COMPLEX_START_TREND_MAX)
        return 0u;

    if (s_lost_line_cnt >= RA_INTER_COMPLEX_RECENT_LOST_FRAMES)
    {
        if (g_tf.valid_row_count < RA_INTER_COMPLEX_RECOVER_VALID_ROWS)
            return 0u;
        if (abs_i16(g_tf.error) > RA_INTER_COMPLEX_RECOVER_ERR_MAX)
            return 0u;
        if (abs_i16(g_tf.lookahead_error) > RA_INTER_COMPLEX_RECOVER_LA_MAX)
            return 0u;
        if (abs_i16(g_tf.error_trend) > RA_INTER_COMPLEX_RECOVER_TREND_MAX)
            return 0u;
    }

    return 1u;
}

static uint8 ra_pending_complex_bridge_context(uint8 flag)
{
    if (s_ra_state != RA_ST_ACTIVE)
        return 0u;
    if (s_ra_orig_flag < 1u || s_ra_orig_flag > 2u)
        return 0u;
    if (s_ra_phase != RA_PH_RECOVER)
        return 0u;
    if (s_ra_recover_cnt < RA_PENDING_COMPLEX_BRIDGE_RECOVER_FRAMES)
        return 0u;
    if (s_ra_recover_seen_cnt < RA_RECOVER_SEEN_CONFIRM_FRAMES)
        return 0u;
    if (ra_complex_start_quality_ready() == 0u)
        return 0u;
    return route_next_flag_is(flag);
}

static uint8 ra_pending_complex_store_conf(uint8 flag)
{
    uint8 conf = RA_INTER_COMPLEX_CONF_PENDING_MIN;

    if (g_inter_result.detected_type == flag &&
        g_inter_result.confidence != 0u)
    {
        conf = g_inter_result.confidence;
    }
    if (conf < RA_INTER_COMPLEX_CONF_PENDING_MIN &&
        ra_pending_complex_bridge_context(flag))
    {
        conf = RA_INTER_COMPLEX_CONF_PENDING_MIN;
    }
    return conf;
}

static uint8 ra_pending_complex_capture_ok(uint8 flag)
{
    uint8 bridge = ra_pending_complex_bridge_context(flag);

    if (flag < 3u || flag > 5u)
        return 0u;
    if (!route_next_flag_is(flag))
        return 0u;
    if (g_tf.line_lost != 0u)
        return 0u;
    if (g_tf.valid_row_count < RA_PENDING_COMPLEX_VALID_ROWS)
        return 0u;
    if (g_ip_max_row < RA_PENDING_COMPLEX_IP_ROW)
        return 0u;
    if (abs_i16(g_tf.error) > RA_PENDING_COMPLEX_ERR_MAX)
        return 0u;
    if (abs_i16(g_tf.lookahead_error) > RA_PENDING_COMPLEX_LA_MAX)
        return 0u;
    if (abs_i16(g_tf.error_trend) > RA_PENDING_COMPLEX_TREND_MAX)
        return 0u;
    if (g_inter_result.detected_type >= 3u &&
        g_inter_result.detected_type <= 5u)
    {
        if (g_inter_result.detected_type != flag)
            return 0u;
        if (g_inter_result.confidence < RA_INTER_COMPLEX_CONF_PENDING_MIN &&
            bridge == 0u)
            return 0u;
    }
    return 1u;
}

static void ra_pending_complex_update(void)
{
    uint8 flag = (uint8)g_ra_flag;
    uint8 expected = route_next_expected_flag();

    if (!route_next_is_complex())
    {
        ra_pending_complex_clear();
        return;
    }

    if (ra_pending_complex_capture_ok(flag))
    {
        s_ra_pending_complex_flag = flag;
        s_ra_pending_complex_cnt = RA_PENDING_COMPLEX_HOLD_FRAMES;
        s_ra_pending_complex_ip_row = g_ip_max_row;
        s_ra_pending_complex_conf = ra_pending_complex_store_conf(flag);
        s_ra_pending_complex_bridge = ra_pending_complex_bridge_context(flag);
        return;
    }

    if ((s_ra_state == RA_ST_ACTIVE || s_ra_post_recover_cnt > 0u) &&
        s_ra_orig_flag >= 1u &&
        s_ra_orig_flag <= 2u &&
        expected >= 3u &&
        expected <= 5u &&
        flag != expected &&
        ra_pending_complex_capture_ok(expected))
    {
        s_ra_pending_complex_flag = expected;
        s_ra_pending_complex_cnt = RA_PENDING_COMPLEX_HOLD_FRAMES;
        s_ra_pending_complex_ip_row = g_ip_max_row;
        s_ra_pending_complex_conf = ra_pending_complex_store_conf(expected);
        s_ra_pending_complex_bridge = ra_pending_complex_bridge_context(expected);
        return;
    }

    if (s_ra_pending_complex_cnt > 0u)
    {
        s_ra_pending_complex_cnt--;
        if (s_ra_pending_complex_cnt == 0u)
            ra_pending_complex_clear();
    }
}

static uint8 ra_pending_complex_start_ok(uint8 flag)
{
    if (s_ra_pending_complex_cnt == 0u)
        return 0u;
    if (s_ra_pending_complex_flag != flag)
        return 0u;
    if (s_ra_pending_complex_ip_row < RA_PENDING_COMPLEX_IP_ROW)
        return 0u;
    if (s_ra_pending_complex_conf < RA_INTER_COMPLEX_CONF_PENDING_MIN)
        return 0u;
    return route_immediate_flag_is(flag);
}

static uint8 route_immediate_flag_is(uint8 flag)
{
    if (route_dbg_step >= (uint8)USER_RULE_COUNT)
        return 0u;
    return (user_rules[route_dbg_step].flag == flag) ? 1u : 0u;
}

static void ra_clear_pre_flags(void)
{
    g_ra_pre_flag = 0u;
    g_ra_pre_dir = 0u;
    g_ra_pre_slow_flag = 0u;
    s_ra_pre_direct_match_cnt = 0u;
    s_pre_lock = 0u;
    s_pre_timeout = 0u;
}

static void ra_clear_all_flags(void)
{
    g_ra_flag = 0u;
    s_ra_complex_lost_match_cnt = 0u;
    ra_pending_complex_clear();
    ra_clear_pre_flags();
}

static void ra_clear_flags_keep_pending(void)
{
    g_ra_flag = 0u;
    s_ra_complex_lost_match_cnt = 0u;
    ra_clear_pre_flags();
}

static uint8 ra_keep_next_route_flag(uint8 flag)
{
    if (flag == 0u || !route_next_flag_is(flag))
        return 0u;

    if (flag >= 3u && flag <= 5u)
    {
        if (g_tf.line_lost != 0u)
            return 0u;
        if (g_tf.valid_row_count < RA_KEEP_COMPLEX_VALID_ROWS)
            return 0u;
        if (abs_i16(g_tf.error) > RA_KEEP_COMPLEX_ERR_MAX)
            return 0u;
        if (abs_i16(g_tf.lookahead_error) > RA_KEEP_COMPLEX_LA_MAX)
            return 0u;
        if (abs_i16(g_tf.error_trend) > RA_KEEP_COMPLEX_TREND_MAX)
            return 0u;
    }

    return flag;
}

/* ======================== 丢线搜索 ======================== */

/* lost_search_reset - 复位丢线搜索状�? * 无参数，无返回�?*/
static void lost_search_reset(void)         /* 复位丢线搜索 */
{
    s_lost_search_active = 0u;              /* 搜索�活标志清�?*/
    s_lost_line_cnt = 0u;                   /* 丢线帧�数清�?*/
    s_lost_search_cnt = 0u;                 /* 搜索运�帧计数清�?*/
}

/* lost_search_pick_dir - 根据丢线前的�后��选择搜索方向
 * @err: 丢线前的�后位���? * 返回: 1=向右搜索�?=向左搜索
 * err > 死区 �?向右搜索(1)
 * err < -死区 �?向左搜索(2)
 * 在�区� �?保持上�搜索方� */
static uint8 lost_search_pick_dir(int16 err) /* 选择丢线搜索方向 */
{
    if (err > LOST_SEARCH_ERR_DEADZONE)      /* ��偏右，向右搜�?*/
        return 1u;                           /* 返回右转方向 */
    if (err < -LOST_SEARCH_ERR_DEADZONE)     /* ��偏左，向左搜�?*/
        return 2u;                           /* 返回左转方向 */

    return (s_lost_search_dir == 2u) ? 2u : 1u; /* 在�区内，保持上�方�?*/
}

/* lost_search_step - 丢线搜索主�辑，每PID周期调用
 * @pos_err: 当前位置��
 * 返回: 1=正在搜索（本帧已直接输出电机），0=�搜索（继�正常PID�? *
 * 流程�? *   1. 若已找回线（line_lost=0且有效�足够）� �出搜�? *   2. �RA活跃 �?不搜�? *   3. �RA flag存在 �?不搜�（�RA处理�? *   4. 连续丢线达到阈�?�?�动搜�
 *   5. 搜索�定期切换方向，原地左右旋�寻线
 *   6. 搜索时�度清零，只输出��?*/
static uint8 lost_search_step(int16 pos_err) /* 丢线搜索主�辑 */
{
    /* �出条件：找回线且有效行足�?*/
    if (g_tf.line_lost == 0u &&             /* ���?*/
        g_tf.valid_row_count >= LOST_SEARCH_EXIT_VALID_ROWS) /* 有效行足�?*/
    {
        s_lost_last_err = pos_err;          /* 记录找回线时的��?*/
        lost_search_reset();                /* 复位搜索状�?*/
        return 0u;                          /* �出搜�，继�正常PID */
    }

    /* RA活跃时不搜索 */
    if (s_ra_state != RA_ST_NONE)           /* RA正在执� */
    {
        lost_search_reset();                /* 复位搜索状�?*/
        return 0u;                          /* 不搜�?*/
    }

    /* 有RA flag时不搜索（�RA处理�?*/
    if (g_ra_flag != 0u || g_ra_pre_flag != 0u) /* 有RA标志 */
        return 0u;                          /* 不搜�，交给RA处理 */

    if (s_ra_lost_guard_cnt > 0u)
    {
        s_ra_lost_guard_cnt--;
        s_lost_line_cnt = 0u;
        s_lost_search_active = 0u;
        return 0u;
    }

    if (s_ra_post_recover_cnt > 0u)
    {
        s_lost_line_cnt = 0u;
        s_lost_search_active = 0u;
        return 0u;
    }

    /* �丢线时重�计数 */
    if (g_tf.line_lost == 0u)              /* 当前���?*/
    {
        s_lost_line_cnt = 0u;              /* 丢线计数清零 */
        return 0u;                          /* 不搜�?*/
    }

    /* �计丢线帧� */
    if (s_lost_line_cnt < 255u)            /* 防溢�?*/
        s_lost_line_cnt++;                 /* 丢线帧�数�1 */

    /* �达到�动阈�?*/
    if (s_lost_line_cnt < LOST_SEARCH_ENTER_FRAMES) /* 丢线帧数不足 */
        return 0u;                          /* 不启动搜�?*/

    /* 首�进入搜�：�择方向 */
    if (!s_lost_search_active)              /* 尚未�活搜�?*/
    {
        s_lost_search_active = 1u;          /* �活搜�?*/
        s_lost_search_cnt = 0u;             /* 搜索计数清零 */
        s_lost_search_dir = lost_search_pick_dir(s_lost_last_err); /* 根据�后��选方�?*/
    }

    /* 定期切换搜索方向（防止单方向�过头� */
    s_lost_search_cnt++;                    /* 搜索帧�数�1 */
    if (s_lost_search_cnt >= LOST_SEARCH_SWITCH_FRAMES) /* 达到切换阈�?*/
    {
        s_lost_search_cnt = 0u;             /* 计数清零 */
        s_lost_search_dir = (s_lost_search_dir == 1u) ? 2u : 1u; /* 左右切换 */
    }

    /* 搜索时清零�度和积分，重置PID状�?*/
    base_speed = 0;                         /* 基�速度清零 */
    s_speed_integral = 0.0f;                /* 速度�分清� */
    reset_speed_planner();                  /* 复位速度规划�?*/
    reset_speed_ff_state();                 /* 复位速度前� */
    line_pid_reset_derivative();            /* 重置�分状�?*/

    /* 输出�速：forward recovery, no wheel reverse while running fast */
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

/* single_edge_reset - 复位单边巡线状�，恢�双边模�
 * 无参数，无返回�?*/
static void single_edge_reset(void)         /* 复位单边巡线 */
{
    uint8 was_single_edge = (s_edge_active != 0u || g_post_edge_side != EDGE_BOTH) ? 1u : 0u;
    s_edge_active = 0u;                     /* �活标志清�?*/
    s_edge_side = EDGE_BOTH;                /* 方向恢�双� */
    s_edge_cnt = 0u;                        /* 帧�数清�?*/
    s_edge_age_cnt = 0u;
    s_edge_target = 0u;                     /* �标帧数清� */
    s_edge_until_next_turn = 0u;            /* 清除保持模式 */
    s_edge_release_after_turn = 0u;         /* 清除��后释放标�?*/
    s_single_edge_fast_hold = 0u;
    g_post_edge_side = EDGE_BOTH;           /* 全局方向恢�双� */
    if (was_single_edge)
        turn_right_led_off();
}

/* start_single_edge - �动单边巡线模�
 * @side: EDGE_LEFT �?EDGE_RIGHT
 * @duration_ms: 持续时间（�秒），会���为PID周期帧数
 * 非法参数时�位为双边模� */
void start_single_edge(uint8 side, uint16 duration_ms) /* �动单边巡� */
{
    if ((side != EDGE_LEFT && side != EDGE_RIGHT) || duration_ms == 0u) /* 参数合法性��?*/
    {
        single_edge_reset();                /* 参数非法，�位为双� */
        return;                             /* 返回 */
    }

    s_edge_active = 1u;                     /* �活单边巡�?*/
    s_edge_side = side;                     /* 设置巡线方向 */
    s_edge_cnt = 0u;                        /* 帧�数清�?*/
    s_edge_age_cnt = 0u;

    if (duration_ms == SINGLE_EDGE_UNTIL_NEXT_TURN)
    {
        s_edge_until_next_turn = 1u;        /* 保持到下�次真正转�?*/
        s_edge_release_after_turn = 0u;     /* 尚未遇到�要释放的�� */
        s_edge_target = 0u;                 /* 不使用时间��?*/
    }
    else
    {
        s_edge_until_next_turn = 0u;        /* �通定时单�?*/
        s_edge_release_after_turn = 0u;     /* �通定时模式不使用��释放 */
        /* Convert ms to PID ticks, rounded up. */
        s_edge_target = PID_MS_TO_TICKS(duration_ms);
        if (s_edge_target == 0u)            /* 防�转换后�0 */
            s_edge_target = 1u;             /* 保证至少1�?*/
    }

    g_post_edge_side = side;                /* 设置全局单边方向 */
    turn_right_led_on();
}

/* single_edge_tick - 单边巡线每帧tick，由 line_pid_control() 调用
 * �� g_post_edge_side 与�定方向�致，计时到期后自动��? * 无参数，无返回�?*/
static void single_edge_tick(void)          /* 单边巡线帧更�?*/
{
    if (!s_edge_active)                     /* ���?*/
        return;                             /* 直接返回 */

    /* 防��外部意�修�?*/
    if (g_post_edge_side != s_edge_side)    /* 全局方向与�定不一�?*/
        g_post_edge_side = s_edge_side;     /* 恢�为设定方�?*/

    if (s_edge_age_cnt < 65535u)
        s_edge_age_cnt++;

    if (s_edge_until_next_turn)             /* 保持模式不按时间结束 */
        return;

    s_edge_cnt++;                           /* 帧�数�1 */
    if (s_edge_cnt >= s_edge_target)        /* 达到�标帧� */
        single_edge_reset();                /* 复位单边巡线 */
}

/* ======================== ��屏蔽（Turn Shield�?======================== */

/* ra_finish_ex - RA结束的扩展版�? * @keep_flag: 结束后保留的RA flag�?=清除，非0=保留给下�个RA�? * @use_shield: �否启动转�屏蔽
 *
 * 逻辑�? *   1. 关闭LED
 *   2. 重置速度�分和PID��
 *   3. 提交�线调试信�
 *   4. �查路线完成状�? *   5. 根据post_edge配置�动单边巡�
 *   6. �动转�屏蔽（可选）
 *   7. 复位RA状�机 */
static void ra_finish_ex(uint8 keep_flag) /* RA结束扩展 */
{
    uint8 edge_side = EDGE_BOTH;            /* 单边巡线方向，初始为双边 */
    uint8 force_left_single_edge = 0u;
    uint8 from_recover = (s_ra_phase == RA_PH_RECOVER) ? 1u : 0u; /* �否从RECOVER��?*/

    turn_right_led_off();                   /* 关闭右转指示LED */
    g_ra_flag = keep_flag;                  /* 设置保留的flag�?=清除�?*/
    if (from_recover)
    {
        s_speed_integral *= 0.50f;
        s_ra_post_recover_complex = route_next_is_complex();
        s_ra_post_recover_cnt = s_ra_post_recover_complex ?
                                RA_POST_RECOVER_COMPLEX_FRAMES :
                                RA_POST_RECOVER_FRAMES;
    }
    else
    {
        s_speed_integral = 0.0f;
        s_ra_post_recover_complex = 0u;
    }
    /* RECOVER�出时保留部分输出，避免突�?*/
    if (from_recover)                       /* 从RECOVER阶���?*/
        line_pid_reset_derivative_keep_output(); /* 部分重置，保�?0%输出 */
    else                                    /* 从其他阶段��?*/
        line_pid_reset_derivative();        /* 完全重置�分状�?*/
    route_debug_commit();                   /* 提交�线调试信� */
    update_rules_done();                    /* �查路线完成状�?*/

    if (s_ra_orig_flag == 1u && s_ra_straight == 0u)
    {
        if (s_completed_right_ra_count < 255u)
            s_completed_right_ra_count++;
        if (s_completed_right_ra_count >= 2u && s_edge_release_after_turn == 0u)
            force_left_single_edge = 1u;
    }

    if (s_edge_release_after_turn)          /* 已经完成下一次真正转�?*/
        single_edge_reset();                /* 单边任务结束，恢复双�?*/

    /* �定单边巡线方� */
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
    else if (s_ra_post_edge_side == EDGE_AUTO) /* �动模� */
    {
        /* �动模式：右转后用左边线，左转后用右边� */
        if (s_ra_dir == 1u)                 /* 右转 */
            edge_side = EDGE_LEFT;          /* 用左边线 */
        else if (s_ra_dir == 2u)            /* 左转 */
            edge_side = EDGE_RIGHT;         /* 用右边线 */
    }

    /* �动单边巡� */
    if (edge_side != EDGE_BOTH && s_ra_post_edge_ms > 0u) /* �要单边且时间>0 */
        start_single_edge(edge_side, s_ra_post_edge_ms); /* �动单边巡� */

    g_ra_pre_flag = 0u;
    g_ra_pre_dir = 0u;
    g_ra_pre_slow_flag = 0u;
    s_pre_lock = 0u;
    s_pre_timeout = 0u;
    if (keep_flag == 0u && (from_recover || s_ra_straight != 0u))
        track_intersection_suppress_after_turn();
    ra_reset();                             /* 复位RA状�机 */
}

/* ra_finish - RA正常结束：清�flag + �动转�屏蔽
 * 无参数，无返回�?*/
static void ra_finish(void)                 /* RA正常结束 */
{
    uint8 saved_dir = s_ra_dir;
    uint8 saved_orig_flag = s_ra_orig_flag;
    uint8 saved_straight = s_ra_straight;

    ra_finish_ex(0u);                   /* 清除flag，启动转�屏�?*/

    /* Item 8: Set continuous turn remnant frames for direct turns,
     * so a closely-following same-direction RA gets turn-assist priming */
    if (saved_orig_flag < 3u && saved_straight == 0u)
    {
        s_continuous_turn_remnant_frames = RA_CONTINUOUS_TURN_REMNANT_FRAMES;
        s_continuous_turn_post_dir = saved_dir;
    }
}

static uint8 ra_line_takeover_ready(void)
{
#if RA_LINE_TAKEOVER_ENABLE
    float yaw_now;
    float rate_abs;

    ra_dbg_line_takeover_ready = 0u;
    ra_dbg_takeover_valid_rows = g_tf.valid_row_count;
    ra_dbg_takeover_error = g_tf.error;
    ra_dbg_takeover_lookahead = g_tf.lookahead_error;
    ra_dbg_takeover_trend = g_tf.error_trend;

    if (s_ra_state != RA_ST_ACTIVE)
        return 0u;
    if (s_ra_straight != 0u)
        return 0u;
    if (s_ra_dir != 1u && s_ra_dir != 2u)
        return 0u;
    if (s_ra_orig_flag >= 3u)
        return 0u;

    if (s_ra_phase != RA_PH_HARD &&
        s_ra_phase != RA_PH_YAW_LOCK &&
        s_ra_phase != RA_PH_RECOVER)
    {
        s_ra_line_takeover_cnt = 0u;
        ra_dbg_line_takeover_cnt = 0u;
        return 0u;
    }

    yaw_now = ra_abs_yaw_progress();
    rate_abs = ra_abs_yaw_rate();

    if (yaw_now < RA_LINE_TAKEOVER_MIN_YAW_DEG)
    {
        s_ra_line_takeover_cnt = 0u;
        ra_dbg_line_takeover_cnt = 0u;
        return 0u;
    }

    if (rate_abs > RA_LINE_TAKEOVER_RATE_MAX)
    {
        s_ra_line_takeover_cnt = 0u;
        ra_dbg_line_takeover_cnt = 0u;
        return 0u;
    }

    if (g_tf.line_lost != 0u)
    {
        s_ra_line_takeover_cnt = 0u;
        ra_dbg_line_takeover_cnt = 0u;
        return 0u;
    }

    if (g_tf.valid_row_count < RA_LINE_TAKEOVER_VALID_ROWS)
    {
        s_ra_line_takeover_cnt = 0u;
        ra_dbg_line_takeover_cnt = 0u;
        return 0u;
    }

    if (abs_i16(g_tf.error) > RA_LINE_TAKEOVER_ERR_MAX)
    {
        s_ra_line_takeover_cnt = 0u;
        ra_dbg_line_takeover_cnt = 0u;
        return 0u;
    }

    if (abs_i16(g_tf.lookahead_error) > RA_LINE_TAKEOVER_LA_MAX)
    {
        s_ra_line_takeover_cnt = 0u;
        ra_dbg_line_takeover_cnt = 0u;
        return 0u;
    }

    if (abs_i16(g_tf.error_trend) > RA_LINE_TAKEOVER_TREND_MAX)
    {
        s_ra_line_takeover_cnt = 0u;
        ra_dbg_line_takeover_cnt = 0u;
        return 0u;
    }

    if (s_ra_line_takeover_cnt < 255u)
        s_ra_line_takeover_cnt++;

    ra_dbg_line_takeover_cnt = s_ra_line_takeover_cnt;
    if (s_ra_line_takeover_cnt >= RA_LINE_TAKEOVER_CONFIRM_FRAMES)
    {
        ra_dbg_line_takeover_ready = 1u;
        return 1u;
    }

    return 0u;
#else
    return 0u;
#endif
}

/* ==================== visual_exit_ready (item 2) ====================
 * 普通直角1/2出弯视觉判断，用于HARD/YAW_LOCK/RECOVER阶段。
 * 第一版宽松条件，可收紧到严格条件。
 */
static uint8 visual_exit_ready(uint8 use_strict)
{
    uint8 stable = 0u;
    uint8 valid;
    uint16 vrows;
    int16 e, la, tr;

    if (s_ra_straight != 0u)
        return 0u;
    if (s_ra_orig_flag >= 3u)
        return 0u;
    if (s_ra_dir != 1u && s_ra_dir != 2u)
        return 0u;

    vrows = g_tf.valid_row_count;
    e = abs_i16(g_tf.error);
    la = abs_i16(g_tf.lookahead_error);
    tr = abs_i16(g_tf.error_trend);

    if (use_strict)
    {
        /* 严格条件 */
        if (g_tf.line_lost == 0u &&
            vrows >= RA_VISUAL_EXIT_STRICT_VALID_ROWS &&
            e <= RA_VISUAL_EXIT_STRICT_ERR_MAX &&
            la <= RA_VISUAL_EXIT_STRICT_LA_MAX &&
            tr <= RA_VISUAL_EXIT_STRICT_TREND_MAX)
        {
            stable = 1u;
        }
    }
    else
    {
        /* 正常条件 */
        if (g_tf.line_lost == 0u &&
            vrows >= RA_VISUAL_EXIT_VALID_ROWS &&
            e <= RA_VISUAL_EXIT_ERR_MAX &&
            la <= RA_VISUAL_EXIT_LA_MAX &&
            tr <= RA_VISUAL_EXIT_TREND_MAX)
        {
            stable = 1u;
        }
    }

    /* 检查是否与新的路口候选冲突 */
    if (stable && g_fast_ra_type != 0u)
    {
        uint8 conflict = 0u;
        if (g_fast_ra_type == 1u && s_ra_dir == 2u) conflict = 1u;
        if (g_fast_ra_type == 2u && s_ra_dir == 1u) conflict = 1u;
        if (g_fast_ra_type >= 3u) conflict = 1u;
        if (conflict)
            stable = 0u;
    }

    if (stable)
    {
        if (use_strict)
        {
            if (s_visual_strict_cnt < 255u)
                s_visual_strict_cnt++;
            valid = (s_visual_strict_cnt >= RA_VISUAL_EXIT_VERY_STRICT_FRAMES) ? 1u : 0u;
        }
        else
        {
            if (s_visual_stable_cnt < 255u)
                s_visual_stable_cnt++;
            valid = (s_visual_stable_cnt >= RA_VISUAL_EXIT_STABLE_FRAMES) ? 1u : 0u;
        }
    }
    else
    {
        if (use_strict)
            s_visual_strict_cnt = 0u;
        else
            s_visual_stable_cnt = 0u;
        valid = 0u;
    }

    if (use_strict == 0u)
        ra_dbg_visual_stable_cnt = s_visual_stable_cnt;

    return valid;
}

static void visual_reset_stable(void)
{
    s_visual_stable_cnt = 0u;
    s_visual_strict_cnt = 0u;
}

/* ==================== 普通直角补线模式 (item 3) ====================
 * 参考补线/拉线方法：
 *   右直角：在图像右侧固定列范围(78~88)找白线跳变
 *   左直角：在图像左侧固定列范围(6~16)找白线跳变
 *   从图像底部中心向找到的白点拉临时中线
 *   补线权重渐进增加，出弯稳定后回落
 */
static void turn_assist_reset(void)
{
    s_turn_assist_active = 0u;
    s_turn_assist_weight = 0u;
    s_turn_assist_found_col = -1;
    s_turn_assist_frame_cnt = 0u;
}

static uint8 turn_assist_is_direct(void)
{
    if (s_ra_state != RA_ST_ACTIVE)
        return 0u;
    if (s_ra_straight != 0u)
        return 0u;
    if (s_ra_orig_flag >= 3u)
        return 0u;
    if (s_ra_dir != 1u && s_ra_dir != 2u)
        return 0u;
    if (s_ra_phase != RA_PH_HARD &&
        s_ra_phase != RA_PH_YAW_LOCK &&
        s_ra_phase != RA_PH_RECOVER)
        return 0u;
    /* Check g_fast_ra_type doesn't conflict */
    if (g_fast_ra_type != 0u)
    {
        if (g_fast_ra_type == 1u && s_ra_dir != 1u) return 0u;
        if (g_fast_ra_type == 2u && s_ra_dir != 2u) return 0u;
        if (g_fast_ra_type >= 3u) return 0u;
    }
    return 1u;
}

/* Scan binarized image for white transition at a fixed column */
static int16 turn_assist_scan_column(uint8 col, uint8 start_row, uint8 end_row)
{
    int16 r;
    uint8 prev = Image_Binarize[start_row][col];

    for (r = (int16)start_row - 1; r >= (int16)end_row; r--)
    {
        uint8 cur = Image_Binarize[(uint8)r][col];
        if (prev == Image_BLACK && cur == Image_WHITE)
            return r;
        prev = cur;
    }
    return -1;
}

/* Find the best white point for turn assist */
static int16 turn_assist_find_point(void)
{
    uint8 col_start, col_end;
    uint8 start_row = (TF_IMG_H > 4u) ? (uint8)(TF_IMG_H - 4u) : (uint8)(TF_IMG_H - 1u);
    uint8 end_row = TF_SEARCH_END_ROW;
    int16 best_row = -1;
    uint8 best_col = 0;
    uint8 c;

    if (s_ra_dir == 1u)
    {
        /* Right turn: scan right side 78~88 */
        col_start = RA_TURN_ASSIST_RIGHT_START;
        col_end = RA_TURN_ASSIST_RIGHT_END;
    }
    else if (s_ra_dir == 2u)
    {
        /* Left turn: scan left side 6~16 */
        col_start = RA_TURN_ASSIST_LEFT_START;
        col_end = RA_TURN_ASSIST_LEFT_END;
    }
    else
    {
        return -1;
    }

    if (col_end >= TF_IMG_W)
        col_end = (uint8)(TF_IMG_W - 1u);
    if (col_start >= TF_IMG_W)
        col_start = (uint8)(TF_IMG_W - 1u);

    for (c = col_start; c <= col_end; c++)
    {
        int16 row = turn_assist_scan_column(c, start_row, end_row);
        if (row > best_row)
        {
            best_row = row;
            best_col = c;
        }
    }

    return (best_row >= 0) ? (int16)best_col : -1;
}

/* Apply turn assist: replace center_line with a fake line from bottom-center to found point */
static uint8 turn_assist_apply(void)
{
    int16 found_col;
    uint8 new_weight;
    uint8 center = TF_IMG_CENTER;
    uint8 bottom_row = (uint8)(TF_IMG_H - 1u);

    if (!turn_assist_is_direct())
    {
        if (s_turn_assist_active)
        {
            /* Gradually reduce weight when exiting turn assist */
            if (s_turn_assist_weight > RA_TURN_ASSIST_WEIGHT_RECOVER)
                s_turn_assist_weight -= RA_TURN_ASSIST_WEIGHT_RECOVER;
            else
                s_turn_assist_weight = 0u;
            if (s_turn_assist_weight == 0u)
                turn_assist_reset();
        }
        ra_dbg_turn_assist_active = s_turn_assist_active;
        ra_dbg_turn_assist_weight = s_turn_assist_weight;
        return s_turn_assist_active;
    }

    found_col = turn_assist_find_point();
    s_turn_assist_found_col = found_col;
    ra_dbg_turn_assist_found_col = found_col;

    if (found_col >= 0)
    {
        /* Found a white point - compute fake center line */
        uint8 r;
        s_turn_assist_frame_cnt++;

        /* Progressive weight */
        if (s_turn_assist_frame_cnt == 1u)
            new_weight = RA_TURN_ASSIST_WEIGHT_FRAME1;
        else if (s_turn_assist_frame_cnt == 2u)
            new_weight = RA_TURN_ASSIST_WEIGHT_FRAME2;
        else if (s_turn_assist_frame_cnt == 3u)
            new_weight = RA_TURN_ASSIST_WEIGHT_FRAME3;
        else
            new_weight = RA_TURN_ASSIST_WEIGHT_MAX;

        if (!s_turn_assist_active)
        {
            s_turn_assist_active = 1u;
            s_turn_assist_weight = 0u;
        }
        s_turn_assist_weight = new_weight;

        /* Build fake center line: bottom-center to found point */
        for (r = 0u; r < TF_IMG_H; r++)
        {
            int16 fake_center;
            float row_t;

            if (r >= (uint8)found_col || bottom_row <= 0u)
            {
                fake_center = (int16)center;
            }
            else
            {
                row_t = (float)r / (float)bottom_row;
                fake_center = (int16)((float)center * (1.0f - row_t) +
                                      (float)found_col * row_t);
            }

            if (fake_center < 0) fake_center = 0;
            if (fake_center >= (int16)TF_IMG_W) fake_center = (int16)(TF_IMG_W - 1);

            /* Only override if assist weight is meaningful */
            if (g_tf.center_line[r] >= 0)
            {
                int16 blended = (int16)((float)g_tf.center_line[r] * (100u - (uint16)s_turn_assist_weight) / 100.0f +
                                        (float)fake_center * (float)s_turn_assist_weight / 100.0f);
                g_tf.center_line[r] = blended;
            }
            else
            {
                g_tf.center_line[r] = fake_center;
            }
        }
    }
    else
    {
        /* No white point found: fallback to single-edge */
        s_turn_assist_frame_cnt = 0u;
        if (s_turn_assist_active)
        {
            if (s_turn_assist_weight > RA_TURN_ASSIST_WEIGHT_RECOVER)
                s_turn_assist_weight -= RA_TURN_ASSIST_WEIGHT_RECOVER;
            else
                s_turn_assist_weight = 0u;
            if (s_turn_assist_weight == 0u)
                turn_assist_reset();
        }

        /* Fallback: use single edge on the opposite side */
        if (s_ra_dir == 1u)
        {
            /* Right turn: use left edge only */
            if (g_post_edge_side != EDGE_LEFT)
                g_post_edge_side = EDGE_LEFT;
        }
        else
        {
            /* Left turn: use right edge only */
            if (g_post_edge_side != EDGE_RIGHT)
                g_post_edge_side = EDGE_RIGHT;
        }
    }

    ra_dbg_turn_assist_active = s_turn_assist_active;
    ra_dbg_turn_assist_weight = s_turn_assist_weight;
    return s_turn_assist_active;
}

/* ==================== Direct turn diff-limit helpers (item 6) ==================== */
static uint8 ra_direct_diff_inner_pct(void)
{
    return RA_DIRECT_INNER_MIN_PCT;
}

static uint8 ra_direct_diff_outer_boost_pct(void)
{
    float rate_abs = ra_abs_yaw_rate();
    uint8 pct = RA_DIRECT_OUTER_MAX_BOOST_PCT;

    if (rate_abs > RA_YAW_RATE_OVER_LIMIT * 0.7f)
    {
        /* Reduce outer boost when yaw rate is high */
        uint8 reduction = (uint8)((rate_abs - RA_YAW_RATE_OVER_LIMIT * 0.7f) /
                                   (RA_YAW_RATE_OVER_LIMIT * 0.3f) * (float)RA_DIRECT_OUTER_MAX_BOOST_PCT);
        if (reduction > pct)
            pct = 0u;
        else
            pct -= reduction;
    }
    return pct;
}

static uint8 ra_direct_reverse_allowed(void)
{
#if RA_DIRECT_REVERSE_ENABLE_SPEED_LOW_ONLY
    if (ra_speed_ref() > RA_LOW_SPEED_START)
        return 0u;
#endif
    if (s_ra_orig_flag >= 3u)
        return 0u;
    if (s_over_turn_guard)
        return 0u;
    return 1u;
}

static void ra_finish_by_line_takeover(void)
{
    float keep_steer = s_ra_hard_steer_seed;

    ra_finish();

    s_steer_d_reset_flag = 1u;
    s_filtered_err = (float)g_tf.error;
    s_steer_last_pos_err = s_filtered_err;
    s_steer_last_raw_err = (float)g_tf.error;
    s_steer_ff_filtered = 0.0f;
    s_cas_target_filtered = 0.0f;
    s_cas_last_yaw_err = 0.0f;
    s_cas_pos_integral *= 0.5f;

    s_prev_steer_output = keep_steer *
        ((float)RA_LINE_TAKEOVER_STEER_KEEP_PCT * 0.01f);

    s_base_speed_ramped = (int16)((float)motor_speed * 8.0f *
        ((float)RA_LINE_TAKEOVER_SPEED_PCT * 0.01f));
    s_prev_target_speed = (float)s_base_speed_ramped;
    s_speed_integral *= 0.5f;
    s_speed_ff_ready = 1u;

#if RA_EXIT_BOOST_ENABLE
    s_ra_exit_boost_active = 1u;
    s_ra_exit_boost_cnt = 0u;
    ra_dbg_exit_boost_active = 1u;
    ra_dbg_exit_boost_cnt = 0u;
#endif

    /* Item 7: speed cap for first N frames after line takeover */
    s_line_takeover_speed_cap_frames = RA_TAKEOVER_SPEED_CAP_FRAMES;
    ra_dbg_line_takeover_speed_cap = s_line_takeover_speed_cap_frames;
}

static void ra_enter_yaw_lock(void)
{
    turn_right_led_off();
    g_ra_flag = 0u;
    g_ra_pre_dir = 0u;
    g_ra_pre_slow_flag = 0u;
    s_ra_phase = RA_PH_YAW_LOCK;
    s_ra_phase_cnt = 0u;
    s_ra_yaw_lock_cnt = 0u;
    s_ra_recover_cnt = 0u;
    s_speed_integral *= 0.50f;
    line_pid_reset_derivative();
    ra_debug_update();
}

/* ra_enter_recover - 进入RECOVER阶�
 * 特点�? *   - 使用HARD阶�的速度/�向�子值平滑过�? *   - 重新初�化PD控制�（以当前��为初始�）
 *   - 速度前�就�（避免启动突变）
 * 无参数，无返回�?*/
static void ra_enter_recover(void)          /* 进入RECOVER阶� */
{
    turn_right_led_off();                   /* 关闭右转指示LED */
    g_ra_flag = 0u;                         /* 清除RA flag */
    g_ra_pre_dir = 0u;
    g_ra_pre_slow_flag = 0u;
    s_ra_phase = RA_PH_RECOVER;             /* 切换到RECOVER阶� */
    s_ra_phase_cnt = 0u;                    /* 阶��数清零 */
    s_ra_recover_cnt = 0u;                  /* RECOVER帧�数清�?*/
    s_ra_yaw_lock_cnt = 0u;
    s_ra_lost_guard_cnt = RA_LOST_GUARD_FRAMES;
    s_ra_recover_good_cnt = 0u;             /* RECOVER�认�数清零 */
    s_ra_recover_seen_cnt = 0u;
    s_ra_exit_last_err = 0.0f;
    s_ra_exit_last_turn = s_ra_hard_steer_seed *
                          ((float)RA_RECOVER_SEED_STEER_PCT * 0.01f);
    s_ra_exit_pd_ready = 0u;
    s_speed_integral *= 0.60f;
    route_debug_commit();
    s_steer_d_reset_flag = 1u;              /* 设置�分重�标志 */
    /* 以当前����初�化滤波�，避免恢复时跳�?*/
    s_filtered_err = (float)g_tf.error;     /* 用当前��初�化滤波�?*/
    s_steer_last_pos_err = s_filtered_err;  /* 上�滤波�� = 当前滤波�?*/
    s_steer_last_raw_err = (float)g_tf.error; /* 上�原始�� = 当前�� */
    s_steer_ff_filtered = 0.0f;             /* 前�滤波�清�?*/
    s_cas_target_filtered = 0.0f;           /* 串级�标滤波�清�?*/
    s_cas_last_yaw_err = 0.0f;             /* 串级内环��清零 */
    /* 从HARD种子值按比例过渡 */
    s_prev_steer_output = s_ra_hard_steer_seed * /* HARD阶�转向�子 */
                          ((float)RA_RECOVER_SEED_STEER_PCT * 0.01f); /* RECOVER比例过渡 */
    s_base_speed_ramped = (int16)((float)motor_speed * 8.0f *
                                  ((float)RA_RECOVER_SPEED_PCT * 0.01f));
    s_prev_target_speed = (float)s_base_speed_ramped; /* 上一�标�度 = 种子�?*/
    s_speed_ff_ready = 1u;                  /* 前�就� */
    ra_debug_update();                      /* 更新调试变量 */
}

/* ra_start - �动RA状�机
 * @dir: 方向�?=直� 1=�?2=左）
 * @orig_flag: 原�flag�? * @straight: �否直行�过
 * @post_edge_side: 结束后单边方�? * @post_edge_ms: 结束后单边时�? * 无返回�?*/
static void ra_start(uint8 dir, uint8 orig_flag, uint8 straight,
                     uint8 post_edge_side, uint16 post_edge_ms) /* �动RA */
{
    if (!straight && s_edge_until_next_turn)/* 下一�真�转�已开�?*/
        s_edge_release_after_turn = 1u;     /* 等这个RA结束后恢复双�?*/

    s_ra_dir = dir;                         /* 设置��方向 */
    s_ra_orig_flag = orig_flag;             /* 保存原�flag */
    s_ra_speed_ref_latched = ra_speed_ref();
    s_ra_state = RA_ST_ACTIVE;              /* 状�切�为活� */
    /* 直��过不需要等�点；真�转�� WAIT，避免远场路口刚识别就硬�� */
    s_ra_ip_row = g_ip_max_row;
    if (straight)
    {
        s_ra_phase = RA_PH_SLOW;
    }
    else
    {
        if ((orig_flag == 1u || orig_flag == 2u) &&
            g_ip_max_row >= RA_VERY_LATE_DIRECT_IP_ROW)
        {
            s_ra_phase = RA_PH_APPROACH;
        }
        else if ((orig_flag == 1u || orig_flag == 2u) &&
                 g_ip_max_row >= RA_LATE_DIRECT_IP_ROW)
        {
            s_ra_phase = RA_PH_SLOW;
        }
        else
        {
            s_ra_phase = RA_PH_WAIT;
        }
    }
    s_ra_straight = straight;               /* 设置直�标� */
    s_ra_post_edge_side = post_edge_side;   /* 设置结束后单边方�?*/
    s_ra_post_edge_ms = post_edge_ms;       /* 设置结束后单边时�?*/
    s_ra_exit_good_cnt = 0u;                /* �出确认�数清�?*/
    s_ra_recover_good_cnt = 0u;             /* 恢�确认�数清零 */
    s_ra_recover_seen_cnt = 0u;
    s_ra_approach_cnt = 0u;                 /* 接近计数清零 */
    s_ra_timer = 0u;                        /* 全局计时器清�?*/
    s_ra_hard_cnt = 0u;                     /* HARD计数清零 */
    s_ra_recover_cnt = 0u;                  /* RECOVER计数清零 */
    s_ra_yaw_lock_cnt = 0u;
    s_ra_phase_cnt = 0u;                    /* 阶��数清零 */
    s_ra_yaw_base = normalize_angle(yaw_angle);
    s_ra_total_yaw_base = s_ra_yaw_base;       /* Item 1: total yaw base, never reset during RA */
    s_ra_hard_yaw_target = 0.0f;
    s_ra_hard_speed_seed = 0.0f;            /* 速度种子清零 */
    s_ra_hard_steer_seed = 0.0f;            /* �向�子清零 */
    s_ra_hard_diff_cmd = 0.0f;
    s_ra_hard_diff_ready = 0u;
    s_ra_exit_last_err = 0.0f;
    s_ra_exit_last_turn = 0.0f;
    s_ra_exit_pd_ready = 0u;
    s_ra_line_takeover_cnt = 0u;
    ra_dbg_line_takeover_ready = 0u;
    ra_dbg_line_takeover_cnt = 0u;
    s_speed_integral *= 0.70f;
    reset_speed_planner();                  /* 复位速度规划�?*/
    lost_search_reset();                    /* 复位丢线搜索 */

    /* Item 8: Check continuous turn before resetting */
    s_continuous_turn_mode = 0u;
    if (s_continuous_turn_remnant_frames > 0u &&
        s_continuous_turn_post_dir == dir &&
        !straight && orig_flag < 3u)
    {
        s_continuous_turn_mode = 1u;
    }
    /* Reset new improved direct turn state */
    visual_reset_stable();
    if (s_continuous_turn_mode == 0u)
        turn_assist_reset();
    s_yaw_guard_active = 0u;
    s_over_turn_guard = 0u;
    s_line_takeover_speed_cap_frames = 0u;
    s_continuous_turn_remnant_frames = 0u;
    s_continuous_turn_post_dir = dir;
    ra_dbg_visual_exit_ready = 0u;
    ra_dbg_yaw_guard_active = 0u;
    ra_dbg_over_turn_guard = 0u;
    ra_dbg_line_takeover_speed_cap = 0u;
    ra_dbg_turn_assist_active = 0u;
    ra_dbg_turn_assist_weight = 0u;
    ra_dbg_turn_assist_found_col = -1;
    ra_dbg_inner_speed_pct = 100u;
    ra_dbg_outer_boost_pct = 0u;
    ra_dbg_continuous_turn_mode = s_continuous_turn_mode;
    ra_dbg_exit_reason_verbose = 0u;
    ra_dbg_inner_min_pct = 100u;

    /* 非直行模式点亮LED指示 */
    if (!straight)                          /* 非直行（�要转�� */
        turn_right_led_on();                /* 点亮右转指示LED */

    ra_debug_update();                      /* 更新调试变量 */
}

/* ra_enter_hard - 进入HARD阶�（����? * 重置yaw角，准�用固定duty驱动��
 * 无参数，无返回�?*/
static void ra_enter_hard(void)             /* 进入HARD���阶�?*/
{
    if (s_ra_phase == RA_PH_HARD)           /* 已经在HARD阶� */
        return;                             /* 避免重�进� */

    s_ra_phase = RA_PH_HARD;
    s_ra_yaw_base = normalize_angle(yaw_angle);                /* 切换到HARD阶� */
    s_ra_phase_cnt = 0u;                    /* 阶��数清零 */
    s_ra_hard_cnt = 0u;                     /* HARD帧�数清�?*/
    s_ra_exit_good_cnt = 0u;                /* �出确认�数清�?*/
    s_ra_recover_good_cnt = 0u;             /* 恢�确认�数清零 */
    s_ra_recover_seen_cnt = 0u;
    s_ra_recover_cnt = 0u;                  /* RECOVER计数清零 */
    s_ra_exit_last_err = 0.0f;
    s_ra_exit_last_turn = 0.0f;
    s_ra_exit_pd_ready = 0u;
    s_ra_line_takeover_cnt = 0u;
    ra_dbg_line_takeover_ready = 0u;
    ra_dbg_line_takeover_cnt = 0u;
    s_ra_yaw_lock_cnt = 0u;
    s_ra_hard_yaw_peak = 0.0f;
    s_ra_hard_diff_cmd = 0.0f;
    s_ra_hard_diff_ready = 0u;
    s_speed_integral *= 0.50f;
    s_ra_pre_turn_ff = 0.0f;
    line_pid_reset_derivative();            /* 重置�分状�?*/

    ra_debug_update();                      /* 更新调试变量 */
}

/* ======================== PID初��?======================== */

/* line_pid_init - 初�化�有PID相关状�（系统�动或重新使能时调��? * 复位：转向PD、�度PI、RA状�机、单边巡线�丢线搜�、路线�数�
 * 无参数，无返回�?*/
void line_pid_init(void)                    /* PID控制器初始化 */
{
    s_steer_last_pos_err = 0.0f;            /* 上�滤波��清零 */
    s_steer_last_raw_err = 0.0f;            /* 上�原始��清零 */
    s_filtered_err = 0.0f;                  /* 滤波��清零 */
    s_prev_steer_output = 0.0f;             /* 上�转向输出清� */
    s_steer_ff_filtered = 0.0f;             /* 前�滤波�清�?*/
    s_cas_target_filtered = 0.0f;           /* 串级�标滤波�清�?*/
    s_cas_last_yaw_err = 0.0f;             /* 串级内环误差清零 */
    s_yaw_bias = 0.0f;                      /* IMU零漂补偿清零 */
    s_steer_d_reset_flag = 1u;              /* 设置�分重�标志 */
    s_speed_integral = 0.0f;               /* 速度�分清� */
    s_cas_pos_integral = 0.0f;
    s_motor_run_counter = 0u;               /* 电机运��数清零 */
    s_vacuum_on = 0u;                       /* 负压状�清�?*/
    s_vacuum_duty = 0u;
    s_vacuum_prearm_ticks = 0u;
    s_vacuum_prearm_timeout = 0u;
    vacuum_enable = 0u;                     /* 负压显示状�清�?*/
    s_rules_done = 0u;                      /* �线完成标志清� */
    s_rules_done_timer = 0u;                /* �线完成�时器清�?*/
    g_ra_pre_slow_flag = 0u;                /* 清除预减速专用标�?*/
    s_pre_lock = 0u;
    s_pre_timeout = 0u;
    single_edge_reset();                    /* 复位单边巡线 */
    s_completed_right_ra_count = 0u;
    lost_search_reset();                    /* 复位丢线搜索 */
    s_lost_last_err = 0;                    /* 丢线�后��清�?*/
    s_lost_search_dir = 1u;                /* 搜索方向默��?*/
    reset_speed_planner();                  /* 复位速度规划�?*/
    reset_speed_ff_state();                 /* 复位速度前� */
    route_debug_reset();                    /* 复位�线调� */

    ra_reset();                             /* 复位RA状�机 */

    /* 清零�有路口类型��?*/
    for (uint8 i = 0u; i < 7u; i++)         /* 遍历�有flag类型�?~6�?*/
        s_inter_count[i] = 0u;             /* 计数清零 */
}

/* ======================== 串级PID�向�算 ======================== */

/* cascade_steer_calc - 串级PID�向�算：图像位�外�?+ IMU角�度内环
 * @pos_err: 位置��
 * @slew_max: 输出变化率限�? * 返回: �向输出�? *
 * 外环�? *   - 对位���做软死区处理（�区内p_scale=0，软区内二�平滑�? *   - 组合：位�� + 趋势前� + 前瞻前� �?�标��度
 *   - �口附近抑制趋�/前瞻前�
 *   - �标��度低�滤�? * 内环�? *   - 角�度�� = �标��度 - 实际yaw_rate
 *   - 内环PD控制（kp/kd由菜单调节）
 *   - 输出限幅 + 变化率限�?*/
static float cascade_steer_calc(int16 pos_err, float kp_scale, float kd_scale, float ff_scale, float slew_max) /* 串级PID�� */
{
    float filter_alpha = s_straight_active ?
                         ERROR_FILTER_STRAIGHT_ALPHA :
                         ERROR_FILTER_ALPHA;
    float raw_err = (float)pos_err;

    s_filtered_err = s_filtered_err * filter_alpha +
                     raw_err * (1.0f - filter_alpha);

    float err     = s_filtered_err;         /* 位置误差 */
    float err_abs = abs_f(err);             /* 位置误差绝对值 */

    /* IMU零漂跟踪: 关闭，防止纠偏角速度被当成零漂吃掉 */
    s_yaw_bias = 0.0f;
    if (g_tf.line_lost != 0u ||
        g_tf.valid_row_count < CAS_INTEGRAL_VALID_ROWS ||
        s_ra_state != RA_ST_NONE ||
        err_abs > CAS_INTEGRAL_ERR_MAX ||
        abs_i16(g_tf.lookahead_error) > CAS_INTEGRAL_LA_MAX)
    {
        s_cas_pos_integral *= CAS_INTEGRAL_DECAY;
        if (abs_f(s_cas_pos_integral) < 0.5f)
            s_cas_pos_integral = 0.0f;
    }
    else if (err_abs > (float)STEER_DEADZONE)
    {
        s_cas_pos_integral += err * CAS_INTEGRAL_GAIN;
        if (s_cas_pos_integral > CAS_INTEGRAL_LIMIT)
            s_cas_pos_integral = CAS_INTEGRAL_LIMIT;
        if (s_cas_pos_integral < -CAS_INTEGRAL_LIMIT)
            s_cas_pos_integral = -CAS_INTEGRAL_LIMIT;
    }

    float aux_scale = 1.0f + ff_scale;
    /* Outer loop: soft deadzone on position error. 外环：软死区处理 */
    float p_scale = 1.0f;                   /* 比例缩放因子，初始为1 */
    if (err_abs <= (float)STEER_DEADZONE)   /* ��在�区� */
    {
        p_scale = 0.0f;                     /* 比例项归�?*/
    }
    else if (err_abs < (float)STEER_SOFT_END) /* ��在软死区范围�?*/
    {
        float t = (err_abs - (float)STEER_DEADZONE) / /* 当前距��区起点 */
                  ((float)STEER_SOFT_END - (float)STEER_DEADZONE); /* 除以�区���?*/
        p_scale = t * t;                    /* 二�曲线平滑过� */
    }

    float yaw_target = CAS_POS_KP  * kp_scale * err * p_scale          /* 位置项：误差×比例 */
                     + 1.5f * s_cas_pos_integral                       /* 积分项：消除稳态误差 */
                     + CAS_TREND_KD * aux_scale * (float)g_tf.error_trend /* 趋势前馈 */
                     + CAS_LA_K    * aux_scale * (float)g_tf.lookahead_error; /* 前瞻前馈 */

    /* Suppress trend/LA feedforward near intersections. �口附近抑制前� */
    if (g_ra_flag != 0u) /* 有RA标志 */
        yaw_target = CAS_POS_KP * kp_scale * err * p_scale; /* 仅保留位�� */

    /* Low-pass filter + clamp. 低�滤�?+ 限幅 */
    yaw_target = clamp_f(yaw_target, -CAS_TARGET_MAX, CAS_TARGET_MAX); /* 限幅到�?20 */
    s_cas_target_filtered = s_cas_target_filtered * CAS_TARGET_FILTER /* 上一帧�?.55 */
                          + yaw_target * (1.0f - CAS_TARGET_FILTER);  /* 新��?.45 */

    /* Inner loop: IMU yaw_rate closed loop. 内环：IMU角�度�� */
    float yaw_err = s_cas_target_filtered - ((float)yaw_rate - s_yaw_bias); /* 角速度误差(去偏) */
    if (abs_f(yaw_err) < CAS_DEADZONE_DPS)  /* ��在�区� */
        yaw_err = 0.0f;                     /* 归零，防�?*/

    float kp_val = (float)yaw_kp * CAS_YAW_KP_SCALE * kd_scale; /* 内环比例增益 */
    float kd_val = (float)yaw_kd * CAS_YAW_KD_SCALE * kd_scale; /* 内环�分�益 */

    float p_out  = kp_val * yaw_err;        /* 内环P�?*/
    float d_out = 0.0f;
    if (s_steer_d_reset_flag == 0u)
        d_out = kd_val * (yaw_err - s_cas_last_yaw_err);
    else
        s_steer_d_reset_flag = 0u;
    s_cas_last_yaw_err = yaw_err;           /* 保存当前��供下�D项使�?*/

    float steer = p_out + d_out;            /* 内环总输�?*/
    steer = clamp_f(steer, -STEER_MAX, STEER_MAX); /* 限幅到�?000 */

    /* Slew rate limit (consistent with original PD). 变化率限�?*/
    float delta = steer - s_prev_steer_output; /* 输出变化�?*/
    if      (delta >  slew_max) steer = s_prev_steer_output + slew_max; /* 正向限幅 */
    else if (delta < -slew_max) steer = s_prev_steer_output - slew_max; /* 负向限幅 */

    s_prev_steer_output = steer;            /* 保存当前输出 */
    s_steer_last_pos_err = err;
    s_steer_last_raw_err = raw_err;
    return steer;                           /* 返回�向�?*/
}

/* ======================== �向PD控制 ======================== */

/* steer_pd_calc - �向PD控制�? * @pos_err: 位置��（负=偏左，�=偏右�? * @kp_scale: 比例增益缩放（由调度器�算�
 * @kd_scale: �分�益缩放（由调度器�算�
 * @feedforward: 前�信号（基于前瞻���
 * @slew_max: 输出变化率限�? * 返回: �向输出�（叠加到�度上形成差速）
 *
 * 逻辑�? *   1. �阶低通滤�位置��? *   2. 死区判断：��很小且无前�时输出0
 *   3. �死区：二次曲线平滑过�
 *   4. P�?= KP * kp_scale * 滤波�� * p_scale
 *   5. D�?= KD * kd_scale * (当前原��� - 上�原始��)
 *   6. 输出 = P + D + 前�
 *   7. 限幅 + 变化率限�?*/
static float steer_pd_calc(int16 pos_err,           /* 位置�� */
                           float kp_scale,           /* 比例增益缩放 */
                           float kd_scale,           /* �分�益缩放 */
                           float feedforward,        /* 前�信� */
                           float slew_max)           /* 变化率限�?*/
{
    float filter_alpha = s_straight_active ?
                         ERROR_FILTER_STRAIGHT_ALPHA :
                         ERROR_FILTER_ALPHA;

    s_filtered_err = s_filtered_err * filter_alpha +
                     (float)pos_err * (1.0f - filter_alpha);

    float err = s_filtered_err;             /* 滤波后的�� */
    float err_abs = abs_f(err);             /* 滤波��绝��?*/
    float raw_err = (float)pos_err;         /* 原���（未滤波�?*/

    /* 死区：��极小且前馈也很小时，直接返�?（防抖） */
    if (err_abs <= (float)STEER_DEADZONE && abs_f(feedforward) <= 1.0f) /* 死区+前�极� */
    {
        s_steer_last_pos_err = err;         /* 保存滤波�� */
        s_steer_last_raw_err = raw_err;     /* 保存原��� */
        s_prev_steer_output *= 0.5f;        /* 衰减上�输出（防�积�� */
        return 0.0f;                        /* 输出0，防�?*/
    }

    /* �死区：�区到软区间用二次曲线平�?*/
    float p_scale = 1.0f;                   /* 比例缩放因子，初始为1 */
    if (err_abs <= (float)STEER_DEADZONE)   /* ��在�区� */
    {
        p_scale = 0.0f;                     /* 比例项归�?*/
    }
    else if (err_abs < (float)STEER_SOFT_END) /* ��在软死区范围�?*/
    {
        float t = (err_abs - (float)STEER_DEADZONE) / /* 当前距��区起点 */
                  ((float)STEER_SOFT_END - (float)STEER_DEADZONE); /* 除以�区���?*/
        p_scale = t * t;                    /* 二�曲线平� */
    }

    /* P项��?*/
    float p_out = STEER_KP * kp_scale * err * p_scale; /* 比例输出 */
    float d_out = 0.0f;                     /* �分输出，初�为0 */

    /* D项：若未�重置则�常计算，否则跳过一�?*/
    if (s_steer_d_reset_flag == 0u)         /* �分未�重置 */
        d_out = STEER_KD * kd_scale * (err - s_steer_last_pos_err) * PID_D_SCALE; /* �分输� */
    else                                    /* �分�重�?*/
        s_steer_d_reset_flag = 0u;          /* 清除重置标志，下帧恢复D�?*/

    s_steer_last_pos_err = err;             /* 保存当前滤波�� */
    s_steer_last_raw_err = raw_err;         /* 保存当前原��� */

    /* 总输�?= P + D + 前� */
    float output = p_out + d_out + feedforward; /* 三项相加 */

    /* 输出限幅 */
    if (output > STEER_MAX) output = STEER_MAX;         /* 正向限幅 */
    else if (output < -STEER_MAX) output = -STEER_MAX;  /* 负向限幅 */

    /* 变化率限制（防�转向突变�?*/
    float delta = output - s_prev_steer_output; /* 输出变化�?*/
    if (delta > slew_max)                   /* 正向变化过大 */
        output = s_prev_steer_output + slew_max; /* 限制正向变化�?*/
    else if (delta < -slew_max)             /* 负向变化过大 */
        output = s_prev_steer_output - slew_max; /* 限制负向变化�?*/

    s_prev_steer_output = output;           /* 保存当前输出 */
    return output;                          /* 返回�向�?*/
}

/* ======================== 速度PI控制 ======================== */

/* speed_pi_calc - 速度PI控制�? * @target: �标�度
 * @actual: 实际速度（左右轮平均�? * @integral: �分指�
 * @pos_err_abs: 位置��绝��（用于�分分离�? * 返回: 速度PI输出
 *
 * �分分离：位置��大时不积分（防�弯道超调）
 * �分限幅：防�积分饱�?*/
static float speed_pi_calc(float target, float actual, float *integral, int16 pos_err_abs) /* 速度PI */
{
    float speed_err = target - actual;      /* 速度�� = �� - 实际 */

    /* �分分离：位置��?< 阈�时才积�?*/
    if (pos_err_abs < SPEED_I_SEPARATION)   /* 位置��小于分�阈�?20) */
    {
        *integral += speed_err * PID_DT_SCALE; /* �加积分�?*/

        /* �分限� */
        if (*integral > SPEED_I_MAX) *integral = SPEED_I_MAX;       /* 正向�分限� */
        else if (*integral < -SPEED_I_MAX) *integral = -SPEED_I_MAX; /* 负向�分限� */
    }
    else
    {
        *integral *= 0.92f;
        if (*integral > -1.0f && *integral < 1.0f)
            *integral = 0.0f;
    }

    return SPEED_KP * speed_err + SPEED_KI * (*integral); /* P�?+ I�?*/
}

/* ======================== ���适应速度 ======================== */

/* calc_adapted_speed - 根据位置���适应调整速度
 * @base: 基�速度
 * @pos_err_abs: 位置��绝��? * 返回: �适应后的速度
 *
 * 逻辑：��小于t1时用ratio_1%，大于t2时用ratio_2%，中间线性插�? * 通常ratio_1 > ratio_2（直道快，弯道慢�?*/
static int16 calc_adapted_speed(int16 base, int16 pos_err_abs) /* ���适应速度 */
{
    int16 t1 = sp_err_t1;                  /* ��阈�下限（菜单�调�?*/
    int16 t2 = sp_err_t2;                  /* ��阈�上限（菜单�调�?*/
    int16 r1 = sp_ratio_1;                 /* 低��速度百分比（菜单�调�?*/
    int16 r2 = sp_ratio_2;                 /* 高��速度百分比（菜单�调�?*/

    if (t2 <= t1)                           /* 阈�无效（上限≤下限） */
        t2 = t1 + 1;                       /* �正为下�?1 */

    if (pos_err_abs <= t1)                  /* ��小于下限 */
        return (int16)((int32)base * r1 / 100); /* 用高百分比（直道�� */

    if (pos_err_abs >= t2)                  /* ��大于上限 */
        return (int16)((int32)base * r2 / 100); /* 用低百分比（�道慢� */

    /* �间线性插�?*/
    int32 ratio = (int32)r1 + ((int32)(r2 - r1) * (pos_err_abs - t1)) / (t2 - t1); /* 线�插�?*/
    return (int16)((int32)base * ratio / 100); /* 用插值百分比 */
}

/* ======================== 直道判定 ======================== */

/* straight_speed_candidate - 判断当前�否为直道加���? * @pos_err_abs: 位置��绝��? * 条件：未丢线、有效�足够���小�前瞻小、趋势小、无RA
 * 返回 1=�直道候�?*/
static uint8 straight_speed_candidate(int16 pos_err_abs) /* 判断直道加���?*/
{
    if (g_tf.valid_row_count < SPEED_STRAIGHT_VALID_ROWS) /* 有效行不�?35) */
        return 0u;                          /* 不是候�?*/
    if (pos_err_abs > SPEED_STRAIGHT_ERR_MAX) /* 位置��过大(>6) */
        return 0u;                          /* 不是候�?*/
    if (abs_i16(g_tf.lookahead_error) > SPEED_STRAIGHT_LOOKAHEAD_MAX) /* 前瞻��过大(>8) */
        return 0u;                          /* 不是候�?*/
    if (abs_i16(g_tf.error_trend) > SPEED_STRAIGHT_TREND_MAX) /* 趋势��过大(>8) */
        return 0u;                          /* 不是候�?*/
    if (g_ra_flag != 0u) /* 有RA标志 */
        return 0u;                          /* 不是候�?*/

    return 1u;                              /* 满足�有条件，�直道候�?*/
}

static uint8 straight_pre_slow_clear_candidate(int16 pos_err_abs)
{
    if (g_tf.line_lost != 0u || g_ra_flag != 0u)
        return 0u;

    if (straight_speed_candidate(pos_err_abs))
        return 1u;

    if (g_tf.valid_row_count >= SPEED_VALID_RUSH_ROWS &&
        pos_err_abs <= SPEED_COMPONENT_ERR_MAX &&
        abs_i16(g_tf.lookahead_error) <= SPEED_COMPONENT_LA_MAX &&
        abs_i16(g_tf.error_trend) <= SPEED_COMPONENT_TREND_MAX)
    {
        return 1u;
    }

    if (g_tf.valid_row_count >= SPEED_VALID_RUN_ROWS &&
        pos_err_abs <= SPEED_STRAIGHT_HOLD_ERR_MAX &&
        abs_i16(g_tf.lookahead_error) <= SPEED_STRAIGHT_HOLD_LOOKAHEAD_MAX &&
        abs_i16(g_tf.error_trend) <= SPEED_STRAIGHT_HOLD_TREND_MAX)
    {
        return 1u;
    }

    return 0u;
}

static uint8 pre_slow_signal_trusted(int16 pos_err_abs)
{
    if (g_ra_pre_flag != 0u &&
        (g_ra_pre_dir == 1u || g_ra_pre_dir == 2u))
    {
        return route_next_flag_is((uint8)g_ra_pre_dir);
    }

    if (g_ra_pre_slow_flag == 0u)
        return 0u;

    if (straight_pre_slow_clear_candidate(pos_err_abs))
        return 0u;

    return 1u;
}
/* sym_straight_speed_candidate - 判断当前�否为对称组件直道加���? * @pos_err_abs: 位置��绝��? * 对称组件（�三极�干扰区）��测到时，说明赛道较直
 * 条件：有对称组件、未丢线、有效�足够���适中、无RA
 * 返回 1=�候�?*/

static uint8 straight_hold_safe(int16 pos_err_abs)
{
    if (g_tf.valid_row_count < SPEED_STRAIGHT_HOLD_VALID_ROWS)
        return 0u;
    if (pos_err_abs > SPEED_STRAIGHT_HOLD_ERR_MAX)
        return 0u;
    if (abs_i16(g_tf.lookahead_error) > SPEED_STRAIGHT_HOLD_LOOKAHEAD_MAX)
        return 0u;
    if (abs_i16(g_tf.error_trend) > SPEED_STRAIGHT_HOLD_TREND_MAX)
        return 0u;
    if (g_ra_flag != 0u)
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
    if (g_sym_component_flag == 0u)         /* ��测到对称组件 */
        return 0u;                          /* 不是候�?*/
    if (g_tf.valid_row_count < SPEED_SYM_VALID_ROWS) /* 有效行不�?30) */
        return 0u;                          /* 不是候�?*/
    if (pos_err_abs > SPEED_SYM_ERR_MAX)   /* 位置��过大(>12) */
        return 0u;                          /* 不是候�?*/
    if (abs_i16(g_tf.lookahead_error) > SPEED_COMPONENT_LA_MAX)
        return 0u;
    if (abs_i16(g_tf.error_trend) > SPEED_COMPONENT_TREND_MAX)
        return 0u;
    if (g_ra_flag != 0u) /* 有RA标志 */
        return 0u;                          /* 不是候�?*/

    return 1u;                              /* 满足�有条件，�候�?*/
}

static uint8 component_fast_speed_candidate(int16 pos_err_abs)
{
    if (g_sym_component_flag == 0u)
        return 0u;
    if (g_tf.valid_row_count < SPEED_COMPONENT_VALID_ROWS)
        return 0u;
    if (pos_err_abs > SPEED_COMPONENT_ERR_MAX)
        return 0u;
    if (abs_i16(g_tf.lookahead_error) > SPEED_COMPONENT_LA_MAX)
        return 0u;
    if (abs_i16(g_tf.error_trend) > SPEED_COMPONENT_TREND_MAX)
        return 0u;
    if (g_ra_flag != 0u)
        return 0u;

    return 1u;
}

/* single_edge_speed_candidate - 单边巡线稳定时允许快速�过
 * 单边用于压住右侧岔路干扰，稳定时应按直道逻辑提�；
 * 真�转弯RA或�减速阶段不加��?*/
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
        if (g_ra_flag != 0u && !route_next_flag_is((uint8)g_ra_flag))
            ra_clear_all_flags();

        if (g_ra_pre_flag != 0u &&
            (g_ra_pre_dir != 1u && g_ra_pre_dir != 2u))
            ra_clear_pre_flags();
        else if (g_ra_pre_flag != 0u &&
                 !route_next_flag_is((uint8)g_ra_pre_dir))
            ra_clear_pre_flags();
    }
    if (s_ra_state == RA_ST_NONE &&
        g_ra_flag != 0u)
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
        if (pos_err_abs > SPEED_SINGLE_EDGE_HOLD_ERR_MAX ||
            abs_i16(g_tf.lookahead_error) > SPEED_SINGLE_EDGE_HOLD_LOOKAHEAD_MAX ||
            abs_i16(g_tf.error_trend) > SPEED_SINGLE_EDGE_HOLD_TREND_MAX)
        {
            s_single_edge_fast_hold = 0u;
            return 0u;
        }
        s_single_edge_fast_hold--;
        return 1u;
    }

    return 0u;
}

/* ======================== 速度斜坡 ======================== */

/* speed_ramp_apply - 基�速度斜坡处理（无调试信息版）
 * 防�目标�度突变，按步长渐变
 * @target: �标�度
 * 返回: 斜坡处理后的速度 */
static int16 speed_ramp_apply(int16 target) /* 速度斜坡处理 */
{
    int16 up_step = (int16)((float)SPEED_RAMP_UP_STEP * PID_DT_SCALE + 0.5f);
    int16 down_step = (int16)((float)SPEED_RAMP_DOWN_STEP * PID_DT_SCALE + 0.5f);

    if (up_step < 1)
        up_step = 1;
    if (down_step < 1)
        down_step = 1;

    if (target < 0)                         /* �标�度为负 */
        target = 0;                         /* 限制�? */

    /* 首�调�（负值表示未初�化），直接跳到��?*/
    if (s_base_speed_ramped < 0)            /* 首�调� */
    {
        s_base_speed_ramped = target;       /* 直接设为�� */
        return s_base_speed_ramped;         /* 返回 */
    }

    /* 按�长渐�?*/
    if (target > s_base_speed_ramped + up_step) /* �要加�?*/
        s_base_speed_ramped += up_step;         /* 按上升�长增�?*/
    else if (target < s_base_speed_ramped - down_step) /* �要减�?*/
        s_base_speed_ramped -= down_step;       /* 按下降�长减�?*/
    else                                    /* �值在步长范围�?*/
        s_base_speed_ramped = target;       /* 直接设为�� */

    return s_base_speed_ramped;             /* 返回斜坡处理后的速度 */
}

/* speed_ramp_apply_reason - 带调试信�的�度斜坡处理
 * @target: �标�度
 * @reason: 速度变化原因编号
 * 直道/单边加�时使用更大的上升��? * 返回: 斜坡处理后的速度 */
static int16 speed_ramp_apply_reason(int16 target, uint8 reason) /* 带原因的斜坡 */
{
    int16 up_step = SPEED_RAMP_UP_STEP;     /* 上升步长 */
    int16 down_step = SPEED_RAMP_DOWN_STEP;

    speed_dbg_plan = target;                /* 记录规划速度 */
    speed_dbg_reason = reason;              /* 记录原因编号 */

    if (target < 0)                         /* �标�度为负 */
        target = 0;                         /* 限制�? */

    /* 直道/单边加�原�?4/6/9)使用更快的上升��?*/
    if (reason == 4u || reason == 6u || reason == 9u)
        up_step = SPEED_RAMP_STRAIGHT_UP_STEP; /* 使用更大的上升��?*/
    if (reason == 5u || reason == 7u || reason == 8u)
        down_step = SPEED_RAMP_SOFT_DOWN_STEP;

    up_step = (int16)((float)up_step * PID_DT_SCALE + 0.5f);
    down_step = (int16)((float)down_step * PID_DT_SCALE + 0.5f);
    if (up_step < 1)
        up_step = 1;
    if (down_step < 1)
        down_step = 1;

    /* 首�调用直接跳到目� */
    if (s_base_speed_ramped < 0)            /* 首�调� */
    {
        s_base_speed_ramped = target;       /* 直接设为�� */
        return s_base_speed_ramped;         /* 返回 */
    }

    /* 按�长渐�?*/
    if (target > s_base_speed_ramped + up_step) /* �要加�?*/
        s_base_speed_ramped += up_step;          /* 按上升�长增�?*/
    else if (target < s_base_speed_ramped - down_step) /* �要减�?*/
        s_base_speed_ramped -= down_step; /* 按下降�长减�?*/
    else                                    /* �值在步长范围�?*/
        s_base_speed_ramped = target;       /* 直接设为�� */

    return s_base_speed_ramped;             /* 返回斜坡处理后的速度 */
}

/* ======================== 速度百分比��?======================== */

/* apply_speed_pct - 将目标�度乘以百分比缩�? * @target: �标�度
 * @pct: 百分比（0~120�?00=不变�? * 返回: 缩放后的速度 */
static int16 apply_speed_pct(int16 target, int16 pct) /* 速度百分比缩�?*/
{
    if (pct < 0) pct = 0;                  /* 百分比下限为0 */
    if (pct > 120) pct = 120;              /* 百分比上限为120 */

    return (int16)((int32)target * pct / 100); /* 计算百分�?*/
}

/* calc_signal_speed_pct - 根据信号值�算降�百分比
 * @signal: 信号值（如前瞻��、趋势���
 * @t1: 信号阈�下限（低于此不降�）
 * @t2: 信号阈�上限（高于此最大降速）
 * @min_pct: �大降速百分比
 * 返回: 速度百分比（100=不降，min_pct=�大降�?*/
static int16 calc_signal_speed_pct(int16 signal, int16 t1, int16 t2, int16 min_pct) /* 信号降�?*/
{
    if (t2 <= t1)                           /* 阈�无�?*/
        t2 = t1 + 1;                       /* �� */

    if (signal <= t1)                       /* 信号低于下限 */
        return 100;                         /* 不降�?*/

    if (signal >= t2)                       /* 信号高于上限 */
        return min_pct;                     /* �大降�?*/

    /* �间线性插�?*/
    return (int16)(100 - ((int32)(100 - min_pct) * (signal - t1)) / (t2 - t1)); /* 线�插�?*/
}

/* ======================== 前瞻/趋势降�?======================== */

/* calc_lookahead_speed_pct - 根据前瞻��和趋势��计算降�百分比
 * @pos_err_abs: 位置��绝��? * 返回: 速度百分比，100=不降�?*/
static int16 calc_lookahead_speed_pct(int16 pos_err_abs) /* 前瞻/趋势降�百分比 */
{
    int16 la_abs = abs_i16(g_tf.lookahead_error); /* 前瞻��绝��?*/
    int16 trend_abs = abs_i16(g_tf.error_trend);   /* 趋势��绝��?*/
    /* 前瞻降�百分比 */
    int16 la_pct = calc_signal_speed_pct(la_abs,          /* 前瞻信号 */
                                         SPEED_LOOKAHEAD_SLOW_T1, /* 前瞻降�下�?*/
                                         SPEED_LOOKAHEAD_SLOW_T2, /* 上限28 */
                                         SPEED_LOOKAHEAD_MIN_PCT); /* 前瞻�低�度百分�?*/
    /* 趋势降�百分比 */
    int16 trend_pct = calc_signal_speed_pct(trend_abs,     /* 趋势信号 */
                                            SPEED_TREND_SLOW_T1,   /* 趋势降�下�?*/
                                            SPEED_TREND_SLOW_T2,   /* 上限24 */
                                            SPEED_TREND_MIN_PCT);  /* 趋势�低�度百分�?*/
    /* 取较小�（降�更多） */
    int16 pct = (la_pct < trend_pct) ? la_pct : trend_pct; /* 取降速更多的 */

    /* ��极大时叠加���适应降�?*/
    if (pos_err_abs > sp_err_t2)            /* ��超过�适应上限 */
        pct = (pct < sp_ratio_2) ? pct : sp_ratio_2; /* 取更小�?*/

    return pct;
}

/* ======================== 视�质量降�?======================== */

/* calc_quality_speed_pct - 根据视�质量�算降�百分比
 * 降�因素：
 *   1. 有效行数�?�?降�（有效行越少，赛道信息越不�靠�? *   2. ��跳变�?�?降�（帧间��突变说明跟踪不稳定）
 * 取两者中降�更多的�? * 返回: 速度百分比，100=不降�?*/
static int16 calc_quality_speed_pct(void) /* 视�质量降速百分比 */
{
    int16 pct = 100;                        /* 初�百分�?00%（不降�） */

    /* 有效行数不足时降�?*/
    if (g_tf.valid_row_count < SPEED_QUALITY_GOOD_ROWS) /* 有效行不�?35) */
    {
        int16 row_pct = SPEED_QUALITY_ROW_MIN_PCT; /* �低百分比(70%) */
        uint16 span = SPEED_QUALITY_GOOD_ROWS - SPEED_VISION_BAD_VALID_ROWS; /* 区间宽度 */

        if (g_tf.valid_row_count <= SPEED_VISION_BAD_VALID_ROWS) /* 有效行极�?�?8) */
        {
            /* 有效行极少，�大降�?*/
            row_pct = SPEED_VISION_BAD_PCT;
        }
        else if (span > 0u)                 /* 区间有效 */
        {
            /* 线�插�?*/
            row_pct = (int16)(SPEED_VISION_BAD_PCT +
                      ((int32)(SPEED_QUALITY_ROW_MIN_PCT - SPEED_VISION_BAD_PCT) *
                      (int32)(g_tf.valid_row_count - SPEED_VISION_BAD_VALID_ROWS)) /
                      (int32)span);          /* 线�插值��?*/
        }

        if (row_pct < pct)                  /* 行降速更�?*/
            pct = row_pct;                  /* 更新百分�?*/
    }

    /* ��跳变�测：当前帧与上帧����?*/
    if (s_prev_quality_err_valid)            /* 上帧��有效 */
    {
        int16 err_jump = abs_i16((int16)(g_tf.error - s_prev_quality_err)); /* ��跳变�?*/
        int16 jump_pct = calc_signal_speed_pct(err_jump,   /* 跳变信号 */
                                               SPEED_ERR_JUMP_T1,   /* 下限10 */
                                               SPEED_ERR_JUMP_T2,   /* 上限28 */
                                               SPEED_ERR_JUMP_MIN_PCT); /* ��?0% */
        if (jump_pct < pct)                 /* 跳变降�更�?*/
            pct = jump_pct;                 /* 更新百分�?*/
    }

    s_prev_quality_err = g_tf.error;        /* 保存当前��供下帧比�?*/
    s_prev_quality_err_valid = 1u;          /* 标�上帧��有效 */
    speed_dbg_vq_pct = (uint8)pct;          /* 记录视�质量百分�?*/

    return pct;
}

/* ======================== �向差速限�?======================== */

/* limit_normal_steer - 限制�向输出（�速）的幅�? * @steer: �向�? * @speed_out: 速度输出�? * �速限�?= 速度输出 * 百分比，再限幅到 [min, max]
 * 不同模式使用不同百分比：正常95%、直�?5%、RECOVER 120%
 * 返回: 限幅后的�向�?*/
static uint8 ra_curve_pid_mode(void);
uint8 ra_curve_yaw_takeover_ready(void)
{
#if RA_CURVE_PID_TURN_ENABLE
    if (!ra_curve_pid_mode())
        return 0u;
    if (imu_ready && !imu_error &&
        ra_yaw_progress() < RA_CURVE_PID_LINE_TAKEOVER_YAW_DEG)
        return 0u;
    return 1u;
#else
    return 0u;
#endif
}

static uint8 ra_curve_line_takeover_active(void)
{
    if (ra_curve_yaw_takeover_ready() == 0u)
        return 0u;
    if (g_tf.line_lost != 0u)
        return 0u;
    if (g_tf.valid_row_count < RA_CURVE_PID_LINE_VALID_ROWS)
        return 0u;
    if (abs_i16(g_tf.error) > RA_CURVE_PID_LINE_ERR_MAX)
        return 0u;
    if (abs_i16(g_tf.lookahead_error) > RA_CURVE_PID_LINE_LA_MAX)
        return 0u;
    if (abs_i16(g_tf.error_trend) > RA_CURVE_PID_LINE_TREND_MAX)
        return 0u;
    return 1u;
}
static int16 ra_curve_inner_pos_err(int16 pos_err)
{
    int32 v = (int32)pos_err;
    int32 bias = (int32)RA_CURVE_INNER_ERR_BIAS;

    if (!ra_curve_pid_mode())
        return pos_err;

    if (s_ra_orig_flag >= 3u &&
        (int32)RA_COMPLEX_CURVE_INNER_ERR_BIAS > bias)
    {
        bias = (int32)RA_COMPLEX_CURVE_INNER_ERR_BIAS;
    }

    if (ra_curve_line_takeover_active())
        bias = bias * (int32)RA_CURVE_PID_TAKEOVER_BIAS_PCT / 100;

    if (s_ra_dir == 1u)
        v -= bias;
    else
        v += bias;

    if (v > 32767) v = 32767;
    if (v < -32768) v = -32768;
    return (int16)v;
}
static float ra_yaw_guard_steer(float steer);

static float limit_normal_steer(float steer, float speed_out)
{
    uint8 curve_mode = ra_curve_pid_mode();
    uint8 curve_hold = (curve_mode && ra_curve_line_takeover_active() == 0u) ? 1u : 0u;
    int16 pct = STEER_DIFF_NORMAL_PCT;

    if (s_straight_active)
        pct = STEER_DIFF_STRAIGHT_PCT;
    if (s_straight_active && g_post_edge_side != EDGE_BOTH)
        pct = STEER_DIFF_SINGLE_EDGE_PCT;

    if (curve_hold)
        pct = STEER_RA_CURVE_DIFF_PCT;

    if (s_ra_state == RA_ST_ACTIVE && s_ra_phase == RA_PH_RECOVER)
        pct = STEER_DIFF_RECOVER_PCT;

    uint8 recover_mode =
        (s_ra_state == RA_ST_ACTIVE && s_ra_phase == RA_PH_RECOVER) ? 1u : 0u;
    float speed_abs = abs_f(speed_out);
    float max_limit = curve_hold ?
                      STEER_RA_CURVE_DIFF_MAX :
                      STEER_DIFF_MAX_LIMIT;
    float limit = speed_abs * (float)pct * 0.01f;

    if (!recover_mode && speed_out <= 0.0f)
        return 0.0f;

    if (!recover_mode && speed_out > 0.0f)
    {
        float no_reverse_limit = speed_out * (curve_hold ? 1.00f : 0.95f);
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

    return clamp_f(steer, -limit, limit);
}

static float line_recenter_steer_boost(float steer)
{
    int16 signal = abs_i16(g_tf.error);
    int16 la_abs = abs_i16(g_tf.lookahead_error);
    int16 trend_abs = abs_i16(g_tf.error_trend);
    float t;

    if (g_tf.line_lost != 0u)
        return steer;

    if (la_abs > signal)
        signal = la_abs;
    if (trend_abs > signal)
        signal = trend_abs;

    t = range_ratio_i16(signal, STEER_RECENTER_T1, STEER_RECENTER_T2);
    if (t <= 0.0f)
        return steer;

    {
        int16 max_pct = (g_post_edge_side != EDGE_BOTH) ?
                        STEER_SINGLE_EDGE_RECENTER_MAX_PCT :
                        STEER_RECENTER_MAX_PCT;
        return steer * lerp_f(1.0f,
                              (float)max_pct * 0.01f,
                              t);
    }
}

static float line_left_bias_steer_trim(float steer)
{
    float max_trim = STEER_LEFT_BIAS_MAX;
    float t;

    if (s_ra_state != RA_ST_NONE)
        return steer;
    if (g_ra_flag != 0u || g_ra_pre_flag != 0u || g_ra_pre_slow_flag != 0u)
        return steer;
    if (g_post_edge_side != EDGE_BOTH)
        return steer;
    if (g_tf.line_lost != 0u ||
        g_tf.valid_row_count < STEER_LEFT_BIAS_VALID_ROWS)
        return steer;
    if (g_tf.error >= -STEER_LEFT_BIAS_ERR_MIN)
        return steer;

    t = range_ratio_i16((int16)(-g_tf.error),
                        STEER_LEFT_BIAS_ERR_MIN,
                        STEER_LEFT_BIAS_ERR_MAX);
    if (route_dbg_step == STEER_LEFT_BIAS_START_ROUTE_STEP)
        max_trim = STEER_LEFT_BIAS_START_MAX;
    return steer - (max_trim * t);
}
static float recover_steer_scale(void)
{
    return (float)RA_RECOVER_STEER_PCT * 0.01f;
}

static uint8 ra_curve_pid_mode(void)
{
#if RA_CURVE_PID_TURN_ENABLE
    return (s_ra_state == RA_ST_ACTIVE &&
            s_ra_straight == 0u &&
            s_ra_phase == RA_PH_APPROACH &&
            (s_ra_dir == 1u || s_ra_dir == 2u)) ? 1u : 0u;
#else
    return 0u;
#endif
}


static float ra_yaw_guard_steer(float steer)
{
    float progress;
    float rate;
    float predicted;
    float over;
    float brake;
    float brake_max;
    float sign;

    if (s_ra_state != RA_ST_ACTIVE ||
        s_ra_straight != 0u ||
        (s_ra_dir != 1u && s_ra_dir != 2u) ||
        (s_ra_phase != RA_PH_SLOW &&
         s_ra_phase != RA_PH_APPROACH &&
         s_ra_phase != RA_PH_RECOVER) ||
        !imu_ready || imu_error)
    {
        return steer;
    }

    progress = ra_yaw_progress();
    rate = ra_yaw_progress_rate();
    predicted = progress + rate * (RA_CURVE_YAW_GUARD_PREDICT_MS * 0.001f);

    if (predicted < RA_CURVE_YAW_GUARD_DEG &&
        progress < RA_CURVE_YAW_GUARD_DEG)
    {
        return steer;
    }

    sign = (s_ra_dir == 1u) ? -1.0f : 1.0f;
    if (steer * sign > 0.0f)
        steer = 0.0f;

    over = predicted - RA_CURVE_YAW_GUARD_DEG;
    if (over < 0.0f)
        over = 0.0f;

    brake_max = RA_CURVE_YAW_BRAKE_MAX;
    if (s_ra_phase == RA_PH_RECOVER &&
        brake_max > RA_RECOVER_YAW_BRAKE_MAX)
    {
        brake_max = RA_RECOVER_YAW_BRAKE_MAX;
    }

    brake = over * RA_CURVE_YAW_BRAKE_KP + rate * RA_CURVE_YAW_BRAKE_KD;
    brake = clamp_f(brake, 0.0f, brake_max);
    steer -= sign * brake;

    return steer;
}

static uint8 ra_hard_exit_reason(uint8 direct_fast,
                                  uint8 min_hard,
                                  uint8 hard_limit,
                                  uint8 line_ok,
                                  float hard_yaw_target,
                                  float yaw_progress,
                                  float yaw_progress_rate)
{
    uint16 force_limit;
    uint16 emergency_limit;

    (void)direct_fast;

    if (s_ra_hard_cnt < min_hard)
        return RA_EXIT_NONE;

    force_limit =
        (uint16)((uint16)hard_limit + (uint16)RA_HARD_FORCE_TIMEOUT_EXTRA);
    emergency_limit =
        (uint16)((uint16)force_limit + (uint16)RA_HARD_EMERGENCY_TIMEOUT_EXTRA);

    if (!imu_ready || imu_error || hard_yaw_target <= 0.0f)
    {
        if (line_ok && s_ra_exit_good_cnt >= RA_EXIT_CONFIRM_FRAMES)
            return RA_EXIT_LINE;
        return (s_ra_hard_cnt >= emergency_limit) ?
               RA_EXIT_NO_IMU : RA_EXIT_NONE;
    }

    if (line_ok &&
        s_ra_exit_good_cnt >= RA_EXIT_CONFIRM_FRAMES)
    {
        if (s_ra_orig_flag < 3u &&
            yaw_progress >= RA_DIRECT_LINE_EXIT_MIN_YAW_DEG)
            return RA_EXIT_LINE;
        if (yaw_progress >= hard_yaw_target - 8.0f)
            return RA_EXIT_LINE;
    }

    if (yaw_progress >= RA_EXIT_FORCE_YAW_DEG)
        return RA_EXIT_YAW;

#if RA_HARD_PREDICT_EXIT_ENABLE
    if (s_ra_orig_flag < 3u)
    {
        float target = hard_yaw_target;
        float yaw_pred;

        if (target > RA_HARD_TARGET_YAW_MAX)
            target = RA_HARD_TARGET_YAW_MAX;
        yaw_pred = yaw_progress + yaw_progress_rate * RA_HARD_PREDICT_TIME_S;
        ra_dbg_yaw_pred10 = (int16)(yaw_pred * 10.0f);
        ra_dbg_yaw_remain10 = (int16)((target - yaw_pred) * 10.0f);
        if (yaw_pred >= target)
            return RA_EXIT_YAW;
    }
#endif

    if (yaw_progress >= hard_yaw_target)
        return RA_EXIT_YAW;

    if (yaw_progress >= hard_yaw_target - RA_HARD_COAST_REMAIN_DEG &&
        yaw_progress +
        yaw_progress_rate * (RA_HARD_YAW_PREDICT_MS * 0.001f) >= hard_yaw_target)
        return RA_EXIT_COAST;

    if (yaw_progress_rate >= RA_HARD_COAST_YAW_RATE &&
        yaw_progress >= hard_yaw_target - RA_HARD_COAST_REMAIN_DEG)
        return RA_EXIT_COAST;

    if (s_ra_hard_cnt >= emergency_limit)
        return RA_EXIT_EMERGENCY;

    if (s_ra_hard_cnt >= force_limit &&
        yaw_progress >= hard_yaw_target - RA_HARD_TIMEOUT_REMAIN_DEG)
        return RA_EXIT_TIMEOUT;

    return RA_EXIT_NONE;
}

/* ======================== 速度规划主函�?======================== */

/* plan_base_speed - 基�速度规划（每PID周期调用�? * @target: 原�目标�度
 * @pos_err_abs: 位置��绝��? * @pre_slow_active: 预减速激活标�? * 返回: 规划后的基�速度（经斜坡处理�? *
 * 规划优先级：
 *   1. RA活跃或�减�?�?直接用目标�度
 *   2. 丢线 �?25%速度
 *   3. 有效行极�?�?40%速度
 *   4. 单边稳定通过 �?120%加�（用于�后T右直行）
 *   5. 元器件直�?�?120%加�（跳过质量/前瞻降�）
 *   6. 视�质量降速（有效�?��跳变�? *   7. 对称组件直道 �?120%加�（立即�? *   8. 前瞻/趋势降�? *   9. 直道�� �?120%加�（�连续N帧确认） */
static int16 plan_base_speed(int16 target, int16 pos_err_abs, uint8 pre_slow_active) /* 速度规划 */
{
    int16 speed_cap = (s_ra_state != RA_ST_NONE ||
                       s_ra_post_recover_cnt > 0u) ?
                      SPEED_BASE_CAP_RA :
                      SPEED_BASE_CAP_NORMAL;

    if (target > speed_cap)
        target = speed_cap;

    s_straight_active = 0u;                 /* 直道�活标志清�?*/
    speed_dbg_vq_pct = 100u;               /* 视�质量百分比重�?*/

    /* RA活跃或�减速期间，不做额��划 */
    if (s_ra_state != RA_ST_NONE) /* RA活跃或�减�?*/
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

        return speed_ramp_apply_reason(target, 1u); /* 直接用目标�度，原�?RA */
    }

    /* Item 7: Line takeover speed cap (higher priority than exit boost) */
    if (s_line_takeover_speed_cap_frames > 0u)
    {
        s_line_takeover_speed_cap_frames--;
        target = apply_speed_pct(target, RA_LINE_TAKEOVER_SPEED_PCT);
        ra_dbg_line_takeover_speed_cap = s_line_takeover_speed_cap_frames;
        s_straight_cnt = 0u;
        s_straight_hold = 0u;
        return speed_ramp_apply_reason(target, 12u);
    }

#if RA_EXIT_BOOST_ENABLE
    if (s_ra_exit_boost_active != 0u)
    {
        if (s_ra_exit_boost_cnt < 255u)
            s_ra_exit_boost_cnt++;

        if (s_ra_exit_boost_cnt <= RA_EXIT_BOOST_DELAY_FRAMES)
        {
            target = apply_speed_pct(target, RA_LINE_TAKEOVER_SPEED_PCT);
        }
        else if (g_tf.line_lost == 0u &&
                 g_tf.valid_row_count >= RA_EXIT_BOOST_VALID_ROWS &&
                 abs_i16(g_tf.error) <= RA_EXIT_BOOST_ERR_MAX)
        {
            target = apply_speed_pct(target, RA_EXIT_BOOST_SPEED_PCT);
            s_ra_exit_boost_active = 0u;
        }
        else
        {
            target = apply_speed_pct(target, RA_LINE_TAKEOVER_SPEED_PCT);
        }

        ra_dbg_exit_boost_active = s_ra_exit_boost_active;
        ra_dbg_exit_boost_cnt = s_ra_exit_boost_cnt;
        s_straight_cnt = 0u;
        s_straight_hold = 0u;
        return speed_ramp_apply_reason(target, 10u);
    }
#endif

    if (s_ra_post_recover_cnt > 0u)
    {
        s_ra_post_recover_cnt--;
        s_straight_cnt = 0u;
        s_straight_hold = 0u;
        target = apply_speed_pct(target, s_ra_post_recover_complex ?
                                 RA_POST_RECOVER_COMPLEX_SPEED_PCT :
                                 RA_POST_RECOVER_SPEED_PCT);
        if (s_ra_post_recover_cnt == 0u)
            s_ra_post_recover_complex = 0u;
        return speed_ramp_apply_reason(target, 1u);
    }

    /* 丢线时大幅降�?*/
    if (g_tf.line_lost != 0u)
    {
        s_straight_cnt = 0u;                /* 直道计数清零 */
        s_straight_hold = 0u;
        speed_dbg_vq_pct = SPEED_LINE_LOST_PCT;
        target = (int16)((int32)target * SPEED_LINE_LOST_PCT / 100);
        return speed_ramp_apply_reason(target, 2u); /* 返回，原�?丢线 */
    }

    /* 有效行极少时降�?*/
    if (g_tf.valid_row_count < SPEED_VISION_BAD_VALID_ROWS) /* 有效行极�?�?8) */
    {
        s_straight_cnt = 0u;                /* 直道计数清零 */
        s_straight_hold = 0u;
        speed_dbg_vq_pct = SPEED_VISION_BAD_PCT;
        target = (int16)((int32)target * SPEED_VISION_BAD_PCT / 100);
        return speed_ramp_apply_reason(target, 3u); /* 返回，原�?视��?*/
    }

    if (pre_slow_active)
    {
        s_straight_cnt = 0u;
        s_straight_hold = 0u;
        return speed_ramp_apply_reason(target, 10u);
    }

    if (route_next_is_complex() &&
        g_ip_max_row >= SPEED_INTER_RISK_IP_ROW &&
        g_tf.valid_row_count < SPEED_INTER_RISK_VALID_ROWS)
    {
        s_straight_cnt = 0u;
        s_straight_hold = 0u;
        speed_dbg_vq_pct = SPEED_INTER_RISK_PCT;
        target = (int16)((int32)target * SPEED_INTER_RISK_PCT / 100);
        return speed_ramp_apply_reason(target, 7u);
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
        pos_err_abs <= SPEED_VALID_RUSH_ERR_MAX &&
        abs_i16(g_tf.lookahead_error) <= SPEED_VALID_RUSH_LA_MAX &&
        abs_i16(g_tf.error_trend) <= SPEED_VALID_RUSH_TREND_MAX &&
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
            pos_err_abs <= SPEED_VALID_RUN_ERR_MAX &&
            abs_i16(g_tf.lookahead_error) <= SPEED_VALID_RUN_LA_MAX &&
            abs_i16(g_tf.error_trend) <= SPEED_VALID_RUN_TREND_MAX &&
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

    return ACT_STRAIGHT;                    /* 其他默�直� */
}

uint8 route_next_turn_dir(uint8 flag)
{
    uint8 action;
    uint8 step = route_next_step_index();

    if (step >= (uint8)USER_RULE_COUNT)
        return 0u;

    const IntersectionRule *rule = &user_rules[step];
    if (flag != 0u && rule->flag != flag)
        return 0u;

    action = (rule->action == ACT_AUTO) ?
             route_action_from_flag(rule->flag) :
             rule->action;

    if (action == ACT_RIGHT) return 1u;
    if (action == ACT_LEFT) return 2u;
    return 0u;
}

uint8 ra_curve_turn_dir(void)
{
#if RA_CURVE_PID_TURN_ENABLE
    if (s_ra_state == RA_ST_ACTIVE &&
        s_ra_straight == 0u &&
        (s_ra_dir == 1u || s_ra_dir == 2u) &&
        s_ra_phase == RA_PH_APPROACH)
    {
        return s_ra_dir;
    }
#endif
    return 0u;
}

uint8 ra_curve_ip_row(void)
{
#if RA_CURVE_PID_TURN_ENABLE
    uint16 row;

    if (ra_curve_turn_dir() == 0u)
        return 0u;

    row = ((uint16)s_ra_ip_row + 1u) / 2u;
    if (row < (uint16)TF_SEARCH_END_ROW)
        row = (uint16)TF_SEARCH_END_ROW;
    if (row >= (uint16)TF_IMG_H)
        row = (uint16)(TF_IMG_H - 1u);

    return (uint8)row;
#else
    return 0u;
#endif
}

uint8 ra_exit_bezier_turn_dir(void)
{
    return (s_ra_state == RA_ST_ACTIVE &&
            s_ra_straight == 0u &&
            s_ra_phase == RA_PH_RECOVER &&
            (s_ra_dir == 1u || s_ra_dir == 2u)) ? s_ra_dir : 0u;
}

uint8 ra_exit_bezier_ip_row(void)
{
    uint16 row = (uint16)RA_EXIT_BEZIER_IP_ROW;

    if (ra_exit_bezier_turn_dir() == 0u)
        return 0u;
    if (row < (uint16)TF_SEARCH_END_ROW)
        row = (uint16)TF_SEARCH_END_ROW;
    if (row >= (uint16)TF_IMG_H)
        row = (uint16)(TF_IMG_H - 1u);

    return (uint8)row;
}

/* fallback_intersection_decision - 保底�口决策（�线表�匹配时的默��为�
 * @flag: �口类型flag
 * 返回: �口决策结构�?*/
static RouteDecision fallback_intersection_decision(uint8 flag) /* 保底�口决� */
{
    RouteDecision d = { ACT_STRAIGHT, EDGE_BOTH, 0u, 0u }; /* 初�化为直行，无单� */

    d.action = route_action_from_flag(flag); /* 根据flag推断动作 */
    return d;                               /* 返回决策 */
}

/* ======================== �线表匹�?======================== */

/* select_intersection_decision - 根据当前flag从路线表�选择动作
 * @flag: 当前�测到的路口类�? * 返回: �口决策（包含动作、单边配�、有效标志）
 *
 * 流程�? *   1. 先查�线表中flag+count匹配的条�? *   2. 匹配�?�?消��条�，返回�应��? *   3. �匹�?�?�查是否启用保底直�? *   4. 都没�?�?返回默�决策（flag=1/2不消耗，flag=3/4/5不动作） */
static RouteDecision select_intersection_decision(uint8 flag)
{
    RouteDecision d = fallback_intersection_decision(flag);
    const IntersectionRule *rule;

    if (flag >= 7u)
        return d;
    if (route_dbg_step >= (uint8)USER_RULE_COUNT)
        return d;

    rule = &user_rules[route_dbg_step];
    if (rule->flag != flag)
        return d;

    s_inter_count[flag] = rule->count;

    d.action = (rule->action == ACT_AUTO) ?
               route_action_from_flag(flag) :
               rule->action;
    d.post_edge_side = rule->post_edge_side;
    d.post_edge_ms = rule->post_edge_ms;
    d.valid = 1u;

    s_route_pending_valid = 1u;
    s_route_pending_flag = flag;
    s_route_pending_count = rule->count;
    s_route_pending_action = d.action;
    return d;
}

static uint8 ra_try_start_from_pre_direct(void)
{
#if RA_PRE_DIRECT_EARLY_ENABLE
    uint8 expected;
    uint8 dir;

    if (s_ra_state != RA_ST_NONE)
        return 0u;

    expected = route_next_expected_flag();
    if (expected != 1u && expected != 2u)
        return 0u;

    dir = (expected == 1u) ? 1u : 2u;

    if (g_ra_pre_flag == 0u)
        return 0u;
    if (g_ra_pre_dir != dir)
        return 0u;
    if (g_tf.valid_row_count < RA_PRE_DIRECT_MIN_VALID_ROWS)
        return 0u;
    if (abs_i16(g_tf.error) > RA_PRE_DIRECT_MAX_ERR)
        return 0u;

    if (s_ra_pre_direct_match_cnt < RA_PRE_DIRECT_MIN_FRAMES &&
        g_ra_pre_slow_flag == 0u)
    {
        return 0u;
    }

    ra_start(dir, expected, 0u, EDGE_AUTO, 0u);
    s_ra_phase = RA_PH_SLOW;
    s_ra_phase_cnt = 0u;
    return 1u;
#else
    return 0u;
#endif
}

static uint8 ra_try_start_direct_flag(void);

static uint8 ra_try_start_route_direct_early_flag(void)
{
#if RA_ROUTE_DIRECT_EARLY_ENABLE
    uint8 expected;
    RouteDecision d;

    if (s_ra_state != RA_ST_NONE || g_ra_flag != 0u)
        return 0u;
    if (ra_speed_ref() < RA_FAST_SPEED_START)
        return 0u;
    if (s_motor_run_counter < (uint32)RA_ROUTE_DIRECT_EARLY_MIN_TICKS)
        return 0u;
#if RA_ROUTE_DIRECT_EARLY_FIRST_ONLY
    if (route_next_step_index() != 0u)
        return 0u;
#endif

    expected = route_next_expected_flag();
    if (expected != 1u && expected != 2u)
        return 0u;

    if (g_tf.line_lost != 0u || g_sym_component_flag != 0u)
        return 0u;
    if (g_ip_max_row != 0u)
        return 0u;
    if (g_tf.valid_row_count < RA_ROUTE_DIRECT_EARLY_VALID_ROWS_MIN ||
        g_tf.valid_row_count > RA_ROUTE_DIRECT_EARLY_VALID_ROWS_MAX)
        return 0u;
    if (abs_i16(g_tf.error) > RA_ROUTE_DIRECT_EARLY_ERR_MAX)
        return 0u;
    if (abs_i16(g_tf.lookahead_error) < RA_ROUTE_DIRECT_EARLY_LA_MIN)
        return 0u;

    d = select_intersection_decision(expected);
    if (!d.valid || (d.action != ACT_RIGHT && d.action != ACT_LEFT))
        return 0u;

    g_ra_flag = expected;
    ra_start((d.action == ACT_RIGHT) ? 1u : 2u,
             expected,
             0u,
             d.post_edge_side,
             d.post_edge_ms);
    if (s_ra_state == RA_ST_ACTIVE && s_ra_straight == 0u)
    {
        s_ra_ip_row = RA_ROUTE_DIRECT_EARLY_IP_ROW;
        ra_enter_hard();
        s_speed_integral = 0.0f;
    }
    ra_pending_complex_clear();
    ra_clear_pre_flags();
    return 1u;
#else
    return 0u;
#endif
}

static uint8 ra_try_start_route_pre_hard_flag(void)
{
#if RA_ROUTE_PRE_HARD_ENABLE
    uint8 flag = 0u;
    uint8 dir_ok = 0u;
    uint8 slow_ok = 0u;
    uint8 row_ok = 0u;

    if (s_ra_state != RA_ST_NONE || g_ra_flag != 0u)
        return 0u;
    if (g_sym_component_flag != 0u)
        return 0u;

    if (route_immediate_flag_is(1u))
        flag = 1u;
    else if (route_immediate_flag_is(2u))
        flag = 2u;
    else
        return 0u;

    if (g_tf.line_lost != 0u)
        return 0u;

    if (g_ra_pre_dir != 0u && g_ra_pre_dir != flag)
        return 0u;

    dir_ok = (g_ra_pre_flag != 0u && g_ra_pre_dir == flag) ? 1u : 0u;
    slow_ok = (g_ra_pre_slow_flag != 0u &&
               (g_ra_pre_dir == 0u || g_ra_pre_dir == flag)) ? 1u : 0u;
    row_ok = (g_ip_max_row >= RA_ROUTE_PRE_HARD_IP_ROW) ? 1u : 0u;

    if (dir_ok == 0u && slow_ok == 0u && row_ok == 0u)
        return 0u;

    if (row_ok == 0u &&
        g_tf.valid_row_count > RA_ROUTE_PRE_HARD_VALID_ROWS &&
        abs_i16(g_tf.lookahead_error) < RA_ROUTE_PRE_HARD_LOOKAHEAD_MIN)
        return 0u;

    g_ra_flag = flag;
    (void)ra_try_start_direct_flag();

    if (s_ra_state == RA_ST_ACTIVE && s_ra_straight == 0u)
    {
        s_ra_ip_row = (g_ip_max_row == 0u) ?
                      RA_PRE_DIRECT_NO_IP_ROW :
                      ra_direct_fixed_turn_trigger_row();
        ra_enter_hard();
        s_speed_integral = 0.0f;
    }

    ra_pending_complex_clear();
    return 0u;
#else
    return 0u;
#endif
}
static uint8 ra_try_start_pre_direct_flag(void)
{
    uint8 flag;
    uint8 turn_row;
    uint8 direct_hard_ready = 0u;
    uint8 pre_direct_row_ready = 0u;

    if (s_ra_state != RA_ST_NONE || g_ra_flag != 0u)
    {
        s_ra_pre_direct_match_cnt = 0u;
        return 0u;
    }
    if (g_ra_pre_flag == 0u ||
        (g_ra_pre_dir != 1u && g_ra_pre_dir != 2u))
    {
        s_ra_pre_direct_match_cnt = 0u;
        return 0u;
    }
#if RA_FIXED_HARD_ROW_ENABLE
    if (g_sym_component_flag != 0u && g_ip_max_row < RA_FIXED_HARD_ROW)
    {
        s_ra_pre_direct_match_cnt = 0u;
        return 0u;
    }
#else
    if (g_sym_component_flag != 0u)
    {
        s_ra_pre_direct_match_cnt = 0u;
        return 0u;
    }
#endif

    flag = g_ra_pre_dir;
    if (!route_next_flag_is(flag))
    {
        s_ra_pre_direct_match_cnt = 0u;
        return 0u;
    }

    if (g_ip_max_row == 0u &&
        g_tf.line_lost == 0u &&
        g_tf.valid_row_count >= RA_PRE_DIRECT_STRAIGHT_VALID_ROWS &&
        abs_i16(g_tf.error) <= RA_PRE_DIRECT_STRAIGHT_ERR_MAX &&
        abs_i16(g_tf.lookahead_error) <= RA_PRE_DIRECT_STRAIGHT_LA_MAX)
    {
        s_ra_pre_direct_match_cnt = 0u;
        return 0u;
    }

    if (flag == 2u &&
        g_ip_max_row == 0u &&
        g_tf.line_lost == 0u &&
        g_tf.valid_row_count >= RA_PRE_DIRECT_LEFT_STRAIGHT_VALID_ROWS &&
        abs_i16(g_tf.error) <= RA_PRE_DIRECT_LEFT_STRAIGHT_ERR_MAX &&
        abs_i16(g_tf.lookahead_error) <= RA_PRE_DIRECT_LEFT_STRAIGHT_LA_MAX)
    {
        s_ra_pre_direct_match_cnt = 0u;
        return 0u;
    }

    if (g_ip_max_row >= RA_PRE_DIRECT_HARD_IP_ROW)
    {
        pre_direct_row_ready = 1u;
    }
#if RA_PRE_DIRECT_NO_IP_ENABLE
    else if (g_ip_max_row == 0u &&
             g_tf.valid_row_count <= RA_PRE_DIRECT_NO_IP_VALID_ROWS &&
             abs_i16(g_tf.lookahead_error) >= RA_PRE_DIRECT_NO_IP_LA_MIN)
    {
        pre_direct_row_ready = 1u;
    }
#endif

    if (g_tf.line_lost == 0u &&
        g_sym_component_flag == 0u &&
        pre_direct_row_ready != 0u)
    {
        if (s_ra_pre_direct_match_cnt < 255u)
            s_ra_pre_direct_match_cnt++;
        if (s_ra_pre_direct_match_cnt >= RA_PRE_DIRECT_HARD_FRAMES)
            direct_hard_ready = 1u;
    }
    else
    {
        s_ra_pre_direct_match_cnt = 0u;
    }

    if (g_post_edge_side != EDGE_BOTH &&
        g_ip_max_row < RA_SINGLE_EDGE_PRE_DIRECT_IP_ROW &&
        (s_edge_age_cnt < (uint16)RA_SINGLE_EDGE_PRE_DIRECT_MIN_TICKS ||
         s_ra_pre_direct_match_cnt < RA_SINGLE_EDGE_PRE_DIRECT_LONG_FRAMES))
    {
        return 0u;
    }

    if (direct_hard_ready)
    {
        s_ra_pre_direct_match_cnt = 0u;
        g_ra_flag = flag;
        if (ra_try_start_direct_flag())
            return 1u;
#if RA_FIXED_HARD_ROW_ENABLE
        if (s_ra_state == RA_ST_ACTIVE && s_ra_straight == 0u &&
            ra_speed_ref() <= RA_PRE_DIRECT_IMMEDIATE_HARD_SPEED_MAX)
        {
            s_ra_ip_row = (g_ip_max_row == 0u) ?
                          RA_PRE_DIRECT_NO_IP_ROW :
                          ra_direct_fixed_turn_trigger_row();
            ra_enter_hard();
            s_speed_integral = 0.0f;
        }
#endif
        return 0u;
    }

    turn_row = ra_slow_trigger_row();

    uint8 row_for_turn = g_ip_max_row;
    /* 普通直角用控制用拐点提前触发（复杂3/4/5不走这条路） */
    if (flag < 3u &&
        g_ip_ctrl_row > row_for_turn &&
        g_ip_ctrl_dir == flag)
    {
        row_for_turn = g_ip_ctrl_row;
    }

    if (row_for_turn < turn_row)
        return 0u;

    g_ra_flag = flag;
    return ra_try_start_direct_flag();
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

static uint8 ra_complex_center_ready(void)
{
    if (!route_immediate_flag_is((uint8)g_ra_flag))
        return 0u;
    if (g_tf.valid_row_count < RA_INTER_COMPLEX_ROUTE_VALID_ROWS)
        return 0u;
    if (abs_i16(g_tf.error) > RA_INTER_COMPLEX_ROUTE_ERR_MAX)
        return 0u;
    if (abs_i16(g_tf.lookahead_error) > RA_INTER_COMPLEX_ROUTE_LA_MAX)
        return 0u;
    if (abs_i16(g_tf.error_trend) > RA_INTER_COMPLEX_ROUTE_TREND_MAX)
        return 0u;
    return 1u;
}

static uint8 ra_complex_last_chance_ready(void)
{
    if (!route_immediate_flag_is((uint8)g_ra_flag))
        return 0u;
    if (g_ip_max_row < RA_INTER_COMPLEX_LAST_CHANCE_ROW)
        return 0u;
    if (g_tf.valid_row_count < RA_INTER_COMPLEX_ROUTE_VALID_ROWS)
        return 0u;
    if (abs_i16(g_tf.error) > RA_INTER_COMPLEX_LAST_ERR_MAX)
        return 0u;
    if (abs_i16(g_tf.lookahead_error) > RA_INTER_COMPLEX_LAST_LA_MAX)
        return 0u;
    if (abs_i16(g_tf.error_trend) > RA_INTER_COMPLEX_LAST_TREND_MAX)
        return 0u;
    return 1u;
}

static uint8 ra_complex_one_frame_ready(void)
{
    uint8 flag = (uint8)g_ra_flag;
    uint8 evidence = g_inter_result.evidence;

    if (flag < 3u || flag > 5u)
        return 0u;
    if (!route_immediate_flag_is(flag))
        return 0u;
    if (g_tf.line_lost != 0u)
        return 0u;
    if (g_ip_max_row < RA_PENDING_COMPLEX_IP_ROW)
        return 0u;
    if (g_tf.valid_row_count < RA_INTER_COMPLEX_START_VALID_ROWS)
        return 0u;
    if (abs_i16(g_tf.error) > RA_INTER_COMPLEX_START_ERR_MAX)
        return 0u;
    if (abs_i16(g_tf.lookahead_error) > RA_INTER_COMPLEX_START_LA_MAX)
        return 0u;
    if (abs_i16(g_tf.error_trend) > RA_INTER_COMPLEX_START_TREND_MAX)
        return 0u;

    if (g_inter_result.detected_type == flag)
        return 1u;
    if (s_ra_pending_complex_cnt > 0u &&
        s_ra_pending_complex_flag == flag)
        return 1u;
    if ((evidence & INTER_EVID_ROUTE) != 0u &&
        (evidence & (INTER_EVID_EDGE |
                     INTER_EVID_FAST |
                     INTER_EVID_CUTOFF)) != 0u &&
        g_inter_result.confidence >= RA_INTER_COMPLEX_CONF_CUTOFF_MIN)
    {
        return 1u;
    }
    return 0u;
}

static uint8 ra_try_start_route_complex_pre_flag(void)
{
#if RA_ROUTE_COMPLEX_PRE_ENABLE
    uint8 expected;
    RouteDecision d;

    if (s_ra_state != RA_ST_NONE || g_ra_flag != 0u)
        return 0u;

    if (!route_next_is_complex())
        return 0u;

    expected = route_next_expected_flag();
    if (expected < 3u || expected > 5u)
        return 0u;

    if (g_tf.line_lost != 0u || g_sym_component_flag != 0u)
        return 0u;
    if (g_ip_max_row >= RA_FIXED_COMPLEX_HARD_ROW)
        return 0u;
    if (g_tf.valid_row_count < RA_ROUTE_COMPLEX_PRE_VALID_ROWS)
        return 0u;
    if (abs_i16(g_tf.error) > RA_ROUTE_COMPLEX_PRE_ERR_MAX)
        return 0u;
    if (abs_i16(g_tf.lookahead_error) < RA_ROUTE_COMPLEX_PRE_LA_MIN)
        return 0u;
    if (abs_i16(g_tf.error_trend) < RA_ROUTE_COMPLEX_PRE_TREND_MIN)
        return 0u;

    d = select_intersection_decision(expected);
    if (!d.valid || (d.action != ACT_RIGHT && d.action != ACT_LEFT))
        return 0u;

    g_ra_flag = expected;
    ra_start((d.action == ACT_RIGHT) ? 1u : 2u,
             expected,
             0u,
             d.post_edge_side,
             d.post_edge_ms);
    if (s_ra_state == RA_ST_ACTIVE && s_ra_straight == 0u)
    {
        s_ra_ip_row = RA_ROUTE_COMPLEX_PRE_IP_ROW;
        ra_enter_hard();
        s_speed_integral = 0.0f;
    }
    ra_pending_complex_clear();
    ra_clear_pre_flags();
    return 1u;
#else
    return 0u;
#endif
}

static uint8 ra_complex_confidence_ready(uint8 last_chance)
{
    uint8 flag = (uint8)g_ra_flag;
    uint8 confidence = g_inter_result.confidence;
    uint8 evidence = g_inter_result.evidence;

    if (flag < 3u || flag > 5u)
        return 1u;

    if (g_inter_result.detected_type != flag &&
        ((evidence & INTER_EVID_ROUTE) == 0u || !route_next_flag_is(flag)))
    {
        return 0u;
    }

    if (confidence >= RA_INTER_COMPLEX_CONF_MIN)
        return 1u;

    if (last_chance != 0u &&
        confidence >= RA_INTER_COMPLEX_CONF_LAST_MIN &&
        (evidence & (INTER_EVID_EDGE | INTER_EVID_ROUTE | INTER_EVID_FAST)) != 0u)
    {
        return 1u;
    }

    if ((evidence & INTER_EVID_CUTOFF) != 0u &&
        confidence >= RA_INTER_COMPLEX_CONF_CUTOFF_MIN)
    {
        return 1u;
    }

    return 0u;
}

static uint8 ra_intersection_start_ready(void)
{
    if (g_ra_flag >= 3u && g_ra_flag <= 5u &&
        route_immediate_flag_is((uint8)g_ra_flag))
    {
        if (ra_complex_start_quality_ready() == 0u)
        {
            s_ra_complex_lost_match_cnt = 0u;
            g_ra_flag = 0u;
            ra_pending_complex_clear();
            return 0u;
        }
#if RA_FIXED_HARD_ROW_ENABLE
        if (ra_complex_predict_ready(ra_complex_row_now()) == 0u)
        {
            s_ra_complex_lost_match_cnt = 0u;
            return 0u;
        }
#endif
        if (ra_pending_complex_start_ok((uint8)g_ra_flag))
        {
            s_ra_complex_lost_match_cnt = 0u;
            return 1u;
        }
        if (ra_complex_one_frame_ready())
        {
            s_ra_complex_lost_match_cnt = 0u;
            return 1u;
        }

        if (ra_complex_center_ready() &&
            ra_complex_confidence_ready(0u))
        {
            s_ra_complex_lost_match_cnt = 0u;
            return 1u;
        }
        if (ra_complex_last_chance_ready() &&
            ra_complex_confidence_ready(1u))
        {
            s_ra_complex_lost_match_cnt = 0u;
            return 1u;
        }
#if RA_INTER_COMPLEX_LOST_FALLBACK_ENABLE
        if (g_ip_max_row >= RA_INTER_COMPLEX_LOST_IP_ROW &&
            (g_tf.line_lost != 0u ||
             g_tf.valid_row_count < RA_INTER_COMPLEX_ROUTE_VALID_ROWS))
        {
            if (s_ra_complex_lost_match_cnt < 255u)
                s_ra_complex_lost_match_cnt++;
            if (s_ra_complex_lost_match_cnt >=
                RA_INTER_COMPLEX_LOST_FRAMES)
            {
                if (ra_complex_confidence_ready(1u))
                {
                    s_ra_complex_lost_match_cnt = 0u;
                    return 1u;
                }
            }
        }
        else
        {
            s_ra_complex_lost_match_cnt = 0u;
        }
#endif
        return 0u;
    }
    s_ra_complex_lost_match_cnt = 0u;
    if (abs_i16(g_tf.error) > RA_INTER_START_ERR_MAX)
        return 0u;
    if (abs_i16(g_tf.lookahead_error) > RA_INTER_START_LA_MAX)
        return 0u;
    if (abs_i16(g_tf.error_trend) > RA_INTER_START_TREND_MAX)
        return 0u;
    if (g_tf.valid_row_count >= RA_INTER_START_VALID_ROWS)
        return 1u;
    if (route_immediate_flag_is((uint8)g_ra_flag) &&
        g_tf.valid_row_count >= RA_INTER_ROUTE_VALID_ROWS &&
        abs_i16(g_tf.lookahead_error) <= RA_INTER_ROUTE_LA_MAX &&
        abs_i16(g_tf.error_trend) <= RA_INTER_ROUTE_TREND_MAX)
    {
        return 1u;
    }

    if (s_ra_post_recover_cnt > 0u &&
        g_ra_flag >= 3u && g_ra_flag <= 5u &&
        (ra_complex_center_ready() || ra_complex_last_chance_ready()))
    {
        return 1u;
    }
    return 0u;
}

static uint8 ra_try_start_intersection_flag(void)
{
    if (s_ra_state == RA_ST_NONE &&
        (g_ra_flag == 0u || !route_next_flag_is((uint8)g_ra_flag)) &&
        s_ra_pending_complex_cnt > 0u &&
        route_next_flag_is(s_ra_pending_complex_flag))
    {
        g_ra_flag = s_ra_pending_complex_flag;
    }

    if (ra_try_start_route_complex_pre_flag())
        return 0u;

    if ((g_ra_flag < 3u || g_ra_flag > 5u) || s_ra_state != RA_ST_NONE)
        return 0u;

    if (!ra_intersection_start_ready())
    {
        if (!route_next_flag_is((uint8)g_ra_flag))
            g_ra_flag = 0u;
        ra_debug_update();
        return 1u;
    }

    if (g_post_edge_side != EDGE_BOTH &&
        !route_next_flag_is((uint8)g_ra_flag))
    {
        ra_clear_all_flags();
        ra_debug_update();
        return 1u;
    }

    uint8 start_ip_row = g_ip_max_row;
    RouteDecision d = select_intersection_decision((uint8)g_ra_flag);

    if (s_ra_pending_complex_cnt > 0u &&
        s_ra_pending_complex_flag == (uint8)g_ra_flag &&
        s_ra_pending_complex_ip_row > start_ip_row)
    {
        start_ip_row = s_ra_pending_complex_ip_row;
    }

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
        if (start_ip_row > s_ra_ip_row)
            s_ra_ip_row = start_ip_row;
#if RA_FIXED_HARD_ROW_ENABLE
        if (s_ra_state == RA_ST_ACTIVE &&
            s_ra_straight == 0u &&
            s_ra_orig_flag >= 3u &&
            ra_complex_predict_ready(s_ra_ip_row) != 0u)
        {
            ra_enter_hard();
            s_speed_integral = 0.0f;
        }
#endif
        ra_pending_complex_clear();
        ra_clear_pre_flags();
    }
    else
    {
        ra_start(0u,
                 (uint8)g_ra_flag,
                 1u,
                 d.post_edge_side,
                 d.post_edge_ms);
        ra_pending_complex_clear();
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

static void ra_output_yaw_lock_drive(void);

static uint8 ra_handle_curve_pid_turn(RaResult *r)
{
#if RA_CURVE_PID_TURN_ENABLE
    float target;
    float progress;
    uint8 yaw_done = 0u;
    uint8 line_done = 0u;
    uint8 timeout_done = 0u;
    uint8 complex_turn = (s_ra_orig_flag >= 3u) ? 1u : 0u;
    float exit_remain = complex_turn ?
                        RA_COMPLEX_CURVE_PID_EXIT_REMAIN_DEG :
                        RA_CURVE_PID_EXIT_REMAIN_DEG;
    uint16 max_frames = complex_turn ?
                        RA_COMPLEX_CURVE_PID_MAX_FRAMES :
                        RA_CURVE_PID_MAX_FRAMES;
    int16 curve_speed_pct = complex_turn ?
                            RA_COMPLEX_CURVE_PID_SPEED_PCT :
                            RA_CURVE_PID_SPEED_PCT;
    uint16 curve_frames;
    float line_exit_yaw = complex_turn ?
                          RA_COMPLEX_CURVE_PID_LINE_EXIT_MIN_YAW_DEG :
                          RA_CURVE_PID_LINE_EXIT_MIN_YAW_DEG;
    float timeout_min_yaw = complex_turn ?
                            RA_COMPLEX_CURVE_PID_TIMEOUT_MIN_YAW_DEG :
                            RA_CURVE_PID_TIMEOUT_MIN_YAW_DEG;
    float min_exit_yaw;

    if (s_ra_state != RA_ST_ACTIVE ||
        s_ra_straight != 0u ||
        (s_ra_dir != 1u && s_ra_dir != 2u) ||
        s_ra_phase != RA_PH_APPROACH)
    {
        return 0u;
    }

#if !RA_COMPLEX_CURVE_PID_ENABLE
    if (s_ra_orig_flag >= 3u)
        return 0u;
#endif

    if (s_ra_approach_cnt == 0u)
    {
        s_ra_hard_cnt = 0u;
        s_ra_exit_good_cnt = 0u;
        s_ra_recover_good_cnt = 0u;
        s_ra_recover_seen_cnt = 0u;
        s_speed_integral *= 0.85f;
        s_steer_d_reset_flag = 1u;
    }

    s_ra_approach_cnt++;
    ra_clear_all_flags();
    curve_frames = s_ra_approach_cnt;

    target = ra_hard_target_limit((float)ra_hard_yaw);
    s_ra_hard_yaw_target = target;
    ra_dbg_hard_target10 = (int16)(target * 10.0f);

    min_exit_yaw = line_exit_yaw;
    if (timeout_min_yaw > min_exit_yaw)
        min_exit_yaw = timeout_min_yaw;
    if (target - exit_remain > min_exit_yaw)
        min_exit_yaw = target - exit_remain;
    if (min_exit_yaw > target)
        min_exit_yaw = target;

    progress = ra_yaw_progress();
    if (imu_ready && !imu_error &&
        curve_frames >= (uint16)RA_CURVE_PID_MIN_FRAMES &&
        progress >= min_exit_yaw)
    {
        yaw_done = 1u;
    }

    if (curve_frames >= (uint16)RA_CURVE_PID_LINE_EXIT_MIN_FRAMES &&
        g_tf.line_lost == 0u &&
        g_tf.valid_row_count >= RA_CURVE_PID_LINE_VALID_ROWS &&
        abs_i16(g_tf.error) <= RA_CURVE_PID_LINE_ERR_MAX &&
        abs_i16(g_tf.lookahead_error) <= RA_CURVE_PID_LINE_LA_MAX &&
        abs_i16(g_tf.error_trend) <= RA_CURVE_PID_LINE_TREND_MAX &&
        ((!imu_ready || imu_error) ||
         progress >= min_exit_yaw))
    {
        line_done = 1u;
    }

    if (curve_frames >= max_frames)
    {
        if (!imu_ready || imu_error || progress >= min_exit_yaw)
        {
            timeout_done = 1u;
        }
    }

    if (yaw_done != 0u || line_done != 0u || timeout_done != 0u)
    {
        ra_dbg_exit_reason = (line_done != 0u) ? RA_EXIT_LINE :
                             ((yaw_done != 0u) ? RA_EXIT_YAW : RA_EXIT_TIMEOUT);
        s_ra_hard_speed_seed = (float)speed_dbg_out;
        s_ra_hard_steer_seed = s_prev_steer_output;
        ra_enter_recover();
        r->speed_scale = (float)RA_RECOVER_SPEED_PCT * 0.01f;
        r->need_pid = 1u;
        ra_debug_update();
        return 1u;
    }

    r->speed_scale = (float)curve_speed_pct * 0.01f;
    r->need_pid = 1u;
    ra_debug_update();
    return 1u;
#else
    (void)r;
    return 0u;
#endif
}
static uint8 ra_recover_line_seen(void)
{
    return (g_tf.line_lost == 0u &&
            g_tf.valid_row_count >= RA_RECOVER_SEEN_ROWS) ? 1u : 0u;
}

static uint8 ra_recover_line_visible(void)
{
    return (g_tf.line_lost == 0u &&
            g_tf.valid_row_count >= RA_RECOVER_VALID_ROWS) ? 1u : 0u;
}

static uint8 ra_recover_line_stable(uint8 recover_visible)
{
    if (recover_visible == 0u)
        return 0u;
    if (imu_ready && !imu_error &&
        ra_yaw_progress() < RA_RECOVER_STABLE_MIN_YAW_DEG)
        return 0u;
    if (abs_i16(g_tf.error) > RA_RECOVER_ERROR_MAX)
        return 0u;
    if (abs_i16(g_tf.lookahead_error) > RA_RECOVER_LOOKAHEAD_MAX)
        return 0u;
    if (abs_i16(g_tf.error_trend) > RA_RECOVER_TREND_MAX)
        return 0u;
    return 1u;
}

static float ra_exit_visual_base_duty(void)
{
    float base = (float)ra_speed_ref() *
                 ((float)RA_RECOVER_SPEED_PCT * 0.01f);
    base *= ra_voltage_comp_scale();
    return clamp_f(base, RA_EXIT_VIS_DUTY_MIN, RA_EXIT_VIS_DUTY_MAX);
}

static uint8 ra_recover_visual_allowed(uint8 recover_seen,
                                       uint8 recover_visible,
                                       uint8 recover_stable)
{
    if (recover_seen == 0u)
        return 0u;
    if (s_ra_recover_seen_cnt < RA_RECOVER_SEEN_CONFIRM_FRAMES)
        return 0u;
    if (recover_stable != 0u)
        return 1u;
    if (!imu_ready || imu_error)
        return 1u;
    if (ra_yaw_progress() >= RA_EXIT_VIS_MIN_YAW_DEG)
        return 1u;
    return (recover_visible != 0u &&
            g_tf.valid_row_count >= RA_EXIT_VIS_STRONG_ROWS) ? 1u : 0u;
}

static uint8 ra_recover_yaw_guard_active(void)
{
    float target;
    float progress;
    float rate;

    if (!imu_ready || imu_error ||
        s_ra_straight != 0u ||
        (s_ra_dir != 1u && s_ra_dir != 2u))
    {
        return 0u;
    }

    target = s_ra_hard_yaw_target;
    if (target <= 1.0f)
        target = ra_hard_target_limit((float)ra_hard_yaw);

    progress = ra_yaw_progress();
    rate = ra_yaw_progress_rate();

    if (progress >= target - 5.0f)
        return 1u;
    return (rate > RA_YAW_LOCK_RATE_DONE) ? 1u : 0u;
}

static float ra_recover_block_same_dir_turn(float turn)
{
    if (ra_recover_yaw_guard_active() == 0u)
        return turn;

    if (s_ra_dir == 1u && turn < 0.0f)
        return 0.0f;
    if (s_ra_dir == 2u && turn > 0.0f)
        return 0.0f;
    return turn;
}

static void ra_output_recover_visual_drive(void)
{
    float err = (float)g_tf.error;
    float derr = 0.0f;
    float turn;
    float delta;
    float base;
    float out_l;
    float out_r;

    if (s_ra_exit_pd_ready != 0u)
        derr = err - s_ra_exit_last_err;
    else
        s_ra_exit_pd_ready = 1u;

    turn = STEER_KP * RA_EXIT_VIS_KP_SCALE * err +
           STEER_KD * RA_EXIT_VIS_KD_SCALE * derr * PID_D_SCALE;
    turn = clamp_f(turn, -RA_EXIT_VIS_TURN_MAX, RA_EXIT_VIS_TURN_MAX);

    turn = ra_recover_block_same_dir_turn(turn);

    delta = turn - s_ra_exit_last_turn;
    if (delta > RA_EXIT_VIS_SLEW_MAX)
        turn = s_ra_exit_last_turn + RA_EXIT_VIS_SLEW_MAX;
    else if (delta < -RA_EXIT_VIS_SLEW_MAX)
        turn = s_ra_exit_last_turn - RA_EXIT_VIS_SLEW_MAX;

    base = ra_exit_visual_base_duty();
    {
        float max_turn = base * ((float)RA_EXIT_VIS_TURN_PCT * 0.01f);
        if (turn > max_turn) turn = max_turn;
        else if (turn < -max_turn) turn = -max_turn;
    }
    out_l = base + turn;
    out_r = base - turn;

    if (out_l < 0.0f) out_l = 0.0f;
    if (out_r < 0.0f) out_r = 0.0f;

    s_ra_exit_last_err = err;
    s_ra_exit_last_turn = turn;
    s_prev_steer_output = turn;
    speed_dbg_out = (int16)((out_l + out_r) * 0.5f);
    steer_dbg_out = (int16)((out_l - out_r) * 0.5f);
    pid_set_duty(clamp_duty(out_l), clamp_duty(out_r));
}

static void ra_output_recover_lost_drive(void)
{
    float outer = RA_EXIT_LOST_OUTER_DUTY;
    float inner = RA_EXIT_LOST_INNER_DUTY;
    float out_l;
    float out_r;

    if (ra_recover_yaw_guard_active() != 0u)
    {
        ra_output_yaw_lock_drive();
        return;
    }

    if (outer > MAX_DUTY)
        outer = MAX_DUTY;
    if (inner < 0.0f)
        inner = 0.0f;

    {
        float volt_scale = ra_voltage_comp_scale();
        outer *= volt_scale;
        inner *= volt_scale;
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

    s_ra_exit_pd_ready = 0u;
    s_ra_exit_last_turn = (out_l - out_r) * 0.5f;
    s_prev_steer_output = s_ra_exit_last_turn;
    speed_dbg_out = (int16)((out_l + out_r) * 0.5f);
    steer_dbg_out = (int16)s_ra_exit_last_turn;
    ra_dbg_outer_cmd = (s_ra_dir == 1u) ? clamp_duty(out_r) : clamp_duty(out_l);
    pid_set_duty(clamp_duty(out_l), clamp_duty(out_r));
}

static uint8 ra_handle_recover_phase(RaResult *r)
{
    uint8 recover_seen;
    uint8 recover_visible;
    uint8 recover_stable;
    uint8 keep_flag;

    if (s_ra_phase != RA_PH_RECOVER)
        return 0u;

    s_ra_recover_cnt++;
    recover_seen = ra_recover_line_seen();
    recover_visible = ra_recover_line_visible();
    recover_stable = ra_recover_line_stable(recover_visible);

    if (recover_seen != 0u)
    {
        if (s_ra_recover_seen_cnt < 255u)
            s_ra_recover_seen_cnt++;
    }
    else
    {
        s_ra_recover_seen_cnt = 0u;
    }

    if (recover_stable != 0u)
    {
        if (s_ra_recover_good_cnt < 255u)
            s_ra_recover_good_cnt++;
    }
    else
    {
        s_ra_recover_good_cnt = 0u;
    }

    keep_flag = ra_keep_next_route_flag((uint8)g_ra_flag);
    if (keep_flag == 0u)
        ra_clear_flags_keep_pending();
    else
        ra_clear_pre_flags();

    if (keep_flag == 0u &&
        s_ra_pending_complex_cnt > 0u &&
        s_ra_recover_cnt >= ((s_ra_pending_complex_bridge != 0u) ?
                             RA_PENDING_COMPLEX_BRIDGE_RECOVER_FRAMES :
                             RA_RECOVER_CHAIN_COMPLEX_MIN_FRAMES) &&
        route_next_flag_is(s_ra_pending_complex_flag) &&
        (s_ra_pending_complex_bridge != 0u || recover_seen != 0u))
    {
        uint8 next_flag = s_ra_pending_complex_flag;
        uint8 next_ip_row = s_ra_pending_complex_ip_row;
        uint8 next_conf = s_ra_pending_complex_conf;
        uint8 next_bridge = s_ra_pending_complex_bridge;
        ra_dbg_exit_reason = RA_EXIT_RECOVER;
        ra_finish_ex(next_flag);
        s_ra_pending_complex_flag = next_flag;
        s_ra_pending_complex_cnt = RA_PENDING_COMPLEX_HOLD_FRAMES;
        s_ra_pending_complex_ip_row = next_ip_row;
        s_ra_pending_complex_conf = next_conf;
        s_ra_pending_complex_bridge = next_bridge;
        (void)ra_try_start_intersection_flag();
        r->should_return = 1u;
        return 1u;
    }

    if (s_ra_recover_good_cnt >= RA_RECOVER_CONFIRM_FRAMES)
    {
        ra_dbg_exit_reason = RA_EXIT_RECOVER;
        ra_finish_ex(keep_flag);
        return 1u;
    }

    if (ra_recover_visual_allowed(recover_seen,
                                  recover_visible,
                                  recover_stable) != 0u)
    {
        ra_output_recover_visual_drive();
    }
    else
    {
        ra_output_recover_lost_drive();
    }

    if (s_ra_recover_cnt >= RA_RECOVER_MAX_FRAMES)
    {
        ra_dbg_exit_reason = RA_EXIT_TIMEOUT;
        ra_finish_ex(keep_flag);
        return 1u;
    }

    r->should_return = 1u;
    ra_debug_update();
    return 1u;
}

static void ra_output_yaw_lock_drive(void)
{
    float base = (float)ra_speed_ref() *
                 ((float)RA_YAW_LOCK_SPEED_PCT * 0.01f);
    float brake = 0.0f;
    float out_l;
    float out_r;

    base *= ra_voltage_comp_scale();
    base = clamp_f(base, RA_YAW_LOCK_DUTY_MIN, RA_YAW_LOCK_DUTY_MAX);

    if (imu_ready && !imu_error)
    {
        float rate = ra_yaw_progress_rate();
        if (rate > RA_YAW_LOCK_BRAKE_RATE)
        {
            brake = (rate - RA_YAW_LOCK_BRAKE_RATE) *
                    RA_YAW_LOCK_BRAKE_KD;
            brake = clamp_f(brake, 0.0f, RA_YAW_LOCK_BRAKE_MAX);
            if (brake > base)
                brake = base;
        }
    }

    if (s_ra_dir == 1u)
    {
        out_l = base + brake;
        out_r = base - brake;
    }
    else
    {
        out_l = base - brake;
        out_r = base + brake;
    }

    if (out_l < 0.0f) out_l = 0.0f;
    if (out_r < 0.0f) out_r = 0.0f;

    speed_dbg_out = (int16)((out_l + out_r) * 0.5f);
    steer_dbg_out = (int16)((out_l - out_r) * 0.5f);
    pid_set_duty(clamp_duty(out_l), clamp_duty(out_r));
}

static uint8 ra_handle_yaw_lock_phase(RaResult *r)
{
    float lock_target;
    float progress;
    float rate;
    uint8 lock_done;

    if (s_ra_phase != RA_PH_YAW_LOCK)
        return 0u;

    s_ra_yaw_lock_cnt++;
    ra_clear_flags_keep_pending();

    lock_target = s_ra_hard_yaw_target + RA_YAW_LOCK_EXTRA_DEG;
    if (lock_target > RA_YAW_LOCK_FINAL_DEG)
        lock_target = RA_YAW_LOCK_FINAL_DEG;

    progress = ra_yaw_progress();
    rate = ra_yaw_progress_rate();
    lock_done = ((imu_ready && !imu_error &&
                  progress >= lock_target - RA_YAW_LOCK_TARGET_MARGIN_DEG &&
                  rate <= RA_YAW_LOCK_RATE_DONE) ||
                 s_ra_yaw_lock_cnt >= RA_YAW_LOCK_FRAMES) ? 1u : 0u;

    ra_output_yaw_lock_drive();
    r->should_return = 1u;

    if (lock_done)
        ra_enter_recover();
    else
        ra_debug_update();

    return 1u;
}

static uint8 ra_is_real_turn(void)
{
    return (s_ra_straight == 0u &&
            (s_ra_dir == 1u || s_ra_dir == 2u)) ? 1u : 0u;
}

static uint8 ra_step_wait_slow_approach(RaResult *r)
{
    if (ra_is_real_turn())
    {
        if ((s_ra_phase == RA_PH_WAIT ||
             s_ra_phase == RA_PH_SLOW ||
             s_ra_phase == RA_PH_APPROACH) &&
            s_ra_timer >= RA_REAL_TURN_FORCE_HARD_FRAMES)
        {
            ra_enter_hard();
            return 0u;
        }
    }
    if (s_ra_phase == RA_PH_WAIT)
    {
        uint8 slow_row = ra_slow_trigger_row();
        uint8 turn_row = ra_turn_trigger_row();
        uint16 wait_limit = (s_ra_orig_flag >= 3u) ?
                            RA_CROSS_WAIT_TIMEOUT :
                            RA_WAIT_TIMEOUT;
        uint8 wait_timeout = (s_ra_phase_cnt >= wait_limit) ? 1u : 0u;
        uint16 late_row = (uint16)turn_row + RA_LATE_APPROACH_SKIP_ROW_MARGIN;

        if (s_ra_straight == 0u &&
            (s_ra_dir == 1u || s_ra_dir == 2u) &&
            g_ra_pre_flag != 0u &&
            g_ra_pre_dir == s_ra_dir &&
            ra_speed_ref() <= RA_PRE_DIRECT_IMMEDIATE_HARD_SPEED_MAX)
        {
            if (s_ra_orig_flag < 3u)
            {
                s_ra_ip_row = ra_direct_fixed_turn_trigger_row();
                ra_enter_hard();
                s_speed_integral = 0.0f;
                return 0u;
            }
            s_ra_phase = RA_PH_APPROACH;
            s_ra_approach_cnt = 0u;
            s_ra_phase_cnt = 0u;
            s_speed_integral = 0.0f;
        }
        else if (s_ra_straight == 0u &&
            (s_ra_dir == 1u || s_ra_dir == 2u) &&
            (uint16)s_ra_ip_row >= late_row)
        {
            s_ra_phase = RA_PH_APPROACH;
            s_ra_approach_cnt = 0u;
            s_ra_phase_cnt = 0u;
            s_speed_integral = 0.0f;
        }
        else if (s_ra_ip_row >= slow_row || wait_timeout)
        {
            s_ra_phase = RA_PH_SLOW;
            s_ra_phase_cnt = 0u;
            s_speed_integral = 0.0f;
        }
    }
    else if (s_ra_phase == RA_PH_SLOW)
    {
        uint8 slow_row = ra_slow_trigger_row();
        uint8 turn_row = ra_turn_trigger_row();
        uint8 fallback_frames = (s_ra_orig_flag >= 3u) ?
                                RA_COMPLEX_SLOW_TO_APPROACH_FALLBACK_FRAMES :
                                RA_SLOW_TO_APPROACH_FALLBACK_FRAMES;

        uint8 row_for_turn = s_ra_ip_row;

        if (s_ra_orig_flag < 3u &&
            g_ip_ctrl_row > row_for_turn &&
            g_ip_ctrl_dir == s_ra_dir)
        {
            row_for_turn = g_ip_ctrl_row;
        }

        if (row_for_turn >= turn_row)
        {
            if (s_ra_orig_flag < 3u)
            {
                s_ra_ip_row = ra_direct_fixed_turn_trigger_row();
                ra_enter_hard();
                s_speed_integral = 0.0f;
                return 0u;
            }
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

        if (s_ra_orig_flag < 3u)
        {
            s_ra_ip_row = ra_direct_fixed_turn_trigger_row();
            ra_enter_hard();
            s_speed_integral = 0.0f;
            return 0u;
        }

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

static int16 ra_dynamic_yaw_target(void)
{
    int16 yaw = ra_hard_yaw;

    if (s_ra_straight == 0u && (s_ra_dir == 1u || s_ra_dir == 2u))
    {
        if (yaw > RA_OVERSHOOT_YAW_LIMIT)
            yaw = RA_OVERSHOOT_YAW_LIMIT;
    }

    if (yaw < RA_REAL_TURN_MIN_YAW)
        yaw = RA_REAL_TURN_MIN_YAW;

    return yaw;
}

static int16 ra_dynamic_outer_cmd(void)
{
    int32 outer = ra_hard_outer;

    if (s_ra_straight == 0u && (s_ra_dir == 1u || s_ra_dir == 2u))
    {
        if (s_ra_ip_row >= RA_VERY_LATE_DIRECT_IP_ROW)
            outer = outer * RA_VERY_LATE_OUTER_PCT / 100;
        else
            outer = outer * RA_OVERSHOOT_OUTER_PCT / 100;
    }

    if (outer < RA_REAL_TURN_MIN_OUTER)
        outer = RA_REAL_TURN_MIN_OUTER;

    return (int16)outer;
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
                     pos_err_abs <= RA_EXIT_ERROR_MAX &&
                     abs_i16(g_tf.lookahead_error) <= RA_EXIT_LOOKAHEAD_MAX &&
                     abs_i16(g_tf.error_trend) <= RA_EXIT_TREND_MAX) ? 1u : 0u;
    float hard_yaw_target =
        ra_hard_target_limit((float)ra_dynamic_yaw_target() +
                             (direct_fast ? RA_FAST_DIRECT_YAW_OFFSET : 0.0f));
    s_ra_hard_yaw_target = hard_yaw_target;
    ra_dbg_hard_target10 = (int16)(hard_yaw_target * 10.0f);
    float yaw_progress;
    float outer;
    float inner;
    float inner_keep;
    float inner_min;
    float out_l;
    float out_r;
    float yaw_progress_rate;
    uint8 ramp_frames;
    uint8 direct_inner_reverse_allowed = 0u;
    float direct_inner_reverse_limit = 0.0f;

    s_ra_hard_cnt++;

    if (line_ok)
        s_ra_exit_good_cnt++;
    else
        s_ra_exit_good_cnt = 0u;

    yaw_progress = ra_yaw_progress();
    yaw_progress_rate = ra_yaw_progress_rate();

    /* 检测yaw绕圈: 如果yaw从峰值掉了30°以上 → 强制退出, 防180° */
    if (yaw_progress > s_ra_hard_yaw_peak)
        s_ra_hard_yaw_peak = yaw_progress;
    else if (s_ra_hard_yaw_peak > 55.0f && s_ra_hard_yaw_peak - yaw_progress > 55.0f)
    {
        ra_dbg_exit_reason = RA_EXIT_EMERGENCY;
        ra_enter_yaw_lock();
        (void)ra_handle_yaw_lock_phase(r);
        return 1u;
    }

    /* ========== Item 4/5: Visual-first exit checks for direct turns ========== */
    if (s_ra_orig_flag < 3u && s_ra_straight == 0u)
    {
        float yaw_total = ra_total_yaw_progress();
        float yaw_rate_abs = ra_abs_yaw_rate();
        uint8 vis_ok;
        uint8 vis_strict_ok;
        uint8 vis_very_strict_ok;

        turn_assist_apply();  /* Item 3: apply turn assist line */

        vis_ok = visual_exit_ready(0u);
        vis_strict_ok = visual_exit_ready(1u);
        vis_very_strict_ok = visual_exit_ready(2u);  /* very strict = use_strict=2 means very-strict path below */

        /* Update debug */
        ra_dbg_visual_exit_ready = vis_ok;
        ra_dbg_yaw_total_progress10 = (int16)(yaw_total * 10.0f);
        ra_dbg_yaw_hard_progress10 = (int16)(yaw_progress * 10.0f);

        /* Item 5 Priority 5: over-turn guard */
        if (yaw_total > RA_TOTAL_YAW_OVERSHOOT || yaw_rate_abs > RA_YAW_RATE_OVER_LIMIT)
        {
            s_over_turn_guard = 1u;
            ra_dbg_over_turn_guard = 1u;
            ra_dbg_exit_reason_verbose = RA_EXIT_VERBOSE_OVERTURN;
            ra_dbg_exit_reason = RA_EXIT_EMERGENCY;
            ra_enter_recover();
            r->speed_scale = (float)RA_RECOVER_SPEED_PCT * 0.01f;
            r->should_return = 1u;
            ra_output_recover_lost_drive();
            ra_debug_update();
            return 1u;
        }

        /* Item 5 Priority 4: yaw < 45° guard */
        if (yaw_total < RA_TOTAL_YAW_EXIT_LOW)
        {
            s_yaw_guard_active = 1u;
            ra_dbg_yaw_guard_active = 1u;
            ra_dbg_exit_reason_verbose = RA_EXIT_VERBOSE_YAW_GUARD;

            /* Only allow finish if very strict visual confirms */
            if (vis_very_strict_ok || (vis_strict_ok && s_visual_strict_cnt >= RA_VISUAL_EXIT_VERY_STRICT_FRAMES))
            {
                /* Visual very stable despite low yaw - allow line takeover */
                ra_dbg_exit_reason = RA_EXIT_LINE;
                ra_finish_by_line_takeover();
                return 1u;
            }
            /* Otherwise continue HARD, don't check normal exit logic */
        }
        else
        {
            s_yaw_guard_active = 0u;
            ra_dbg_yaw_guard_active = 0u;
        }

        /* Item 5 Priority 1/2/3: visual-first exit */
        if (vis_ok && yaw_total >= RA_TOTAL_YAW_EXIT_LOW && !s_yaw_guard_active)
        {
            if (yaw_total >= RA_TOTAL_YAW_EXIT_MIN)
            {
                /* Priority 1: normal line takeover */
                ra_dbg_exit_reason = RA_EXIT_LINE;
                ra_finish_by_line_takeover();
                return 1u;
            }
            else
            {
                /* Priority 2: speed-limited line takeover */
                ra_dbg_exit_reason_verbose = RA_EXIT_VERBOSE_VISUAL_LOW_YAW;
                ra_dbg_exit_reason = RA_EXIT_LINE;
                s_line_takeover_speed_cap_frames = (uint8)((uint16)RA_TAKEOVER_SPEED_CAP_FRAMES + 2u);
                ra_dbg_line_takeover_speed_cap = s_line_takeover_speed_cap_frames;
                ra_finish_by_line_takeover();
                return 1u;
            }
        }

        /* Item 5 Priority 3: yaw_guard active - don't allow normal finish */
        if (s_yaw_guard_active)
        {
            /* Skip standard exit_reason check, continue HARD */
            goto direct_hard_continue;
        }

        /* Normal exit_reason check for direct turns (only when yaw_total >= 65 or visual ok) */
        {
            uint8 exit_reason = ra_hard_exit_reason(direct_fast,
                                                      min_hard,
                                                      hard_limit,
                                                      line_ok,
                                                      hard_yaw_target,
                                                      yaw_progress,
                                                      yaw_progress_rate);
            if (exit_reason != RA_EXIT_NONE)
            {
                uint8 need_yaw_lock = 0u;

                ra_dbg_exit_reason = exit_reason;
                if (exit_reason == RA_EXIT_EMERGENCY ||
                    exit_reason == RA_EXIT_TIMEOUT ||
                    exit_reason == RA_EXIT_NO_IMU ||
                    exit_reason == RA_EXIT_RA_TO ||
                    exit_reason == RA_EXIT_COAST)
                {
                    need_yaw_lock = 1u;
                }

                if (need_yaw_lock)
                {
                    /* Timeout with yaw < 65: go to YAW_LOCK */
                    if ((exit_reason == RA_EXIT_TIMEOUT || exit_reason == RA_EXIT_EMERGENCY) &&
                        yaw_total < RA_TOTAL_YAW_EXIT_LOW)
                    {
                        ra_enter_yaw_lock();
                        (void)ra_handle_yaw_lock_phase(r);
                    }
                    else
                    {
                        ra_enter_yaw_lock();
                        (void)ra_handle_yaw_lock_phase(r);
                    }
                }
                else if (exit_reason == RA_EXIT_YAW && yaw_total >= RA_TOTAL_YAW_EXIT_LOW)
                {
                    ra_enter_recover();
                    r->speed_scale = (float)RA_RECOVER_SPEED_PCT * 0.01f;
                    r->should_return = 1u;
                    ra_debug_update();
                }
                else
                {
                    ra_enter_recover();
                    r->speed_scale = (float)RA_RECOVER_SPEED_PCT * 0.01f;
                    if (exit_reason == RA_EXIT_LINE)
                        ra_output_recover_visual_drive();
                    else
                        ra_output_recover_lost_drive();
                    r->should_return = 1u;
                    ra_debug_update();
                }
                return 1u;
            }
        }
    }
    else
    {
        /* Complex intersections (3/4/5): existing logic unchanged */
        uint8 exit_reason = ra_hard_exit_reason(direct_fast,
                                                  min_hard,
                                                  hard_limit,
                                                  line_ok,
                                                  hard_yaw_target,
                                                  yaw_progress,
                                                  yaw_progress_rate);
        if (exit_reason != RA_EXIT_NONE)
        {
            uint8 need_yaw_lock = 0u;

            ra_dbg_exit_reason = exit_reason;
            if (exit_reason == RA_EXIT_EMERGENCY ||
                exit_reason == RA_EXIT_TIMEOUT ||
                exit_reason == RA_EXIT_NO_IMU ||
                exit_reason == RA_EXIT_RA_TO ||
                exit_reason == RA_EXIT_COAST)
            {
                need_yaw_lock = 1u;
            }
            else if (exit_reason == RA_EXIT_YAW &&
                     imu_ready && !imu_error &&
                     yaw_progress_rate > RA_YAW_LOCK_RATE_DONE)
            {
                need_yaw_lock = 1u;
            }

            if (need_yaw_lock)
            {
                ra_enter_yaw_lock();
                (void)ra_handle_yaw_lock_phase(r);
            }
            else
            {
                ra_enter_recover();
                r->speed_scale = (float)RA_RECOVER_SPEED_PCT * 0.01f;
                if (exit_reason == RA_EXIT_LINE)
                    ra_output_recover_visual_drive();
                else
                    ra_output_recover_lost_drive();
                r->should_return = 1u;
                ra_debug_update();
            }
            return 1u;
        }
    }

direct_hard_continue:

    if (s_ra_orig_flag < 3u)
    {
        outer = (float)ra_fast_hard_outer_cmd();
    }
    else
    {
        outer = (float)ra_dynamic_outer_cmd() *
                (float)(direct_fast ? RA_FAST_HARD_OUTER_PCT : RA_HARD_OUTER_PCT) * 0.01f;
        ra_dbg_hard_outer_dynamic = (int16)outer;
        ra_dbg_outer_scale = 100u;
    }
    inner = (float)ra_hard_inner;
    if (inner < 0.0f)
        inner = 0.0f;
    if (inner > MAX_DUTY)
        inner = MAX_DUTY;

    if (s_ra_orig_flag < 3u && s_ra_hard_cnt <= RA_DIRECT_FAST_REVERSE_FRAMES)
    {
        inner = -RA_DIRECT_FAST_REVERSE_DUTY;
        direct_inner_reverse_allowed = 1u;
        direct_inner_reverse_limit = RA_DIRECT_FAST_REVERSE_DUTY;
        if (outer > RA_DIRECT_ENTRY_OUTER_DUTY_MAX)
            outer = RA_DIRECT_ENTRY_OUTER_DUTY_MAX;
    }

    if (s_ra_orig_flag >= 3u)
    {
        float inner_reverse = (float)ra_hard_inner *
                              (float)RA_COMPLEX_INNER_REVERSE_PCT * 0.01f;
        if (inner_reverse < RA_COMPLEX_INNER_REVERSE_MIN_DUTY)
            inner_reverse = RA_COMPLEX_INNER_REVERSE_MIN_DUTY;
        if (inner_reverse > RA_COMPLEX_INNER_REVERSE_MAX_DUTY)
            inner_reverse = RA_COMPLEX_INNER_REVERSE_MAX_DUTY;
        inner = -inner_reverse;
        outer = outer * (float)RA_COMPLEX_DUTY_PCT * 0.01f;
        if (outer < RA_COMPLEX_PIVOT_OUTER_MIN_DUTY)
            outer = RA_COMPLEX_PIVOT_OUTER_MIN_DUTY;
    }
    else if (outer < RA_PIVOT_OUTER_MIN_DUTY)
    {
        outer = RA_PIVOT_OUTER_MIN_DUTY;
    }

    ra_hard_apply_rate_control(direct_fast,
                               hard_yaw_target,
                               yaw_progress,
                               yaw_progress_rate,
                               &outer,
                               &inner);

    if (s_ra_orig_flag < 3u &&
        s_ra_hard_cnt <= RA_DIRECT_FAST_REVERSE_FRAMES &&
        direct_inner_reverse_allowed != 0u)
    {
        inner = -direct_inner_reverse_limit;
        if (outer > RA_DIRECT_ENTRY_OUTER_DUTY_MAX)
            outer = RA_DIRECT_ENTRY_OUTER_DUTY_MAX;
    }

    if (s_ra_orig_flag >= 3u && outer > RA_COMPLEX_PIVOT_OUTER_MAX_DUTY)
        outer = RA_COMPLEX_PIVOT_OUTER_MAX_DUTY;

    if (outer > MAX_DUTY)
        outer = MAX_DUTY;

    /* Item 6: Diff-limit for direct turns (inner wheel floor, outer ceiling) */
    if (s_ra_orig_flag < 3u && s_ra_straight == 0u)
    {
        float inner_min_pct = (float)ra_direct_diff_inner_pct() * 0.01f;
        float outer_boost_pct = (float)ra_direct_diff_outer_boost_pct() * 0.01f;
        float inner_min_duty = outer * inner_min_pct;
        float nominal_outer = (float)ra_fast_hard_outer_cmd();
        float outer_ceiling = nominal_outer * (1.0f + outer_boost_pct);
        if (outer_ceiling < RA_PIVOT_OUTER_MIN_DUTY)
            outer_ceiling = (float)RA_PIVOT_OUTER_MIN_DUTY;
        if (outer > outer_ceiling)
            outer = outer_ceiling;
        if (!ra_direct_reverse_allowed() && inner < 0.0f)
            inner = 0.0f;
        if (inner >= 0.0f && inner < inner_min_duty)
            inner = inner_min_duty;
        else if (inner < 0.0f && (-inner) > inner_min_duty)
            inner = -inner_min_duty;
        ra_dbg_inner_min_pct = (uint8)(inner_min_pct * 100.0f);
        ra_dbg_outer_boost_pct = (uint8)(outer_boost_pct * 100.0f);
    }

    if (s_ra_dir == 1u)
    {
        out_l = inner;
        out_r = outer;
        inner_keep = inner;
    }
    else
    {
        out_l = outer;
        out_r = inner;
        inner_keep = inner;
    }

#if RA_HARD_LINE_TRIM_ENABLE
    if (g_tf.line_lost == 0u &&
        g_tf.valid_row_count >= RA_HARD_LINE_VALID_ROWS)
    {
        float line_trim =
            (float)g_tf.error * RA_HARD_LINE_ERR_KP +
            (float)g_tf.lookahead_error * RA_HARD_LINE_LA_KP;
        line_trim = clamp_f(line_trim,
                            -RA_HARD_LINE_TRIM_MAX,
                            RA_HARD_LINE_TRIM_MAX);
        out_l += line_trim;
        out_r -= line_trim;
    }
#endif

    {
    if (s_ra_orig_flag < 3u &&
        g_tf.valid_row_count <= 8u &&
        yaw_progress >= 30.0f &&
        yaw_progress < hard_yaw_target - 15.0f)
    {
        inner = -RA_DIRECT_FAST_REVERSE_DUTY;
        outer = 1500.0f;
        direct_inner_reverse_allowed = 1u;
        if (direct_inner_reverse_limit < RA_DIRECT_FAST_REVERSE_DUTY)
            direct_inner_reverse_limit = RA_DIRECT_FAST_REVERSE_DUTY;
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
    }

        float volt_scale = ra_voltage_comp_scale();
        out_l *= volt_scale;
        out_r *= volt_scale;
        inner_keep *= volt_scale;
    }
    inner_min = inner_keep;

    ramp_frames = direct_fast ? RA_FAST_HARD_RAMP_FRAMES : RA_HARD_RAMP_FRAMES;
    if (ramp_frames < 1u)
        ramp_frames = 1u;

    if (s_ra_hard_cnt <= ramp_frames)
    {
        float ramp = (float)s_ra_hard_cnt / (float)ramp_frames;
        float ramp_min = direct_fast ? 0.55f : 0.35f;
        if (ramp < ramp_min)
            ramp = ramp_min;
        out_l *= ramp;
        out_r *= ramp;
        inner_min *= ramp;
    }

    if (s_ra_dir == 1u)
    {
        if (out_l < inner_min)
            out_l = inner_min;
    }
    else
    {
        if (out_r < inner_min)
            out_r = inner_min;
    }

    if (s_ra_orig_flag >= 3u)
    {
        if (s_ra_dir == 1u)
        {
            if (out_l < -RA_COMPLEX_INNER_REVERSE_MAX_DUTY)
                out_l = -RA_COMPLEX_INNER_REVERSE_MAX_DUTY;
            if (out_r < 0.0f) out_r = 0.0f;
        }
        else
        {
            if (out_l < 0.0f) out_l = 0.0f;
            if (out_r < -RA_COMPLEX_INNER_REVERSE_MAX_DUTY)
                out_r = -RA_COMPLEX_INNER_REVERSE_MAX_DUTY;
        }
    }
    else
    {
        if (direct_inner_reverse_allowed != 0u)
        {
            if (s_ra_dir == 1u)
            {
                if (out_l < -direct_inner_reverse_limit)
                    out_l = -direct_inner_reverse_limit;
                if (out_r < 0.0f)
                    out_r = 0.0f;
            }
            else
            {
                if (out_l < 0.0f)
                    out_l = 0.0f;
                if (out_r < -direct_inner_reverse_limit)
                    out_r = -direct_inner_reverse_limit;
            }
        }
        else
        {
            if (out_l < 0.0f) out_l = 0.0f;
            if (out_r < 0.0f) out_r = 0.0f;
        }
    }

    if (s_ra_orig_flag >= 3u)
    {
        if (s_ra_dir == 1u)
        {
            if (out_r > RA_COMPLEX_PIVOT_OUTER_MAX_DUTY)
                out_r = RA_COMPLEX_PIVOT_OUTER_MAX_DUTY;
        }
        else
        {
            if (out_l > RA_COMPLEX_PIVOT_OUTER_MAX_DUTY)
                out_l = RA_COMPLEX_PIVOT_OUTER_MAX_DUTY;
        }
    }
    if (out_l > MAX_DUTY) out_l = MAX_DUTY;
    if (out_r > MAX_DUTY) out_r = MAX_DUTY;

    ra_dbg_outer_cmd = (s_ra_dir == 1u) ? clamp_duty(out_r) : clamp_duty(out_l);
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
    if (s_ra_phase == RA_PH_WAIT && s_ra_orig_flag < 3u)
    {
        int16 pct = RA_PRE_SLOW_PCT;
        int16 ref = ra_speed_ref();
        if (ref > RA_FAST_SPEED_START && pct > 25)
        {
            int16 extra = (int16)(((int32)(ref - RA_FAST_SPEED_START) * 35) /
                                   (int32)(1700 - RA_FAST_SPEED_START));
            if (extra > 35) extra = 35;
            pct -= extra;
        }
        r->speed_scale = (float)pct * 0.01f;
    }
    else if (s_ra_phase == RA_PH_WAIT && s_ra_orig_flag >= 3u)
    {
        r->speed_scale = (float)RA_COMPLEX_WAIT_SPEED_PCT * 0.01f;
    }
    else if (s_ra_phase == RA_PH_SLOW)
    {
        int16 pct = ra_slow_pct;
        /* 高速时自动压低减速比例，让高速弯和低速弯入口速度接近 */
        int16 ref = ra_speed_ref();
        if (ref > RA_FAST_SPEED_START && pct > 25)
        {
            int16 extra = (int16)(((int32)(ref - RA_FAST_SPEED_START) * 35) /
                                   (int32)(1700 - RA_FAST_SPEED_START));
            if (extra > 35) extra = 35;
            pct -= extra;
        }
        if (pct < 10)
            pct = 10;
        if (pct > 100)
            pct = 100;
        r->speed_scale = (float)pct * 0.01f;
    }
    else if (s_ra_phase == RA_PH_APPROACH)
    {
        r->speed_scale = (float)RA_APPROACH_SPEED_PCT * 0.01f;
    }
}

/* ======================== RA状�机主�辑 ======================== */

/* ra_state_machine_step - RA状�机每帧执�
 * @pos_err_abs: 位置��绝��? * 返回: RaResult（need_pid/should_return/speed_scale�? *
 * 状�转��
 *   �测到flag(1/2) �?从路线表选动�?�?ra_start()
 *   �测到flag(3/4/5) �?从路线表选动�?�?ra_start()
 *   WAIT阶� �?等待拐点行达到阈�?�?SLOW
 *   SLOW阶� �?等待拐点行达到转�阈�?�?APPROACH
 *   APPROACH阶� �?等待N�?�?HARD
 *   HARD阶� �?固定duty驱动 �?摄像头恢�?yaw达标/超时 �?RECOVER
 *   RECOVER阶� �?PID恢�巡� �?�认稳� �?结束 */
static RaResult ra_state_machine_step(int16 pos_err_abs) /* RA状�机主�辑 */
{
    RaResult r = { 0u, 0u, 1.0f };          /* 初�化：需要PID，不跳过，�度缩放100% */

    if (s_ra_state == RA_ST_ACTIVE &&
        s_ra_orig_flag >= 1u &&
        s_ra_orig_flag <= 2u &&
        s_ra_phase != RA_PH_RECOVER &&
        g_ra_flag >= 3u &&
        g_ra_flag <= 5u)
    {
        g_ra_flag = 0u;
    }

    ra_pending_complex_update();

    if (ra_try_start_route_direct_early_flag())
        return r;

    if (ra_try_start_route_pre_hard_flag())
        return r;

    if (ra_try_start_pre_direct_flag())
        return r;

    if (ra_try_start_direct_flag())
        return r;

    if (ra_try_start_intersection_flag())
        return r;

    /* RA���?�?返回默��?*/
    if (s_ra_state != RA_ST_ACTIVE)          /* RA���?*/
    {
        ra_debug_update();                   /* 更新调试 */
        return r;                            /* 返回默�结� */
    }

    ra_tick_active();

    if (ra_handle_timeout())
        return r;

    if (ra_handle_straight_phase(&r))
        return r;

    if (s_ra_phase == RA_PH_HARD ||
        s_ra_phase == RA_PH_YAW_LOCK ||
        s_ra_phase == RA_PH_RECOVER)
    {
        if (ra_line_takeover_ready())
        {
            ra_finish_by_line_takeover();
            return r;
        }
    }

    if (ra_handle_yaw_lock_phase(&r))
        return r;

    if (ra_handle_recover_phase(&r))
        return r;

    if (ra_handle_curve_pid_turn(&r))
        return r;

    if (ra_step_wait_slow_approach(&r))
        return r;

    if (ra_handle_hard_phase(pos_err_abs, &r))
        return r;

    ra_apply_slow_scale(&r);

    r.need_pid = 1u;                         /* �要PID控制 */
    ra_debug_update();                       /* 更新调试 */
    return r;                                /* 返回结果 */
}

/* ======================== 正常PID执� ======================== */

/* normal_pid_step - 正常巡线PID控制（非HARD阶�时调用�
 * @pos_err: 位置��
 * @pos_err_abs: 位置��绝��? *
 * 流程�? *   1. 计算�向�益调度参数
 *   2. 计算前�信号（基于前瞻���
 *   3. 选择cascade或普通PD计算��
 *   4. 计算�适应速度 + 速度PI + 速度前�
 *   5. 速度因子缩放�向（高�时温和补偿�? *   6. 直道/RECOVER模式�向缩�
 *   7. �速限�? *   8. 输出：左�?速度+�向，右�?速度-�� */
static void normal_pid_step(int16 pos_err, int16 pos_err_abs) /* 正常PID控制 */
{
    uint8 curve_mode = ra_curve_pid_mode();
    uint8 curve_takeover = (curve_mode && ra_curve_line_takeover_active()) ? 1u : 0u;

    if (curve_mode && curve_takeover == 0u)
    {
        pos_err = ra_curve_inner_pos_err(pos_err);
        pos_err_abs = abs_i16(pos_err);
    }

    /* 计算增益调度参数 */
    SteerSchedule sch = steer_schedule_calc(pos_err_abs); /* 根据速度和弯道�算增�?*/

    if (curve_mode && curve_takeover == 0u)
    {
        sch.kp_scale *= (float)STEER_RA_CURVE_KP_PCT * 0.01f;
        sch.kd_scale *= (float)STEER_RA_CURVE_KD_PCT * 0.01f;
        sch.ff_scale *= (float)STEER_RA_CURVE_FF_PCT * 0.01f;
        sch.slew_max *= (float)STEER_RA_CURVE_SLEW_PCT * 0.01f;
    }
    /* 前�信号：KP * ff_scale * 前瞻�� */
    float ff_raw = STEER_KP * sch.ff_scale * (float)g_tf.lookahead_error; /* 原�前� */
    float steer_ff = 0.0f;                  /* 有效前��，初��? */

    ff_raw = clamp_f(ff_raw, -STEER_FF_MAX, STEER_FF_MAX); /* 前�限� */
    /* 前�低通滤�?*/
    s_steer_ff_filtered = s_steer_ff_filtered * STEER_FF_FILTER_ALPHA + /* 上一帧�?.55 */
                          ff_raw * (1.0f - STEER_FF_FILTER_ALPHA);      /* 新��?.45 */

    /* �口附近抑制前� */
    if (g_ra_flag == 0u) /* 无RA标志 */
        steer_ff = s_steer_ff_filtered;     /* 使用滤波后的前� */

    float steer;                            /* �向输出�?*/

    /* 选择�向控制模式：cascade（IMU内环）或�通PD */
    if (cascade_en && imu_ready && !imu_error && /* 串级启用且IMU正常 */
        !(s_ra_state == RA_ST_ACTIVE && s_ra_phase == RA_PH_RECOVER)) /* 且不在RECOVER阶段 */
    {
        steer = cascade_steer_calc(pos_err,
                                   sch.kp_scale,
                                   sch.kd_scale,
                                   sch.ff_scale,
                                   sch.slew_max); /* 使用串级PID */
    }
    else                                    /* 使用�通PD */
    {
        steer = steer_pd_calc(pos_err,      /* 位置�� */
                              sch.kp_scale, /* 比例增益缩放 */
                              sch.kd_scale, /* �分�益缩放 */
                              steer_ff,     /* 前�信� */
                              sch.slew_max); /* 变化率限�?*/

#if YAW_COMP_ENABLE                         /* yaw角补偿（默�关��?*/
        /* yaw角补偿（默�关��?*/
        {
            float yaw_kp_val = (float)yaw_kp / 10.0f; /* yaw比例增益 */
            float yaw_abs = abs_f(yaw_angle);          /* yaw角绝对�?*/
            if (yaw_abs > YAW_DEADZONE)                 /* 超过死区(1�? */
                steer += yaw_kp_val * yaw_angle;        /* 添加yaw补偿 */
        }
#endif

        /* Independent yaw damping (cascade_en=0 only). �立yaw阻尼 */
        if (yaw_damp_gain != 0 && imu_ready && !imu_error && /* 阻尼增益非零且IMU正常 */
            !(s_ra_state == RA_ST_ACTIVE && s_ra_phase == RA_PH_RECOVER)) /* 不在RECOVER */
        {
            steer -= (float)yaw_damp_gain * YAW_DAMP_SCALE * (float)yaw_rate; /* 减去阻尼�?*/
        }
    }

    /* ===== 速度计算 ===== */
    int16 trend_abs = abs_i16(g_tf.error_trend); /* 趋势��绝��?*/
    int16 speed_err = pos_err_abs;          /* 速度��基准 = 位置�� */
    uint8 component_fast = component_fast_speed_candidate(pos_err_abs);

    /* 高�时将趋势��叠加到�度���（高速弯道更谨慎�?*/
    if (base_speed > 200)                   /* 速度超过200 */
    {
        float trend_factor = (float)(base_speed - 200) / 800.0f; /* 趋势因子 */
        if (trend_factor > 0.5f) trend_factor = 0.5f; /* 上限50% */
        speed_err = pos_err_abs + (int16)((float)trend_abs * trend_factor); /* 叠加趋势 */
    }

    /* �适应速度�� */
    uint8 valid_run_fast =
        (g_tf.line_lost == 0u &&
         g_tf.valid_row_count >= SPEED_VALID_RUN_ROWS &&
         pos_err_abs <= SPEED_VALID_RUN_ERR_MAX &&
         abs_i16(g_tf.lookahead_error) <= SPEED_VALID_RUN_LA_MAX &&
         trend_abs <= SPEED_VALID_RUN_TREND_MAX &&
         g_ra_pre_flag == 0u &&
         g_ra_flag == 0u) ? 1u : 0u;

    uint8 ra_pid_speed_direct = (s_ra_state == RA_ST_ACTIVE) ? 1u : 0u;
    float target_speed = (ra_pid_speed_direct ||
                          component_fast || s_straight_active || valid_run_fast) ?
                         (float)base_speed :
                         (float)calc_adapted_speed(base_speed, speed_err); /* ���适应速度 */
    float actual_l = (float)motor_value.receive_left_speed_data;  /* 左轮实际速度 */
    float actual_r = (float)motor_value.receive_right_speed_data; /* 右轮实际速度 */
    float avg_actual = (actual_l + actual_r) * 0.5f; /* 左右�平均速度 */

    /* 加�度前�：�标�度变化�?* 增益 */
    float accel_ff = 0.0f;                  /* 加�度前�，初�为0 */
    if (s_speed_ff_ready)                   /* 前�就�（非首周期） */
        accel_ff = (target_speed - s_prev_target_speed) * PID_D_SCALE * SPEED_ACCEL_FF_GAIN; /* �标变化量前�?*/
    else                                    /* 首周�?*/
        s_speed_ff_ready = 1u;              /* 标�前馈就� */

    accel_ff = clamp_f(accel_ff, -SPEED_ACCEL_FF_LIMIT, SPEED_ACCEL_FF_LIMIT); /* 限幅加�度前� */
    s_prev_target_speed = target_speed;     /* 保存当前�标�度 */

    /* 速度输出 = 前� + PI */
    float speed_ff = target_speed * SPEED_FF_RATIO + accel_ff; /* 速度前� + 加�度前� */
    float speed_out = speed_ff + speed_pi_calc(target_speed,   /* 前� + 速度PI */
                                               avg_actual,     /* 实际速度 */
                                               &s_speed_integral, /* �分指� */
                                               pos_err_abs);  /* 位置��（积分分离用�?*/
    if (target_speed <= 0.0f)
    {
        speed_out = 0.0f;
        s_speed_integral = 0.0f;
    }
    else if (speed_out < 0.0f)
    {
        speed_out = 0.0f;
        if (s_speed_integral < 0.0f)
            s_speed_integral *= 0.5f;
    }

    /* 速度因子：高速时温和增加�向补� */
    if (target_speed > 0.0f)
    {
        float speed_headroom = target_speed * ((float)SPEED_OUT_MAX_PCT * 0.01f) - target_speed;
        float speed_out_max;
        int16 speed_out_signal = pos_err_abs;
        int16 speed_out_la = abs_i16(g_tf.lookahead_error);
        int16 speed_out_trend = abs_i16(g_tf.error_trend);
        if (speed_headroom < SPEED_OUT_MIN_HEADROOM)
            speed_headroom = SPEED_OUT_MIN_HEADROOM;
        if (speed_headroom > SPEED_OUT_MAX_HEADROOM)
            speed_headroom = SPEED_OUT_MAX_HEADROOM;
        if (speed_out_la > speed_out_signal)
            speed_out_signal = speed_out_la;
        if (speed_out_trend > speed_out_signal)
            speed_out_signal = speed_out_trend;
        if (speed_out_signal > SPEED_OUT_TURN_T1)
        {
            float turn_t = range_ratio_i16(speed_out_signal,
                                           SPEED_OUT_TURN_T1,
                                           SPEED_OUT_TURN_T2);
            float min_pct = (float)SPEED_OUT_TURN_MIN_HEADROOM_PCT * 0.01f;
            speed_headroom *= lerp_f(1.0f, min_pct, turn_t);
            if (speed_headroom < SPEED_OUT_MIN_HEADROOM)
                speed_headroom = SPEED_OUT_MIN_HEADROOM;
        }
        speed_out_max = target_speed + speed_headroom;
        if (speed_out > speed_out_max)
        {
            speed_out = speed_out_max;
            if (s_speed_integral > 0.0f)
                s_speed_integral *= 0.85f;
        }
    }

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

    steer *= speed_factor;                  /* �向乘以�度因子 */
    steer += ra_pre_turn_steer_ff();
    steer += ra_curve_steer_assist();
    steer = ra_yaw_guard_steer(steer);
    steer = ra_pre_turn_steer_guard(steer);
    steer = line_recenter_steer_boost(steer);
    steer = line_left_bias_steer_trim(steer);

    /* 直道时转向缩�?*/
    if (s_straight_active)                  /* 直道加�模�?*/
        steer *= (float)SPEED_STRAIGHT_STEER_PCT * 0.01f; /* �向缩�(100%) */

    /* RECOVER阶�：�向缩�?+ yaw�� */
    if (s_ra_state == RA_ST_ACTIVE && s_ra_phase == RA_PH_RECOVER) /* RECOVER阶� */
    {
        steer *= recover_steer_scale();      /* �向渐入，避免出弯抖�?*/
        if (imu_ready && s_ra_dir != 0u &&
            RA_RECOVER_YAW_KP > 0.0f)
        {
            float yaw_err = (float)ra_hard_yaw - ra_yaw_progress();
            uint8 yaw_drive_allowed = 1u;
            if (yaw_err > 0.0f &&
                ra_yaw_progress_rate() > RA_RECOVER_YAW_DRIVE_RATE_MAX)
                yaw_drive_allowed = 0u;
            if (yaw_drive_allowed && abs_f(yaw_err) > RA_RECOVER_YAW_DEADZONE)
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

    /* �速限�?*/
    if (s_ra_post_recover_cnt > 0u)
        steer *= (float)RA_POST_RECOVER_STEER_PCT * 0.01f;

    steer = limit_normal_steer(steer, speed_out); /* 限制�速幅�?*/

    /* yaw_rate 限幅：仅在转向与 yaw_rate 同号（即�向�在加剧旋转、有发散风险�
     * 时才衰减；转向在纠�旋�（反号）时不�，避免真�道跟随��砍�
     * 并保留下限，绝不把�转的转向清零� */
    if (!curve_mode && imu_ready && !imu_error)              /* IMU正常工作 */
    {
        float yaw_rate_f = (float)yaw_rate;
        float yaw_rate_abs = abs_f(yaw_rate_f); /* 角�度绝�� */
        /* steer>0 左轮�/右轮� � 右转，yaw_rate 符号与转向方向一致才�"加剧" */
        uint8 amplifying = ((steer > 0.0f && yaw_rate_f > 0.0f) ||
                            (steer < 0.0f && yaw_rate_f < 0.0f)) ? 1u : 0u;
        if (amplifying && yaw_rate_abs > YAW_RATE_LIMIT) /* 同向且超限才衰减 */
        {
            float scale = 1.0f - (yaw_rate_abs - YAW_RATE_LIMIT) / YAW_RATE_LIMIT; /* 按比例减� */
            float min_scale = (float)YAW_RATE_LIMIT_MIN_PCT * 0.01f;
            if (scale < min_scale) scale = min_scale; /* 保留下限，不清零 */
            steer *= scale;                   /* �向输出按比例缩放 */
        }
    }

    speed_dbg_out = (int16)speed_out;       /* 记录速度调试�?*/
    steer_dbg_out = (int16)steer;           /* 记录�向调试�?*/

    /* �终输出：左轮=速度+�向，右�?速度-�� */
    pid_set_duty(clamp_duty(speed_out + steer), /* 左轮 = 速度+�� */
                          clamp_duty(speed_out - steer)); /* 右轮 = 速度-�� */
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
    s_ra_post_recover_cnt = 0u;
    s_ra_lost_guard_cnt = 0u;
    s_ra_exit_boost_active = 0u;
    s_ra_exit_boost_cnt = 0u;
    ra_dbg_exit_boost_active = 0u;
    ra_dbg_exit_boost_cnt = 0u;
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

/* line_pid_control - 主PID控制函数，由PID_PERIOD_MS PIT��调用（isr.c:cc60_pit_ch0_isr�? * 这是整个控制系统的核心，每PID_PERIOD_MS执�一�? *
 * 执�流程�? *   1. 电机�使�?�?清零�有状态并返回
 *   2. 运�超� �?停机
 *   3. �线完成延迟停�
 *   4. 更新��屏蔽
 *   5. RA状�机（可能直接输出电机）
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

    /* ===== 电机�使能：全部�位 ===== */
    if (motor_enable == 0)                   /* 电机�使�?*/
    {
        pid_stop_runtime_reset(0u);
        return;                              /* 返回 */
    }

    /* ===== 运�超时保� ===== */
    s_motor_run_counter++;                   /* 运�帧计数�1 */
    if (s_motor_run_counter >= PID_SECONDS_TO_TICKS((uint32)motor_run_time)) /* 超过运�时� */
    {
        motor_enable = 0;                    /* 关闭电机使能 */
        vacuum_stop();
        pid_stop_runtime_reset(0u);
        return;                              /* 返回 */
    }

    /* ===== �线完成延迟停� ===== */
    if (s_rules_done)                        /* �线全部完� */
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


    /* ===== ��屏蔽更新 ===== */

    int16 pos_err = g_tf.error;             /* 获取当前位置�� */
    int16 pos_err_abs = abs_i16(pos_err);   /* 位置��绝��?*/

    /* ===== RA状�机 ===== */
    if (ra_try_start_from_pre_direct())
    {
        ra_debug_update();
    }

    RaResult ra = ra_state_machine_step(pos_err_abs); /* 执�RA状�机 */
    /* HARD阶�RA已直接输出电机，�帧不再执行PID */
    if (ra.should_return)                    /* RA已直接输出电�?*/
    {
        auto_tune_log_pid_tick();
        return;                              /* 跳过��PID */
    }

    /* Item 8: Decrement continuous turn remnant when RA is not active */
    if (s_continuous_turn_remnant_frames > 0u && s_ra_state == RA_ST_NONE)
    {
        s_continuous_turn_remnant_frames--;
        if (s_continuous_turn_remnant_frames == 0u)
        {
            s_continuous_turn_mode = 0u;
            turn_assist_reset();
            ra_dbg_continuous_turn_mode = 0u;
        }
    }

    /* ===== 丢线搜索 ===== */
    /* 丢线搜索已直接输出电机，�帧不再执行PID */
    if (lost_search_step(pos_err))           /* 丢线搜索正在执� */
    {
        auto_tune_log_pid_tick();
        return;                              /* 跳过��PID */
    }

    /* ===== 速度规划 ===== */
    /* 原�目标�度 = 菜单速度 * 8 * RA速度缩放 */
    int16 target_base_speed = (int16)((float)motor_speed * 8.0f * ra.speed_scale); /* 原�目标�度 */
    uint8 pre_slow_active = 0u;              /* 预减速激活标�?*/
    speed_dbg_pre_lock = 0u;                 /* 预减速锁调试标志清零 */
    speed_dbg_raw = target_base_speed;       /* 记录原�目标�度 */

    /* ===== 预减速�辑 ===== */
    /* �?right_angle_pre_detect() �测到直�但还未正式触发RA�?     * 提前降低速度，防止冲出赛�?*/
    if (s_ra_state == RA_ST_NONE)            /* RA���?*/
    {
        /* �测到方向预判或远场�减速信号，且无正式flag �?锁定预减�?*/
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
            if (pre_slow_signal_trusted(pos_err_abs))
            {
                s_pre_lock = 1u;
                s_pre_timeout = 0u;
            }
            else
            {
                s_pre_lock = 0u;
                s_pre_timeout = 0u;
                g_ra_pre_flag = 0u;
                g_ra_pre_dir = 0u;
                g_ra_pre_slow_flag = 0u;
            }
        }

        /* 正式flag到来 �?解除预减速（让RA接��?*/
        if (g_ra_flag != 0u)                 /* 有�式RA flag */
        {
            s_pre_lock = 0u;                 /* 解除锁定 */
            g_ra_pre_slow_flag = 0u;         /* 正式RA接�后清除预减速专用标�?*/
        }

        /* 对称组件（三极�干扰区� �?解除预减�?*/
        if (g_sym_component_flag)            /* �测到对称组件 */
        {
            s_pre_lock = 0u;                 /* 解除锁定 */
            s_pre_timeout = 0u;              /* 超时清零 */
            g_ra_pre_slow_flag = 0u;         /* 防�元器件远场特征锁住预减�?*/
        }

        /* 锁定状�下，�减速信号消失后超时解除 */
        if (s_pre_lock)                      /* 处于锁定状�?*/
        {
            if (!g_ra_pre_flag && !g_ra_pre_slow_flag)
            {
                /* 恢�直� �?解除 */
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
                pre_slow_active = 1u;        /* 标��减速激�?*/
                target_base_speed = (int16)((int32)target_base_speed * RA_PRE_SLOW_PCT / 100); /* 降�?*/
            }
        }

        speed_dbg_pre_lock = s_pre_lock;     /* 记录预减速锁状�?*/
    }

    /* 规划基�速度（含各�降�?加�策略） */
    base_speed = plan_base_speed(target_base_speed, pos_err_abs, pre_slow_active); /* 速度规划 */

    /* 单边巡线倒��?*/
    single_edge_tick();                      /* 更新单边巡线状�?*/
    /* 执��常PID控制（转�?速度�?*/
    normal_pid_step(pos_err, pos_err_abs);   /* 执��常PID控制 */
    auto_tune_log_pid_tick();
}
