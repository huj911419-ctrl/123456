#include "Pid.h"                /* PID控制器头文件，包含所有PID相关常量和变量声明 */
#include "Menu.h"               /* 菜单系统头文件，提供菜单变量（如motor_speed等） */
#include "Track_funsion.h"      /* 视觉融合头文件，提供g_tf视觉状态结构体 */
#include "IMU.h"                /* IMU头文件，提供yaw_angle、yaw_rate等IMU数据 */

/* ======================== 全局调试变量 ======================== */

/* 基础速度，由 plan_base_speed() 规划后写入，单位为菜单设定的速度档位值 */
int16 base_speed = 0;           /* 基础速度值，PID控制的核心速度输入 */
/* 速度环调试输出值（含转向叠加后的左右轮差速中间量） */
int16 speed_dbg_out = 0;        /* 速度调试输出，供TFT显示用 */
/* 转向调试输出值（纯转向分量） */
int16 steer_dbg_out = 0;        /* 转向调试输出，供TFT显示用 */
/* 视觉质量降速百分比（100=无降速），用于TFT调试显示 */
uint8 speed_dbg_vq_pct = 100u;  /* 视觉质量降速百分比，100表示无降速 */
/* 预检测锁定标志，用于TFT调试显示（1=正在预减速） */
uint8 speed_dbg_pre_lock = 0u;  /* 预减速锁定标志，1表示正在预减速 */
/* 规划前的原始目标速度，用于TFT调试显示 */
int16 speed_dbg_raw = 0;        /* 原始目标速度，速度规划前的输入值 */
/* 规划后的目标速度，用于TFT调试显示 */
int16 speed_dbg_plan = 0;       /* 规划后目标速度，速度规划输出值 */
/* 速度规划原因编号，用于TFT调试显示（1=RA 2=丢线 3=视觉差 4=对称加速 5=前瞻 6=直道加速 7=质量 8=正常 9=单边加速） */
uint8 speed_dbg_reason = 0u;    /* 速度变化原因编号，用于调试 */
/* 后转弯单边巡线方向，由 start_single_edge() 写入，Track_funsion.c 读取
 * EDGE_BOTH=双边, EDGE_LEFT=只用左边线, EDGE_RIGHT=只用右边线 */
uint8 g_post_edge_side = EDGE_BOTH; /* 单边巡线方向，全局变量供视觉模块读取 */

/* ======================== RA（直角/路口）调试变量 ======================== */

/* RA状态机当前状态（0=空闲 1=活跃） */
uint8 ra_dbg_state = 0u;        /* RA状态调试变量 */
/* RA当前阶段（0=WAIT 1=SLOW 2=APPROACH 3=HARD 4=RECOVER） */
uint8 ra_dbg_phase = 0u;        /* RA阶段调试变量 */
/* RA转弯方向（0=直行 1=右转 2=左转） */
uint8 ra_dbg_dir = 0u;          /* RA方向调试变量 */
/* RA记录的最大拐点行号 */
uint8 ra_dbg_ip_row = 0u;       /* RA拐点行调试变量 */
/* RA全局计时器（帧数），每PID周期+1 */
uint16 ra_dbg_timer = 0u;       /* RA全局计时器调试变量 */
/* RA HARD阶段帧计数 */
uint16 ra_dbg_hard_cnt = 0u;    /* RA硬转弯计数调试变量 */
/* RA退出条件满足的连续帧数（camera_done判断用） */
uint8 ra_dbg_exit_good_cnt = 0u;/* RA退出计数调试变量 */
/* RA yaw进度 x10，用于TFT显示（避免浮点） */
int16 ra_dbg_yaw10 = 0;         /* RA偏航进度调试变量（乘10避免浮点） */

/* ======================== 转向PD控制静态变量 ======================== */

/* 上一次的位置误差（滤波后），用于D项计算 */
static float s_steer_last_pos_err = 0.0f;   /* 上一周期滤波后误差 */
/* 上一次的原始位置误差（未滤波），用于D项计算 */
static float s_steer_last_raw_err = 0.0f;   /* 上一周期原始误差 */
/* 低通滤波后的位置误差，一阶IIR滤波器输出 */
static float s_filtered_err = 0.0f;          /* 滤波后误差值 */
/* 上一次的转向输出值，用于变化率限制（slew rate limiter） */
static float s_prev_steer_output = 0.0f;     /* 上一周期转向输出 */
/* 前馈信号的低通滤波值，避免前瞻突变导致转向抖动 */
static float s_steer_ff_filtered = 0.0f;     /* 滤波后的前馈信号 */
/* D项重置标志，置1后下一个周期跳过D输出（防止切换时微分冲击） */
static uint8 s_steer_d_reset_flag = 0u;      /* 微分重置标志 */

/* ======================== 速度PI控制静态变量 ======================== */

/* 速度PI控制器的积分累积量 */
static float s_speed_integral = 0.0f;        /* 速度积分累积量 */
/* 上一周期的目标速度，用于加速度前馈计算 */
static float s_prev_target_speed = 0.0f;     /* 上一周期目标速度 */
/* 速度前馈就绪标志，首周期跳过前馈（避免启动突变） */
static uint8 s_speed_ff_ready = 0u;          /* 前馈就绪标志 */
/* 电机运行帧计数器，用于 motor_run_time 超时停机 */
static uint32 s_motor_run_counter = 0;       /* 电机运行帧计数 */
static uint8 s_vacuum_on = 0u;
volatile uint8 vacuum_enable = 0u;           /* 负压实际开启状态，供TFT显示/关屏逻辑使用 */

static void vacuum_set(uint8 on)
{
    if (on)
    {
        if (s_vacuum_on == 0u)
        {
            pwm_set_duty(VAC_PWM_CH, VAC_PWM_DUTY);
            s_vacuum_on = 1u;
            vacuum_enable = 1u;
        }
    }
    else
    {
        if (s_vacuum_on != 0u)
        {
            pwm_set_duty(VAC_PWM_CH, 0u);
            s_vacuum_on = 0u;
            vacuum_enable = 0u;
        }
    }
}

/* ======================== 速度规划静态变量 ======================== */

/* 经过斜坡处理后的基础速度，防止速度突变 */
static int16 s_base_speed_ramped = 0;        /* 斜坡处理后的速度 */
/* 直道条件满足的连续帧计数，用于直道加速确认 */
static uint8 s_straight_cnt = 0u;            /* 直道确认帧计数 */
/* 直道加速激活标志（1=当前处于直道加速模式） */
static uint8 s_straight_active = 0u;         /* 直道加速激活标志 */
/* 上一帧的误差值，用于误差跳变检测（视觉质量降速） */
static int16 s_prev_quality_err = 0;         /* 上一帧误差值 */
/* 上一帧误差值有效标志 */
static uint8 s_prev_quality_err_valid = 0u;  /* 上一帧误差有效标志 */

/* ======================== 转弯屏蔽（turn shield）静态变量 ======================== */

/* 转弯屏蔽剩余帧数，>0时屏蔽RA检测，防止出弯后误触发 */
static uint8 s_turn_shield_frames = 0u;      /* 转弯屏蔽剩余帧数 */
/* 转弯屏蔽累计轮速距离，与帧数共同决定屏蔽结束 */
static uint32 s_turn_shield_dist = 0u;       /* 转弯屏蔽累计距离 */

/* ======================== 单边巡线静态变量 ======================== */

/* 单边巡线激活标志 */
static uint8 s_edge_active = 0u;             /* 单边巡线激活标志 */
/* 当前单边巡线方向（EDGE_LEFT / EDGE_RIGHT） */
static uint8 s_edge_side = EDGE_BOTH;        /* 单边巡线方向 */
/* 单边巡线已运行帧计数 */
static uint16 s_edge_cnt = 0u;               /* 单边巡线当前帧计数 */
/* 单边巡线目标帧数（由时间ms转换） */
static uint16 s_edge_target = 0u;            /* 单边巡线目标帧数 */
/* 单边巡线是否保持到下一次真正转弯RA启动 */
static uint8 s_edge_until_next_turn = 0u;    /* 1=不按时间结束，等下一次非直行RA完成 */
/* 保持模式已遇到下一次真正转弯，等该RA结束后关闭单边 */
static uint8 s_edge_release_after_turn = 0u; /* 1=当前非直行RA结束后恢复双边 */

/* ======================== 丢线搜索静态变量 ======================== */

/* 丢线搜索激活标志（1=正在执行原地转向搜索） */
static uint8 s_lost_search_active = 0u;      /* 丢线搜索激活标志 */
/* 连续丢线帧计数，达到阈值后启动搜索 */
static uint8 s_lost_line_cnt = 0u;           /* 连续丢线帧计数 */
/* 丢线搜索已运行帧计数 */
static uint16 s_lost_search_cnt = 0u;        /* 丢线搜索运行帧计数 */
/* 丢线搜索方向（1=向右转 2=向左转） */
static uint8 s_lost_search_dir = 1u;         /* 丢线搜索方向 */
/* 丢线前的最后误差值，用于选择搜索方向 */
static int16 s_lost_last_err = 0;            /* 丢线前最后误差 */

/* ======================== 串级PID（cascade）内环状态 ======================== */

/* Cascade外环输出的目标角速度，经低通滤波后送给内环 */
static float s_cas_target_filtered = 0.0f;   /* 串级PID目标角速度（滤波后） */
/* 内环上一帧的角速度误差，用于内环D项 */
static float s_cas_last_yaw_err    = 0.0f;   /* 内环上一帧角速度误差 */

/* Cascade菜单变量（在Menu.c中可调） */
/* 串级PID使能标志（0=普通PD 1=串级IMU角速度环） */
int16 cascade_en    = 0;                     /* 串级PID使能标志 */
/* 内环角速度D增益（菜单可调） */
int16 yaw_kd        = 3;                     /* 内环角速度微分增益 */
/* 独立yaw阻尼增益（cascade_en=0时的角速度阻尼，0=关闭） */
int16 yaw_damp_gain = 0;                     /* 独立偏航阻尼增益 */

/* ======================== RA状态机枚举 ======================== */

/* RA状态：无活动 / 有活动 */
typedef enum { RA_ST_NONE, RA_ST_ACTIVE } RaState;  /* RA状态枚举 */
/* RA阶段：等待拐点接近 → 减速 → 接近 → 硬转弯 → 恢复 */
typedef enum { RA_PH_WAIT, RA_PH_SLOW, RA_PH_APPROACH, RA_PH_HARD, RA_PH_RECOVER } RaPhase; /* RA阶段枚举 */

/* RA状态机当前状态 */
static RaState s_ra_state = RA_ST_NONE;      /* RA当前状态，初始为空闲 */
/* RA状态机当前阶段 */
static RaPhase s_ra_phase = RA_PH_WAIT;      /* RA当前阶段，初始为等待 */
/* RA转弯方向（0=直行通过 1=右转 2=左转） */
static uint8 s_ra_dir = 0u;                  /* RA转弯方向 */
/* 触发RA的原始flag值（1~5），用于查路线表和超时配置 */
static uint8 s_ra_orig_flag = 0u;            /* 原始触发flag */
/* RA记录的拐点最大行号，用于阶段转换判断 */
static uint8 s_ra_ip_row = 0u;               /* 拐点最大行号 */
/* 直行通过标志（1=该路口直行不转弯） */
static uint8 s_ra_straight = 0u;             /* 直行通过标志 */
/* RA结束后启用的单边巡线方向 */
static uint8 s_ra_post_edge_side = EDGE_BOTH;/* 结束后单边巡线方向 */
/* RA结束后单边巡线持续时间(ms) */
static uint16 s_ra_post_edge_ms = 0u;        /* 结束后单边巡线时间 */
/* HARD阶段中满足退出条件的连续帧计数 */
static uint8 s_ra_exit_good_cnt = 0u;        /* HARD退出条件满足帧计数 */
/* RECOVER阶段中满足恢复完成条件的连续帧计数 */
static uint8 s_ra_recover_good_cnt = 0u;     /* RECOVER完成条件满足帧计数 */
/* APPROACH阶段帧计数 */
static uint16 s_ra_approach_cnt = 0u;        /* APPROACH阶段帧计数 */
/* RA全局计时器，每PID周期+1，用于超时保护 */
static uint16 s_ra_timer = 0u;               /* RA全局计时器 */
/* HARD阶段帧计数 */
static uint16 s_ra_hard_cnt = 0u;            /* HARD阶段帧计数 */
/* RECOVER阶段帧计数 */
static uint16 s_ra_recover_cnt = 0u;         /* RECOVER阶段帧计数 */
/* 当前阶段内的帧计数（WAIT/SLOW用） */
static uint16 s_ra_phase_cnt = 0u;           /* 当前阶段帧计数 */
/* HARD阶段的速度种子值，用于RECOVER阶段的速度平滑过渡 */
static float s_ra_hard_speed_seed = 0.0f;    /* HARD阶段速度种子 */
/* HARD阶段的转向种子值，用于RECOVER阶段的转向平滑过渡 */
static float s_ra_hard_steer_seed = 0.0f;    /* HARD阶段转向种子 */

/* 前向声明：int16取绝对值 */
static int16 abs_i16(int16 v);               /* 前向声明：int16绝对值函数 */

/* ======================== 路线规则定义 ======================== */

/* 路口动作类型 */
#define ACT_STRAIGHT 0u  /* 直行通过 */
#define ACT_RIGHT    1u  /* 右转 */
#define ACT_LEFT     2u  /* 左转 */
#define ACT_AUTO     3u  /* 自动（根据flag推断方向，用于直角） */

/* 路口规则结构体 */
typedef struct
{
    uint8 count;            /* 第几次出现该类型flag时匹配此规则 */
    uint8 flag;             /* 路口类型flag（1=右直角 2=左直角 3/4/5=普通路口） */
    uint8 action;           /* 执行动作（ACT_STRAIGHT/RIGHT/LEFT/AUTO） */
    uint8 post_edge_side;   /* 转弯后单边巡线方向（EDGE_BOTH=不启用） */
    uint16 post_edge_ms;    /* 转弯后单边巡线持续时间(ms) */
} IntersectionRule;         /* 路口规则结构体定义 */

/* 规则构造宏 */
#define RULE(count, flag, action) \
    { (count), (flag), (action), EDGE_BOTH, 0u }           /* 普通规则，无单边 */
#define RULE_EDGE(count, flag, action, edge_side, edge_ms) \
    { (count), (flag), (action), (edge_side), (edge_ms) }   /* 指定单边方向 */
#define RULE_AUTO(count, flag, action, edge_ms) \
    { (count), (flag), (action), EDGE_AUTO, (edge_ms) }     /* 自动选单边方向 */
#define RULE_RA(count, flag) \
    { (count), (flag), ACT_AUTO, EDGE_BOTH, 0u }            /* 直角自动，无单边 */
#define RULE_RA_AUTO(count, flag, edge_ms) \
    { (count), (flag), ACT_AUTO, EDGE_AUTO, (edge_ms) }     /* 直角自动+自动单边 */
#define RULE_RA_EDGE(count, flag, edge_side, edge_ms) \
    { (count), (flag), ACT_AUTO, (edge_side), (edge_ms) }   /* 直角自动+指定单边 */

/* 路线表：按图中黑线走向填写。
 * RULE：执行指定动作，不开启单边巡线。
 * RULE_AUTO：执行指定动作，转弯结束后自动选单边。
 * RULE_EDGE：执行指定动作，结束后强制指定单边。
 * RULE_RA：直角方向自动，不开启单边。
 * RULE_RA_AUTO：直角方向自动，转完后自动选单边。
 * RULE_RA_EDGE：直角方向自动，转完后强制指定单边。
 * 直角类型：1=右直角，2=左直角。普通路口类型：3/4/5。 */
static const IntersectionRule user_rules[] = {
    /* 如果某个直角出弯后需要单边，就在这里插入：
     * RULE_RA_AUTO(第几次, 1u或2u, 持续时间),
     * 例如：RULE_RA_AUTO(2u, 1u, 500u), */

    /* 当前最快路线：
     * 右直角 -> 右直角 -> 左直角 -> 4右 -> 5左 -> 5右 -> 4右 -> 4右
     * -> 5左 -> 3左 -> 3直行 -> 5右 -> 右直角 -> 右直角后单边。 */
    RULE_RA(  1u, 1u),    /* 第1个flag=1（右直角）：自动方向 */
    RULE(     1u, 4u, ACT_RIGHT),   /* 第1个flag=4（T右）：右转 */
    RULE_RA(  1u, 2u),    /* 第1个flag=2（左直角）：自动方向 */
    RULE(     1u, 5u, ACT_LEFT),    /* 第1个flag=5（十字）：左转 */
    RULE(     2u, 5u, ACT_RIGHT),   /* 第2个flag=5（十字）：右转 */
    RULE(     2u, 4u, ACT_RIGHT),   /* 第2个flag=4（T右）：右转 */
    RULE(     3u, 4u, ACT_RIGHT),   /* 第3个flag=4（T右）：右转 */
    RULE(     3u, 5u, ACT_LEFT),    /* 第3个flag=5（十字）：左转 */
    RULE(     4u, 5u, ACT_LEFT),    /* 第4个flag=5（十字）：左转 */
    RULE(     1u, 3u, ACT_LEFT),    /* 第1个flag=3（T左）：左转 */
    RULE(     2u, 3u, ACT_STRAIGHT),/* 第2个flag=3（T左）：直行 */
    RULE(     5u, 5u, ACT_RIGHT),   /* 第5个flag=5（十字）：右转 */
    RULE_RA(  2u, 1u),    /* 第2个flag=1（右直角）：自动方向 */
    RULE_RA_EDGE(3u, 1u, EDGE_LEFT, SINGLE_EDGE_UNTIL_NEXT_TURN), /* 第3个flag=1（右直角）：结束后靠左单边直到下个真转弯 */
    RULE(     4u, 4u, ACT_STRAIGHT),/* 第4个flag=4（T右）：直行 */
    RULE(     5u, 4u, ACT_STRAIGHT),/* 第5个flag=4（T右）：直行 */
    RULE_RA(  4u, 1u),    /* 第4个flag=1（右直角）：自动方向 */
};
/* 路线表总条目数 */
#define USER_RULE_COUNT (sizeof(user_rules) / sizeof(user_rules[0])) /* 路线表条目数 */

/* ======================== 路线匹配状态 ======================== */

/* 各类型flag（0~6）的已匹配计数，用于按序匹配路线表 */
static uint8 s_inter_count[7] = {0u};  /* 各flag类型已匹配次数 */
/* 路线全部完成标志，置1后延迟停机 */
static uint8 s_rules_done = 0u;        /* 路线完成标志 */
/* 路线完成后的延迟停机计时器（帧数） */
static uint16 s_rules_done_timer = 0u; /* 路线完成延迟停机计时器 */
/* 路线调试：当前执行到第几步（从0开始） */
uint8 route_dbg_step = 0u;             /* 路线调试当前步数 */
/* 路线调试：路线表总步数 */
uint8 route_dbg_total = (uint8)USER_RULE_COUNT; /* 路线调试总步数 */
/* 路线调试：当前匹配的flag类型 */
uint8 route_dbg_flag = 0u;             /* 路线调试当前flag */
/* 路线调试：当前匹配的count值 */
uint8 route_dbg_count = 0u;            /* 路线调试当前count */
/* 路线调试：当前执行的动作 */
uint8 route_dbg_action = ACT_STRAIGHT; /* 路线调试当前动作 */
/* 待提交的路线匹配结果有效标志（延迟一帧提交，避免影响当前帧RA） */
static uint8 s_route_pending_valid = 0u;   /* 待提交有效标志 */
/* 待提交的flag类型 */
static uint8 s_route_pending_flag = 0u;    /* 待提交flag */
/* 待提交的count值 */
static uint8 s_route_pending_count = 0u;   /* 待提交count */
/* 待提交的动作 */
static uint8 s_route_pending_action = ACT_STRAIGHT; /* 待提交动作 */

/* 前向声明：更新RA调试信息 */
static void ra_debug_update(void);         /* 前向声明：更新RA调试变量 */

/* ======================== RA状态机返回结构体 ======================== */

/* RA状态机每帧返回的结果 */
typedef struct
{
    uint8 need_pid;     /* 1=需要继续执行PID控制，0=跳过PID */
    uint8 should_return;/* 1=RA已直接输出电机，本帧PID不执行 */
    float speed_scale;  /* 速度缩放因子（0.0~1.0），用于降速 */
} RaResult;             /* RA状态机返回结构体 */

/* ======================== 路线决策结构体 ======================== */

/* 路口决策结果 */
typedef struct
{
    uint8 action;           /* 动作类型（ACT_STRAIGHT/RIGHT/LEFT） */
    uint8 post_edge_side;   /* 转弯后单边方向 */
    uint16 post_edge_ms;    /* 转弯后单边持续时间 */
    uint8 valid;            /* 决策有效标志（0=未匹配路线表） */
} RouteDecision;            /* 路口决策结构体 */

/* ======================== 转向调度结构体 ======================== */

/* 根据速度和曲率计算的转向PD参数缩放 */
typedef struct
{
    float kp_scale;   /* 比例增益缩放因子 */
    float kd_scale;   /* 微分增益缩放因子 */
    float ff_scale;   /* 前馈增益缩放因子 */
    float slew_max;   /* 输出变化率限制 */
} SteerSchedule;      /* 转向增益调度结构体 */

/* ======================== 工具函数 ======================== */

/* abs_f - 浮点数取绝对值
 * @v: 输入浮点数
 * 返回: |v| */
static float abs_f(float v)                 /* 浮点绝对值函数 */
{
    return (v >= 0.0f) ? v : -v;            /* 非负返回原值，负数取反 */
}

/* clamp_f - 浮点数限幅
 * @v: 输入值
 * @min_v: 下限
 * @max_v: 上限
 * 返回: 限幅后的值 */
static float clamp_f(float v, float min_v, float max_v) /* 浮点限幅函数 */
{
    if (v < min_v) return min_v;            /* 低于下限，返回下限 */
    if (v > max_v) return max_v;            /* 高于上限，返回上限 */
    return v;                               /* 在范围内，返回原值 */
}

/* lerp_f - 线性插值
 * @a: 起始值（t=0时返回）
 * @b: 终止值（t=1时返回）
 * @t: 插值系数（0.0~1.0）
 * 返回: a + (b-a)*t */
static float lerp_f(float a, float b, float t) /* 线性插值函数 */
{
    return a + (b - a) * t;                 /* 标准线性插值公式 */
}

/* range_ratio_i16 - 将int16值映射到[start, end]区间的比例（0.0~1.0）
 * @value: 输入值
 * @start: 区间起点（返回0.0）
 * @end: 区间终点（返回1.0）
 * 返回: 0.0~1.0之间的比例值
 * 用途：增益调度中的归一化 */
static float range_ratio_i16(int16 value, int16 start, int16 end) /* 区间比例映射 */
{
    if (end <= start)                       /* 区间无效或单点 */
        return (value >= end) ? 1.0f : 0.0f; /* 超过终点返回1，否则0 */

    if (value <= start) return 0.0f;        /* 小于起点，返回0 */
    if (value >= end) return 1.0f;          /* 大于终点，返回1 */

    return (float)(value - start) / (float)(end - start); /* 线性比例计算 */
}

/* ======================== 转向增益调度 ======================== */

/* steer_schedule_calc - 根据当前速度和弯道程度计算转向PD的增益缩放参数
 * @pos_err_abs: 位置误差绝对值
 * 返回: 包含kp_scale/kd_scale/ff_scale/slew_max的调度结果
 *
 * 调度逻辑：
 *   1. 取误差/前瞻/趋势中最大的作为弯道信号
 *   2. 速度越高，kp/kd适当降低（防高速振荡）
 *   3. 弯道越急，kp/kd适当增大（加强响应）
 *   4. 前馈在高速时才启用
 *   5. 变化率限制随弯道程度略增 */
static SteerSchedule steer_schedule_calc(int16 pos_err_abs) /* 转向增益调度计算 */
{
    SteerSchedule s;                        /* 调度结果结构体 */
    int16 la_abs = abs_i16(g_tf.lookahead_error);  /* 前瞻误差绝对值 */
    int16 trend_abs = abs_i16(g_tf.error_trend);    /* 趋势误差绝对值 */
    int16 curve_signal = pos_err_abs;       /* 初始弯道信号 = 位置误差 */

    /* 取三者最大值作为弯道信号 */
    if (la_abs > curve_signal) curve_signal = la_abs;   /* 前瞻更大则替换 */
    if (trend_abs > curve_signal) curve_signal = trend_abs; /* 趋势更大则替换 */

    /* 速度因子：低速→0，高速→1 */
    float speed_t = range_ratio_i16((int16)base_speed,  /* 当前基础速度 */
                                    STEER_GAIN_SPEED_START, /* 速度下限180 */
                                    STEER_GAIN_SPEED_END);  /* 速度上限800 */
    /* 弯道因子：直道→0，急弯→1 */
    float curve_t = range_ratio_i16(curve_signal,       /* 弯道信号 */
                                    STEER_GAIN_CURVE_T1, /* 弯道下限10 */
                                    STEER_GAIN_CURVE_T2);/* 弯道上限38 */
    /* 高速时的kp/kd缩放（通常kp略降，kd略增） */
    float kp_fast = lerp_f(1.0f, (float)STEER_FAST_KP_PCT * 0.01f, speed_t); /* 高速kp缩放 */
    float kd_fast = lerp_f(1.0f, (float)STEER_FAST_KD_PCT * 0.01f, speed_t); /* 高速kd缩放 */

    /* 最终kp = 高速kp线性过渡到弯道kp */
    s.kp_scale = lerp_f(kp_fast,                        /* 高速kp基准 */
                        (float)STEER_CURVE_KP_PCT * 0.01f, /* 弯道kp */
                        curve_t);                        /* 弯道因子插值 */
    /* 最终kd = 高速kd线性过渡到弯道kd */
    s.kd_scale = lerp_f(kd_fast,                        /* 高速kd基准 */
                        (float)STEER_CURVE_KD_PCT * 0.01f, /* 弯道kd */
                        curve_t);                        /* 弯道因子插值 */
    /* 前馈缩放：高速时才启用，乘以菜单可调系数 */
    s.ff_scale = range_ratio_i16((int16)base_speed,     /* 当前基础速度 */
                                 STEER_FF_SPEED_START,   /* 前馈启用速度下限 */
                                 STEER_FF_SPEED_END) *   /* 前馈启用速度上限 */
                 ((float)steer_ff_k * 0.01f);            /* 菜单可调前馈系数 */
    /* 变化率限制：弯道时略增（允许更快响应） */
    s.slew_max = STEER_SLEW_MAX * PID_DT_SCALE * lerp_f(0.85f, 1.20f, curve_t); /* 弯道时变化率略增 */

    return s;                               /* 返回调度结果 */
}

/* abs_i16 - int16取绝对值（避免负数溢出）
 * @v: 输入int16值
 * 返回: |v|，使用int16强转避免溢出 */
static int16 abs_i16(int16 v)               /* int16绝对值函数 */
{
    return (v >= 0) ? v : (int16)(-v);      /* 正数返回原值，负数取反并强转 */
}

/* normalize_angle - 角度归一化到[-180, 180]范围
 * @angle: 输入角度（度）
 * 返回: 归一化后的角度 */
static float normalize_angle(float angle)   /* 角度归一化函数 */
{
    while (angle > 180.0f) angle -= 360.0f; /* 大于180则减360 */
    while (angle < -180.0f) angle += 360.0f;/* 小于-180则加360 */
    return angle;                           /* 返回归一化角度 */
}

/* ======================== RA yaw进度计算 ======================== */

/* ra_yaw_progress - 计算RA HARD阶段的yaw转动进度（正值，单位：度）
 * 右转时直接用yaw_angle（正=右转），左转时取反
 * 只返回正值（负值表示还没开始转或方向错误，视为0）
 * 返回: yaw转动进度（正角度） */
static float ra_yaw_progress(void)          /* 计算RA偏航进度 */
{
    float delta = normalize_angle(yaw_angle); /* 获取归一化后的yaw角 */

    /* 左转方向：yaw_angle为负值，取反得到正进度 */
    if (s_ra_dir == 2u)                     /* 左转方向 */
        delta = -delta;                     /* 取反得到正进度 */

    return (delta > 0.0f) ? delta : 0.0f;  /* 返回正值，负值视为0 */
}

/* ======================== RA调试信息更新 ======================== */

/* ra_debug_update - 将RA内部状态复制到全局调试变量，供TFT显示
 * 在RA状态机每个返回点调用
 * 无参数，无返回值 */
static void ra_debug_update(void)           /* 更新RA调试变量 */
{
    float yaw_progress = 0.0f;              /* yaw进度，初始为0 */

    if (s_ra_state == RA_ST_ACTIVE && s_ra_dir != 0u) /* RA活跃且有方向 */
        yaw_progress = ra_yaw_progress();   /* 计算yaw进度 */

    ra_dbg_state = (uint8)s_ra_state;       /* 复制RA状态 */
    ra_dbg_phase = (uint8)s_ra_phase;       /* 复制RA阶段 */
    ra_dbg_dir = s_ra_dir;                  /* 复制RA方向 */
    ra_dbg_ip_row = s_ra_ip_row;            /* 复制拐点行号 */
    ra_dbg_timer = s_ra_timer;              /* 复制全局计时器 */
    ra_dbg_hard_cnt = s_ra_hard_cnt;        /* 复制HARD计数 */
    /* RECOVER阶段显示recover计数，否则显示exit计数 */
    ra_dbg_exit_good_cnt = (s_ra_phase == RA_PH_RECOVER) ? /* 判断当前阶段 */
                           s_ra_recover_good_cnt : /* RECOVER阶段显示恢复计数 */
                           s_ra_exit_good_cnt;     /* 其他阶段显示退出计数 */
    /* yaw进度x10，避免浮点显示 */
    ra_dbg_yaw10 = (int16)(yaw_progress * 10.0f); /* 乘10转int16供TFT显示 */
}

/* ======================== 电机输出限幅 ======================== */

/* clamp_duty - 将浮点duty值限幅到[-MAX_DUTY, MAX_DUTY]并转为int16
 * @val: 浮点duty值
 * 返回: 限幅后的int16 duty值
 * NaN检查：若val!=val为假（NaN），返回0保护电机 */
static int16 clamp_duty(float val)          /* 电机duty限幅函数 */
{
    if (val != val) return 0;               /* NaN检查：NaN不等于自身，返回0保护电机 */
    if (val > MAX_DUTY) val = MAX_DUTY;     /* 超过正最大值，限幅 */
    else if (val < -MAX_DUTY) val = -MAX_DUTY; /* 超过负最大值，限幅 */
    return (int16)val;                      /* 浮点转int16返回 */
}

/* ======================== RA状态机复位 ======================== */

/* ra_reset - 复位RA状态机所有变量到初始状态
 * 无参数，无返回值
 * 调用时机：RA结束、电机使能关闭、初始化时 */
static void ra_reset(void)                  /* RA状态机全复位 */
{
    s_ra_state = RA_ST_NONE;                /* 状态重置为空闲 */
    s_ra_phase = RA_PH_WAIT;                /* 阶段重置为等待 */
    s_ra_dir = 0u;                          /* 方向重置为直行 */
    s_ra_orig_flag = 0u;                    /* 清除原始flag */
    s_ra_ip_row = 0u;                       /* 清除拐点行号 */
    s_ra_straight = 0u;                     /* 清除直行标志 */
    s_ra_post_edge_side = EDGE_BOTH;        /* 单边方向重置为双边 */
    s_ra_post_edge_ms = 0u;                 /* 单边时间清零 */
    s_ra_exit_good_cnt = 0u;                /* 退出计数清零 */
    s_ra_recover_good_cnt = 0u;             /* 恢复计数清零 */
    s_ra_approach_cnt = 0u;                 /* 接近计数清零 */
    s_ra_timer = 0u;                        /* 全局计时器清零 */
    s_ra_hard_cnt = 0u;                     /* HARD计数清零 */
    s_ra_recover_cnt = 0u;                  /* RECOVER计数清零 */
    s_ra_phase_cnt = 0u;                    /* 阶段计数清零 */
    s_ra_hard_speed_seed = 0.0f;            /* 速度种子清零 */
    s_ra_hard_steer_seed = 0.0f;            /* 转向种子清零 */
    s_route_pending_valid = 0u;             /* 待提交标志清零 */
    ra_debug_update();                      /* 更新调试变量 */
}

/* ======================== 路线完成检测 ======================== */

/* update_rules_done - 检查路线表中所有规则是否都已匹配完成
 * 遍历user_rules[]，每条规则要求对应flag的计数>=规则的count
 * 全部满足则置 s_rules_done=1，触发延迟停机
 * 无参数，无返回值 */
static void update_rules_done(void)         /* 检查路线完成状态 */
{
    uint8 all_done = 1u;                    /* 假设全部完成 */

    for (uint8 i = 0u; i < USER_RULE_COUNT; i++) /* 遍历路线表每条规则 */
    {
        uint8 flag = user_rules[i].flag;    /* 获取规则的flag类型 */
        if (flag >= 7u || s_inter_count[flag] < user_rules[i].count) /* 计数不足 */
        {
            all_done = 0u;                  /* 标记未完成 */
            break;                          /* 提前退出循环 */
        }
    }

    if (all_done)                           /* 如果全部规则都满足 */
        s_rules_done = 1u;                  /* 设置路线完成标志 */
}

/* route_debug_reset - 复位路线调试状态
 * 无参数，无返回值 */
static void route_debug_reset(void)         /* 复位路线调试变量 */
{
    route_dbg_step = 0u;                    /* 当前步数清零 */
    route_dbg_total = (uint8)USER_RULE_COUNT; /* 总步数设为规则总数 */
    route_dbg_flag = 0u;                    /* flag清零 */
    route_dbg_count = 0u;                   /* count清零 */
    route_dbg_action = ACT_STRAIGHT;        /* 动作清零为直行 */
    s_route_pending_valid = 0u;             /* 待提交有效标志清零 */
    s_route_pending_flag = 0u;              /* 待提交flag清零 */
    s_route_pending_count = 0u;             /* 待提交count清零 */
    s_route_pending_action = ACT_STRAIGHT;  /* 待提交动作清零 */
}

/* route_debug_commit - 提交待确认的路线匹配结果到调试变量
 * 延迟一帧提交，避免匹配结果影响当帧的RA启动
 * 无参数，无返回值 */
static void route_debug_commit(void)        /* 提交路线调试信息 */
{
    if (!s_route_pending_valid)             /* 无待提交数据 */
        return;                             /* 直接返回 */

    if (route_dbg_step < route_dbg_total)   /* 未超过总步数 */
        route_dbg_step++;                   /* 步数加1 */
    route_dbg_flag = s_route_pending_flag;  /* 提交flag */
    route_dbg_count = s_route_pending_count;/* 提交count */
    route_dbg_action = s_route_pending_action; /* 提交动作 */

    s_route_pending_valid = 0u;             /* 清除待提交标志 */
}

/* ======================== PD微分项重置 ======================== */

/* line_pid_reset_derivative - 完全重置转向PD的微分状态（清零所有历史值）
 * 用于RA开始/结束、停机等场景
 * 无参数，无返回值 */
void line_pid_reset_derivative(void)        /* 完全重置微分状态 */
{
    s_steer_d_reset_flag = 1u;              /* 设置微分重置标志，下一周期跳过D项 */
    s_filtered_err = 0.0f;                  /* 滤波误差清零 */
    s_prev_steer_output = 0.0f;             /* 上次转向输出清零 */
    s_steer_ff_filtered = 0.0f;             /* 前馈滤波值清零 */
    s_cas_target_filtered = 0.0f;           /* 串级目标滤波值清零 */
    s_cas_last_yaw_err = 0.0f;             /* 串级内环误差清零 */
}

/* line_pid_reset_derivative_keep_output - 部分重置微分状态（保留输出值的50%）
 * 用于RECOVER阶段退出，避免转向突变
 * 无参数，无返回值 */
static void line_pid_reset_derivative_keep_output(void) /* 部分重置微分 */
{
    s_steer_d_reset_flag = 1u;              /* 设置微分重置标志 */
    s_steer_ff_filtered *= 0.5f;            /* 前馈值保留50% */
    s_cas_target_filtered = 0.0f;           /* 串级目标清零 */
    s_cas_last_yaw_err = 0.0f;             /* 串级内环误差清零 */
}

/* ======================== 速度规划器复位 ======================== */

/* reset_speed_planner - 复位速度规划相关的所有状态
 * 无参数，无返回值 */
static void reset_speed_planner(void)       /* 复位速度规划器 */
{
    s_base_speed_ramped = 0;                /* 斜坡速度清零 */
    s_straight_cnt = 0u;                    /* 直道计数清零 */
    s_straight_active = 0u;                 /* 直道激活标志清零 */
    s_prev_quality_err = 0;                 /* 上帧误差清零 */
    s_prev_quality_err_valid = 0u;          /* 上帧误差有效标志清零 */
    speed_dbg_vq_pct = 100u;               /* 视觉质量百分比重置为100 */
    speed_dbg_pre_lock = 0u;               /* 预减速锁标志清零 */
    speed_dbg_raw = 0;                      /* 原始速度清零 */
    speed_dbg_plan = 0;                     /* 规划速度清零 */
    speed_dbg_reason = 0u;                  /* 速度原因清零 */
}

/* reset_speed_ff_state - 复位速度前馈状态
 * 无参数，无返回值 */
static void reset_speed_ff_state(void)      /* 复位速度前馈 */
{
    s_prev_target_speed = 0.0f;             /* 上一目标速度清零 */
    s_speed_ff_ready = 0u;                  /* 前馈就绪标志清零 */
}

/* turn_shield_reset - 复位转弯屏蔽状态
 * 无参数，无返回值 */
static void turn_shield_reset(void)         /* 复位转弯屏蔽 */
{
    s_turn_shield_frames = 0u;              /* 屏蔽帧数清零 */
    s_turn_shield_dist = 0u;               /* 屏蔽距离清零 */
}

/* ======================== 路线匹配辅助 ======================== */

/* route_has_next_match - 检查指定flag的下一个count是否能在路线表中找到匹配
 * @flag: 路口类型flag（1~5）
 * 返回: 1=有下一个匹配，0=无 */
static uint8 route_has_next_match(uint8 flag) /* 检查路线表是否有下一条匹配 */
{
    uint8 next_count;                       /* 下一个期望的count值 */

    if (flag >= 7u)                         /* flag越界检查 */
        return 0u;                          /* 无效flag，无匹配 */

    next_count = s_inter_count[flag] + 1u;  /* 当前计数+1 = 下一个期望值 */

    for (uint8 i = 0u; i < USER_RULE_COUNT; i++) /* 遍历路线表 */
    {
        if (user_rules[i].flag == flag &&   /* flag类型匹配 */
            user_rules[i].count == next_count) /* count值匹配 */
        {
            return 1u;                      /* 找到匹配，返回1 */
        }
    }

    return 0u;                              /* 遍历完未找到，返回0 */
}

static uint8 route_next_rule_is(uint8 flag)
{
    if (route_dbg_step >= USER_RULE_COUNT)
        return 0u;
    return (user_rules[route_dbg_step].flag == flag) ? 1u : 0u;
}

/* ra_fallback_direct_enabled - 检查RA保底直接转弯是否启用
 * @flag: 路口类型flag
 * 返回: 1=允许保底转弯，0=不允许
 * 当RA_FALLBACK_DIRECT_ENABLE=1时，直角flag(1/2)允许保底转弯
 * 即使路线表中没有匹配到对应规则 */
static uint8 ra_fallback_direct_enabled(uint8 flag) /* 检查RA保底转弯 */
{
#if RA_FALLBACK_DIRECT_ENABLE               /* 编译时开关：启用保底 */
    return (flag == 1u || flag == 2u) ? 1u : 0u; /* 直角flag(1/2)允许保底 */
#else                                       /* 保底关闭 */
    (void)flag;                             /* 消除未使用参数警告 */
    return 0u;                              /* 不允许保底 */
#endif
}

/* ra_accept_near_flag - 检查是否接受近距离检测到的RA flag
 * @flag: 路口类型flag
 * 返回: 1=接受，0=拒绝
 * 条件：路线表有下一个匹配，或保底直接转弯已启用
 * 用于转弯屏蔽期间和RECOVER阶段的近距flag处理 */
static uint8 ra_accept_near_flag(uint8 flag) /* 检查近距RA flag是否可接受 */
{
    if (route_has_next_match(flag))          /* 路线表有下一个匹配 */
        return 1u;                           /* 接受 */

    return ra_fallback_direct_enabled(flag); /* 检查保底转弯是否启用 */
}

/* ======================== 丢线搜索 ======================== */

/* lost_search_reset - 复位丢线搜索状态
 * 无参数，无返回值 */
static void lost_search_reset(void)         /* 复位丢线搜索 */
{
    s_lost_search_active = 0u;              /* 搜索激活标志清零 */
    s_lost_line_cnt = 0u;                   /* 丢线帧计数清零 */
    s_lost_search_cnt = 0u;                 /* 搜索运行帧计数清零 */
}

/* lost_search_pick_dir - 根据丢线前的最后误差选择搜索方向
 * @err: 丢线前的最后位置误差
 * 返回: 1=向右搜索，2=向左搜索
 * err > 死区 → 向右搜索(1)
 * err < -死区 → 向左搜索(2)
 * 在死区内 → 保持上次搜索方向 */
static uint8 lost_search_pick_dir(int16 err) /* 选择丢线搜索方向 */
{
    if (err > LOST_SEARCH_ERR_DEADZONE)      /* 误差偏右，向右搜索 */
        return 1u;                           /* 返回右转方向 */
    if (err < -LOST_SEARCH_ERR_DEADZONE)     /* 误差偏左，向左搜索 */
        return 2u;                           /* 返回左转方向 */

    return (s_lost_search_dir == 2u) ? 2u : 1u; /* 在死区内，保持上次方向 */
}

/* lost_search_step - 丢线搜索主逻辑，每PID周期调用
 * @pos_err: 当前位置误差
 * 返回: 1=正在搜索（本帧已直接输出电机），0=未搜索（继续正常PID）
 *
 * 流程：
 *   1. 若已找回线（line_lost=0且有效行足够）→ 退出搜索
 *   2. 若RA活跃 → 不搜索
 *   3. 若RA flag存在 → 不搜索（让RA处理）
 *   4. 连续丢线达到阈值 → 启动搜索
 *   5. 搜索中定期切换方向，原地左右旋转寻线
 *   6. 搜索时速度清零，只输出差速 */
static uint8 lost_search_step(int16 pos_err) /* 丢线搜索主逻辑 */
{
    /* 退出条件：找回线且有效行足够 */
    if (g_tf.line_lost == 0u &&             /* 未丢线 */
        g_tf.valid_row_count >= LOST_SEARCH_EXIT_VALID_ROWS) /* 有效行足够 */
    {
        s_lost_last_err = pos_err;          /* 记录找回线时的误差 */
        lost_search_reset();                /* 复位搜索状态 */
        return 0u;                          /* 退出搜索，继续正常PID */
    }

    /* RA活跃时不搜索 */
    if (s_ra_state != RA_ST_NONE)           /* RA正在执行 */
    {
        lost_search_reset();                /* 复位搜索状态 */
        return 0u;                          /* 不搜索 */
    }

    /* 有RA flag时不搜索（让RA处理） */
    if (g_ra_flag != 0u || g_ra_pre_flag != 0u) /* 有RA标志 */
        return 0u;                          /* 不搜索，交给RA处理 */

    /* 未丢线时重置计数 */
    if (g_tf.line_lost == 0u)              /* 当前未丢线 */
    {
        s_lost_line_cnt = 0u;              /* 丢线计数清零 */
        return 0u;                          /* 不搜索 */
    }

    /* 累计丢线帧数 */
    if (s_lost_line_cnt < 255u)            /* 防溢出 */
        s_lost_line_cnt++;                 /* 丢线帧计数加1 */

    /* 未达到启动阈值 */
    if (s_lost_line_cnt < LOST_SEARCH_ENTER_FRAMES) /* 丢线帧数不足 */
        return 0u;                          /* 不启动搜索 */

    /* 首次进入搜索：选择方向 */
    if (!s_lost_search_active)              /* 尚未激活搜索 */
    {
        s_lost_search_active = 1u;          /* 激活搜索 */
        s_lost_search_cnt = 0u;             /* 搜索计数清零 */
        s_lost_search_dir = lost_search_pick_dir(s_lost_last_err); /* 根据最后误差选方向 */
    }

    /* 定期切换搜索方向（防止单方向转过头） */
    s_lost_search_cnt++;                    /* 搜索帧计数加1 */
    if (s_lost_search_cnt >= LOST_SEARCH_SWITCH_FRAMES) /* 达到切换阈值 */
    {
        s_lost_search_cnt = 0u;             /* 计数清零 */
        s_lost_search_dir = (s_lost_search_dir == 1u) ? 2u : 1u; /* 左右切换 */
    }

    /* 搜索时清零速度和积分，重置PID状态 */
    base_speed = 0;                         /* 基础速度清零 */
    s_speed_integral = 0.0f;                /* 速度积分清零 */
    reset_speed_planner();                  /* 复位速度规划器 */
    reset_speed_ff_state();                 /* 复位速度前馈 */
    line_pid_reset_derivative();            /* 重置微分状态 */

    /* 输出差速：左转或右转 */
    if (s_lost_search_dir == 1u)            /* 向右搜索 */
    {
        /* 向右搜索：左轮负、右轮正 */
        small_driver_set_duty(clamp_duty(-LOST_SEARCH_DUTY), /* 左轮反转 */
                              clamp_duty(LOST_SEARCH_DUTY)); /* 右轮正转 */
    }
    else                                    /* 向左搜索 */
    {
        /* 向左搜索：左轮正、右轮负 */
        small_driver_set_duty(clamp_duty(LOST_SEARCH_DUTY),  /* 左轮正转 */
                              clamp_duty(-LOST_SEARCH_DUTY)); /* 右轮反转 */
    }

    return 1u;                              /* 正在搜索，本帧已输出电机 */
}

/* ======================== 单边巡线 ======================== */

/* single_edge_reset - 复位单边巡线状态，恢复双边模式
 * 无参数，无返回值 */
static void single_edge_reset(void)         /* 复位单边巡线 */
{
    s_edge_active = 0u;                     /* 激活标志清零 */
    s_edge_side = EDGE_BOTH;                /* 方向恢复双边 */
    s_edge_cnt = 0u;                        /* 帧计数清零 */
    s_edge_target = 0u;                     /* 目标帧数清零 */
    s_edge_until_next_turn = 0u;            /* 清除保持模式 */
    s_edge_release_after_turn = 0u;         /* 清除转弯后释放标志 */
    g_post_edge_side = EDGE_BOTH;           /* 全局方向恢复双边 */
}

/* start_single_edge - 启动单边巡线模式
 * @side: EDGE_LEFT 或 EDGE_RIGHT
 * @duration_ms: 持续时间（毫秒），会被转换为PID周期帧数
 * 非法参数时复位为双边模式 */
void start_single_edge(uint8 side, uint16 duration_ms) /* 启动单边巡线 */
{
    if ((side != EDGE_LEFT && side != EDGE_RIGHT) || duration_ms == 0u) /* 参数合法性检查 */
    {
        single_edge_reset();                /* 参数非法，复位为双边 */
        return;                             /* 返回 */
    }

    s_edge_active = 1u;                     /* 激活单边巡线 */
    s_edge_side = side;                     /* 设置巡线方向 */
    s_edge_cnt = 0u;                        /* 帧计数清零 */

    if (duration_ms == SINGLE_EDGE_UNTIL_NEXT_TURN)
    {
        s_edge_until_next_turn = 1u;        /* 保持到下一次真正转弯 */
        s_edge_release_after_turn = 0u;     /* 尚未遇到需要释放的转弯 */
        s_edge_target = 0u;                 /* 不使用时间退出 */
    }
    else
    {
        s_edge_until_next_turn = 0u;        /* 普通定时单边 */
        s_edge_release_after_turn = 0u;     /* 普通定时模式不使用转弯释放 */
        /* Convert ms to PID ticks, rounded up. */
        s_edge_target = PID_MS_TO_TICKS(duration_ms);
        if (s_edge_target == 0u)            /* 防止转换后为0 */
            s_edge_target = 1u;             /* 保证至少1帧 */
    }

    g_post_edge_side = side;                /* 设置全局单边方向 */
}

/* single_edge_tick - 单边巡线每帧tick，由 line_pid_control() 调用
 * 确保 g_post_edge_side 与设定方向一致，计时到期后自动复位
 * 无参数，无返回值 */
static void single_edge_tick(void)          /* 单边巡线帧更新 */
{
    if (!s_edge_active)                     /* 未激活 */
        return;                             /* 直接返回 */

    /* 防止被外部意外修改 */
    if (g_post_edge_side != s_edge_side)    /* 全局方向与设定不一致 */
        g_post_edge_side = s_edge_side;     /* 恢复为设定方向 */

    if (s_edge_until_next_turn)             /* 保持模式不按时间结束 */
        return;

    s_edge_cnt++;                           /* 帧计数加1 */
    if (s_edge_cnt >= s_edge_target)        /* 达到目标帧数 */
        single_edge_reset();                /* 复位单边巡线 */
}

/* ======================== 转弯屏蔽（Turn Shield） ======================== */

/* turn_shield_start - 启动转弯屏蔽：出弯后屏蔽RA检测一段时间，防止误触发
 * 同时清除当前的RA flag和pre_flag
 * 无参数，无返回值 */
static void turn_shield_start(void)         /* 启动转弯屏蔽 */
{
    s_turn_shield_frames = 1u;              /* 屏蔽帧数初始化为1 */
    s_turn_shield_dist = 0u;               /* 距离清零 */
    g_ra_flag = 0u;                         /* 清除RA flag */
    g_ra_pre_flag = 0u;                     /* 清除RA预检测flag */
    g_ra_pre_dir = 0u;
    g_ra_pre_slow_flag = 0u;
}

/* avg_wheel_speed_abs - 计算左右轮平均速度绝对值（用于转弯屏蔽距离累计）
 * 返回: 左右轮平均速度绝对值 */
static uint16 avg_wheel_speed_abs(void)     /* 计算轮速平均绝对值 */
{
    uint16 l = (uint16)abs_i16(motor_value.receive_left_speed_data);  /* 左轮速度绝对值 */
    uint16 r = (uint16)abs_i16(motor_value.receive_right_speed_data); /* 右轮速度绝对值 */
    return (uint16)(((uint32)l + (uint32)r) / 2u); /* 平均值 */
}

/* turn_shield_update - 转弯屏蔽更新，每PID周期调用
 * 累计轮速距离，屏蔽期间清除RA flag
 * 退出条件：帧数超限 或 帧数+距离都达标
 * 特殊情况：若屏蔽期间检测到路线表中的下一个RA，提前解除屏蔽
 * 无参数，无返回值 */
static void turn_shield_update(void)        /* 转弯屏蔽帧更新 */
{
    if (s_turn_shield_frames == 0u)         /* 屏蔽未激活 */
        return;                             /* 直接返回 */

    /* 累计轮速距离 */
    s_turn_shield_dist += (uint32)avg_wheel_speed_abs(); /* 累加轮速距离 */

    /* 屏蔽期间检测到下一个RA且满足最小帧数 → 提前解除 */
    if (g_ra_flag != 0u &&                  /* 检测到RA flag */
        s_turn_shield_frames >= TURN_SHIELD_NEAR_ALLOW_FRAMES && /* 满足最小帧数 */
        ra_accept_near_flag((uint8)g_ra_flag)) /* 路线表中有匹配或保底可用 */
    {
        g_ra_pre_flag = 0u;                 /* 清除预检测flag */
        g_ra_pre_dir = 0u;
        g_ra_pre_slow_flag = 0u;
        turn_shield_reset();                /* 复位转弯屏蔽 */
        return;                             /* 提前解除 */
    }

    /* 屏蔽期间清除所有RA标志 */
    if (g_ra_flag != 0u || g_ra_pre_flag != 0u) /* 有RA标志 */
    {
        g_ra_flag = 0u;                     /* 清除RA flag */
        g_ra_pre_flag = 0u;                 /* 清除RA预检测flag */
        g_ra_pre_dir = 0u;
        g_ra_pre_slow_flag = 0u;
    }

    /* 退出条件：帧数超限 或 帧数+距离都达标 */
    if (s_turn_shield_frames >= TURN_SHIELD_MAX_FRAMES || /* 帧数超限 */
        (s_turn_shield_frames >= TURN_SHIELD_MIN_FRAMES && /* 满足最小帧数 */
         s_turn_shield_dist >= TURN_SHIELD_DIST_SUM))      /* 满足距离阈值 */
    {
        turn_shield_reset();                /* 复位转弯屏蔽 */
        return;                             /* 退出屏蔽 */
    }

    s_turn_shield_frames++;                 /* 屏蔽帧数加1 */
}

/* ======================== RA状态机退出/恢复 ======================== */

/* ra_finish_ex - RA结束的扩展版本
 * @keep_flag: 结束后保留的RA flag（0=清除，非0=保留给下一个RA）
 * @use_shield: 是否启动转弯屏蔽
 *
 * 逻辑：
 *   1. 关闭LED
 *   2. 重置速度积分和PID微分
 *   3. 提交路线调试信息
 *   4. 检查路线完成状态
 *   5. 根据post_edge配置启动单边巡线
 *   6. 启动转弯屏蔽（可选）
 *   7. 复位RA状态机 */
static void ra_finish_ex(uint8 keep_flag, uint8 use_shield) /* RA结束扩展 */
{
    uint8 edge_side = EDGE_BOTH;            /* 单边巡线方向，初始为双边 */
    uint8 from_recover = (s_ra_phase == RA_PH_RECOVER) ? 1u : 0u; /* 是否从RECOVER退出 */

    turn_right_led_off();                   /* 关闭右转指示LED */
    g_ra_flag = keep_flag;                  /* 设置保留的flag（0=清除） */
    s_speed_integral = 0.0f;               /* 清除速度积分 */
    /* RECOVER退出时保留部分输出，避免突变 */
    if (from_recover)                       /* 从RECOVER阶段退出 */
        line_pid_reset_derivative_keep_output(); /* 部分重置，保留50%输出 */
    else                                    /* 从其他阶段退出 */
        line_pid_reset_derivative();        /* 完全重置微分状态 */
    route_debug_commit();                   /* 提交路线调试信息 */
    update_rules_done();                    /* 检查路线完成状态 */

    if (s_edge_release_after_turn)          /* 已经完成下一次真正转弯 */
        single_edge_reset();                /* 单边任务结束，恢复双边 */

    /* 确定单边巡线方向 */
    if (s_ra_post_edge_side == EDGE_LEFT || s_ra_post_edge_side == EDGE_RIGHT) /* 规则指定了具体方向 */
    {
        /* 规则指定了具体方向 */
        edge_side = s_ra_post_edge_side;    /* 使用规则指定的方向 */
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

    /* 启动转弯屏蔽或清除pre_flag */
    if (use_shield)                         /* 需要转弯屏蔽 */
        turn_shield_start();                /* 启动转弯屏蔽 */
    else                                    /* 不需要转弯屏蔽 */
    {
        g_ra_pre_flag = 0u;                /* 仅清除预检测flag */
        g_ra_pre_dir = 0u;
        g_ra_pre_slow_flag = 0u;
    }
    ra_reset();                             /* 复位RA状态机 */
}

/* ra_finish - RA正常结束：清除flag + 启动转弯屏蔽
 * 无参数，无返回值 */
static void ra_finish(void)                 /* RA正常结束 */
{
    ra_finish_ex(0u, 1u);                   /* 清除flag，启动转弯屏蔽 */
}

/* ra_enter_recover - 进入RECOVER阶段
 * 特点：
 *   - 使用HARD阶段的速度/转向种子值平滑过渡
 *   - 重新初始化PD控制器（以当前误差为初始值）
 *   - 速度前馈就绪（避免启动突变）
 * 无参数，无返回值 */
static void ra_enter_recover(void)          /* 进入RECOVER阶段 */
{
    turn_right_led_off();                   /* 关闭右转指示LED */
    g_ra_flag = 0u;                         /* 清除RA flag */
    g_ra_pre_dir = 0u;
    g_ra_pre_slow_flag = 0u;
    s_ra_phase = RA_PH_RECOVER;             /* 切换到RECOVER阶段 */
    s_ra_phase_cnt = 0u;                    /* 阶段计数清零 */
    s_ra_recover_cnt = 0u;                  /* RECOVER帧计数清零 */
    s_ra_recover_good_cnt = 0u;             /* RECOVER确认计数清零 */
    s_speed_integral = 0.0f;               /* 速度积分清零 */
    s_steer_d_reset_flag = 1u;              /* 设置微分重置标志 */
    /* 以当前视觉误差初始化滤波器，避免恢复时跳变 */
    s_filtered_err = (float)g_tf.error;     /* 用当前误差初始化滤波器 */
    s_steer_last_pos_err = s_filtered_err;  /* 上次滤波误差 = 当前滤波值 */
    s_steer_last_raw_err = (float)g_tf.error; /* 上次原始误差 = 当前误差 */
    s_steer_ff_filtered = 0.0f;             /* 前馈滤波值清零 */
    s_cas_target_filtered = 0.0f;           /* 串级目标滤波值清零 */
    s_cas_last_yaw_err = 0.0f;             /* 串级内环误差清零 */
    /* 从HARD种子值按比例过渡 */
    s_prev_steer_output = s_ra_hard_steer_seed * /* HARD阶段转向种子 */
                          ((float)RA_RECOVER_SEED_STEER_PCT * 0.01f); /* 35%过渡 */
    {
        int16 recover_seed = (int16)((float)motor_speed * 8.0f *
                            ((float)RA_RECOVER_SPEED_PCT * 0.01f));
        int16 hard_seed = (int16)abs_f(s_ra_hard_speed_seed);
        if (recover_seed < hard_seed)
            recover_seed = hard_seed;
        s_base_speed_ramped = recover_seed;
    }
    s_prev_target_speed = (float)s_base_speed_ramped; /* 上一目标速度 = 种子值 */
    s_speed_ff_ready = 1u;                  /* 前馈就绪 */
    ra_debug_update();                      /* 更新调试变量 */
}

/* ra_start - 启动RA状态机
 * @dir: 方向（0=直行 1=右 2=左）
 * @orig_flag: 原始flag值
 * @straight: 是否直行通过
 * @post_edge_side: 结束后单边方向
 * @post_edge_ms: 结束后单边时间
 * 无返回值 */
static void ra_start(uint8 dir, uint8 orig_flag, uint8 straight,
                     uint8 post_edge_side, uint16 post_edge_ms) /* 启动RA */
{
    if (!straight && s_edge_until_next_turn)/* 下一个真正转弯已开始 */
        s_edge_release_after_turn = 1u;     /* 等这个RA结束后恢复双边 */

    s_ra_dir = dir;                         /* 设置转弯方向 */
    s_ra_orig_flag = orig_flag;             /* 保存原始flag */
    s_ra_state = RA_ST_ACTIVE;              /* 状态切换为活跃 */
    /* 直行模式直接进入SLOW阶段，否则从WAIT开始等待拐点 */
    if (straight || base_speed >= RA_FAST_SPEED_START)
        s_ra_phase = RA_PH_SLOW;
    else
        s_ra_phase = RA_PH_WAIT;
    s_ra_ip_row = g_ip_max_row;             /* 记录当前拐点行号 */
    s_ra_straight = straight;               /* 设置直行标志 */
    s_ra_post_edge_side = post_edge_side;   /* 设置结束后单边方向 */
    s_ra_post_edge_ms = post_edge_ms;       /* 设置结束后单边时间 */
    s_ra_exit_good_cnt = 0u;                /* 退出确认计数清零 */
    s_ra_recover_good_cnt = 0u;             /* 恢复确认计数清零 */
    s_ra_approach_cnt = 0u;                 /* 接近计数清零 */
    s_ra_timer = 0u;                        /* 全局计时器清零 */
    s_ra_hard_cnt = 0u;                     /* HARD计数清零 */
    s_ra_recover_cnt = 0u;                  /* RECOVER计数清零 */
    s_ra_phase_cnt = 0u;                    /* 阶段计数清零 */
    s_ra_hard_speed_seed = 0.0f;            /* 速度种子清零 */
    s_ra_hard_steer_seed = 0.0f;            /* 转向种子清零 */
    s_speed_integral = 0.0f;               /* 速度积分清零 */
    reset_speed_planner();                  /* 复位速度规划器 */
    lost_search_reset();                    /* 复位丢线搜索 */

    /* 非直行模式点亮LED指示 */
    if (!straight)                          /* 非直行（需要转弯） */
        turn_right_led_on();                /* 点亮右转指示LED */

    ra_debug_update();                      /* 更新调试变量 */
}

/* ra_enter_hard - 进入HARD阶段（硬转弯）
 * 重置yaw角，准备用固定duty驱动转弯
 * 无参数，无返回值 */
static void ra_enter_hard(void)             /* 进入HARD硬转弯阶段 */
{
    if (s_ra_phase == RA_PH_HARD)           /* 已经在HARD阶段 */
        return;                             /* 避免重复进入 */

    s_ra_phase = RA_PH_HARD;                /* 切换到HARD阶段 */
    s_ra_phase_cnt = 0u;                    /* 阶段计数清零 */
    s_ra_hard_cnt = 0u;                     /* HARD帧计数清零 */
    s_ra_exit_good_cnt = 0u;                /* 退出确认计数清零 */
    s_ra_recover_good_cnt = 0u;             /* 恢复确认计数清零 */
    s_ra_recover_cnt = 0u;                  /* RECOVER计数清零 */
    s_speed_integral = 0.0f;               /* 速度积分清零 */
    line_pid_reset_derivative();            /* 重置微分状态 */

    /* 重置yaw角为0，用于HARD阶段的yaw进度判断 */
    if (imu_ready && !imu_error)            /* IMU正常工作 */
        imu_reset_yaw();                    /* 重置yaw角为0 */

    ra_debug_update();                      /* 更新调试变量 */
}

/* ======================== PID初始化 ======================== */

/* line_pid_init - 初始化所有PID相关状态（系统启动或重新使能时调用）
 * 复位：转向PD、速度PI、RA状态机、单边巡线、丢线搜索、路线计数等
 * 无参数，无返回值 */
void line_pid_init(void)                    /* PID控制器初始化 */
{
    s_steer_last_pos_err = 0.0f;            /* 上次滤波误差清零 */
    s_steer_last_raw_err = 0.0f;            /* 上次原始误差清零 */
    s_filtered_err = 0.0f;                  /* 滤波误差清零 */
    s_prev_steer_output = 0.0f;             /* 上次转向输出清零 */
    s_steer_ff_filtered = 0.0f;             /* 前馈滤波值清零 */
    s_cas_target_filtered = 0.0f;           /* 串级目标滤波值清零 */
    s_cas_last_yaw_err = 0.0f;             /* 串级内环误差清零 */
    s_steer_d_reset_flag = 1u;              /* 设置微分重置标志 */
    s_speed_integral = 0.0f;               /* 速度积分清零 */
    s_motor_run_counter = 0u;               /* 电机运行计数清零 */
    s_vacuum_on = 0u;                       /* 负压状态清零 */
    vacuum_enable = 0u;                     /* 负压显示状态清零 */
    s_rules_done = 0u;                      /* 路线完成标志清零 */
    s_rules_done_timer = 0u;                /* 路线完成计时器清零 */
    g_ra_pre_slow_flag = 0u;                /* 清除预减速专用标志 */
    turn_shield_reset();                    /* 复位转弯屏蔽 */
    single_edge_reset();                    /* 复位单边巡线 */
    lost_search_reset();                    /* 复位丢线搜索 */
    s_lost_last_err = 0;                    /* 丢线最后误差清零 */
    s_lost_search_dir = 1u;                /* 搜索方向默认右 */
    reset_speed_planner();                  /* 复位速度规划器 */
    reset_speed_ff_state();                 /* 复位速度前馈 */
    route_debug_reset();                    /* 复位路线调试 */

    ra_reset();                             /* 复位RA状态机 */

    /* 清零所有路口类型计数 */
    for (uint8 i = 0u; i < 7u; i++)         /* 遍历所有flag类型（0~6） */
        s_inter_count[i] = 0u;             /* 计数清零 */
}

/* ======================== 串级PID转向计算 ======================== */

/* cascade_steer_calc - 串级PID转向计算：图像位置外环 + IMU角速度内环
 * @pos_err: 位置误差
 * @slew_max: 输出变化率限制
 * 返回: 转向输出值
 *
 * 外环：
 *   - 对位置误差做软死区处理（死区内p_scale=0，软区内二次平滑）
 *   - 组合：位置项 + 趋势前馈 + 前瞻前馈 → 目标角速度
 *   - 路口附近抑制趋势/前瞻前馈
 *   - 目标角速度低通滤波
 * 内环：
 *   - 角速度误差 = 目标角速度 - 实际yaw_rate
 *   - 内环PD控制（kp/kd由菜单调节）
 *   - 输出限幅 + 变化率限制 */
static float cascade_steer_calc(int16 pos_err, float slew_max) /* 串级PID转向 */
{
    float err     = (float)pos_err;         /* 位置误差转浮点 */
    float err_abs = abs_f(err);             /* 位置误差绝对值 */

    /* Outer loop: soft deadzone on position error. 外环：软死区处理 */
    float p_scale = 1.0f;                   /* 比例缩放因子，初始为1 */
    if (err_abs <= (float)STEER_DEADZONE)   /* 误差在死区内 */
    {
        p_scale = 0.0f;                     /* 比例项归零 */
    }
    else if (err_abs < (float)STEER_SOFT_END) /* 误差在软死区范围内 */
    {
        float t = (err_abs - (float)STEER_DEADZONE) / /* 当前距离死区起点 */
                  ((float)STEER_SOFT_END - (float)STEER_DEADZONE); /* 除以软区总宽度 */
        p_scale = t * t;                    /* 二次曲线平滑过渡 */
    }

    /* 计算目标角速度 = 位置项 + 趋势前馈 + 前瞻前馈 */
    float yaw_target = CAS_POS_KP  * err * p_scale          /* 位置项：误差×比例 */
                     + CAS_TREND_KD * (float)g_tf.error_trend /* 趋势前馈 */
                     + CAS_LA_K    * (float)g_tf.lookahead_error; /* 前瞻前馈 */

    /* Suppress trend/LA feedforward near intersections. 路口附近抑制前馈 */
    if (g_ra_pre_flag != 0u || g_ra_flag != 0u) /* 有RA标志 */
        yaw_target = CAS_POS_KP * err * p_scale; /* 仅保留位置项 */

    /* Low-pass filter + clamp. 低通滤波 + 限幅 */
    yaw_target = clamp_f(yaw_target, -CAS_TARGET_MAX, CAS_TARGET_MAX); /* 限幅到±120 */
    s_cas_target_filtered = s_cas_target_filtered * CAS_TARGET_FILTER /* 上一帧×0.55 */
                          + yaw_target * (1.0f - CAS_TARGET_FILTER);  /* 新值×0.45 */

    /* Inner loop: IMU yaw_rate closed loop. 内环：IMU角速度闭环 */
    float yaw_err = s_cas_target_filtered - (float)yaw_rate; /* 角速度误差 */
    if (abs_f(yaw_err) < CAS_DEADZONE_DPS)  /* 误差在死区内 */
        yaw_err = 0.0f;                     /* 归零，防抖 */

    float kp_val = (float)yaw_kp * CAS_YAW_KP_SCALE; /* 内环比例增益 */
    float kd_val = (float)yaw_kd * CAS_YAW_KD_SCALE; /* 内环微分增益 */

    float p_out  = kp_val * yaw_err;        /* 内环P项 */
    float d_out  = kd_val * (yaw_err - s_cas_last_yaw_err); /* 内环D项 */
    s_cas_last_yaw_err = yaw_err;           /* 保存当前误差供下帧D项使用 */

    float steer = p_out + d_out;            /* 内环总输出 */
    steer = clamp_f(steer, -STEER_MAX, STEER_MAX); /* 限幅到±4000 */

    /* Slew rate limit (consistent with original PD). 变化率限制 */
    float delta = steer - s_prev_steer_output; /* 输出变化量 */
    if      (delta >  slew_max) steer = s_prev_steer_output + slew_max; /* 正向限幅 */
    else if (delta < -slew_max) steer = s_prev_steer_output - slew_max; /* 负向限幅 */

    s_prev_steer_output = steer;            /* 保存当前输出 */
    return steer;                           /* 返回转向值 */
}

/* ======================== 转向PD控制 ======================== */

/* steer_pd_calc - 转向PD控制器
 * @pos_err: 位置误差（负=偏左，正=偏右）
 * @kp_scale: 比例增益缩放（由调度器计算）
 * @kd_scale: 微分增益缩放（由调度器计算）
 * @feedforward: 前馈信号（基于前瞻误差）
 * @slew_max: 输出变化率限制
 * 返回: 转向输出值（叠加到速度上形成差速）
 *
 * 逻辑：
 *   1. 一阶低通滤波位置误差
 *   2. 死区判断：误差很小且无前馈时输出0
 *   3. 软死区：二次曲线平滑过渡
 *   4. P项 = KP * kp_scale * 滤波误差 * p_scale
 *   5. D项 = KD * kd_scale * (当前原始误差 - 上次原始误差)
 *   6. 输出 = P + D + 前馈
 *   7. 限幅 + 变化率限制 */
static float steer_pd_calc(int16 pos_err,           /* 位置误差 */
                           float kp_scale,           /* 比例增益缩放 */
                           float kd_scale,           /* 微分增益缩放 */
                           float feedforward,        /* 前馈信号 */
                           float slew_max)           /* 变化率限制 */
{
    /* 一阶IIR低通滤波：alpha=0.55，新值权重0.45 */
    s_filtered_err = s_filtered_err * ERROR_FILTER_ALPHA + /* 上一帧滤波值×0.55 */
                     (float)pos_err * (1.0f - ERROR_FILTER_ALPHA); /* 新误差×0.45 */

    float err = s_filtered_err;             /* 滤波后的误差 */
    float err_abs = abs_f(err);             /* 滤波误差绝对值 */
    float raw_err = (float)pos_err;         /* 原始误差（未滤波） */

    /* 死区：误差极小且前馈也很小时，直接返回0（防抖） */
    if (err_abs <= (float)STEER_DEADZONE && abs_f(feedforward) <= 1.0f) /* 死区+前馈极小 */
    {
        s_steer_last_pos_err = err;         /* 保存滤波误差 */
        s_steer_last_raw_err = raw_err;     /* 保存原始误差 */
        s_prev_steer_output *= 0.5f;        /* 衰减上次输出（防止积累） */
        return 0.0f;                        /* 输出0，防抖 */
    }

    /* 软死区：死区到软区间用二次曲线平滑 */
    float p_scale = 1.0f;                   /* 比例缩放因子，初始为1 */
    if (err_abs <= (float)STEER_DEADZONE)   /* 误差在死区内 */
    {
        p_scale = 0.0f;                     /* 比例项归零 */
    }
    else if (err_abs < (float)STEER_SOFT_END) /* 误差在软死区范围内 */
    {
        float t = (err_abs - (float)STEER_DEADZONE) / /* 当前距离死区起点 */
                  ((float)STEER_SOFT_END - (float)STEER_DEADZONE); /* 除以软区总宽度 */
        p_scale = t * t;                    /* 二次曲线平滑 */
    }

    /* P项计算 */
    float p_out = STEER_KP * kp_scale * err * p_scale; /* 比例输出 */
    float d_out = 0.0f;                     /* 微分输出，初始为0 */

    /* D项：若未被重置则正常计算，否则跳过一帧 */
    if (s_steer_d_reset_flag == 0u)         /* 微分未被重置 */
        d_out = STEER_KD * kd_scale * (err - s_steer_last_pos_err) * PID_D_SCALE; /* 微分输出 */
    else                                    /* 微分被重置 */
        s_steer_d_reset_flag = 0u;          /* 清除重置标志，下帧恢复D项 */

    s_steer_last_pos_err = err;             /* 保存当前滤波误差 */
    s_steer_last_raw_err = raw_err;         /* 保存当前原始误差 */

    /* 总输出 = P + D + 前馈 */
    float output = p_out + d_out + feedforward; /* 三项相加 */

    /* 输出限幅 */
    if (output > STEER_MAX) output = STEER_MAX;         /* 正向限幅 */
    else if (output < -STEER_MAX) output = -STEER_MAX;  /* 负向限幅 */

    /* 变化率限制（防止转向突变） */
    float delta = output - s_prev_steer_output; /* 输出变化量 */
    if (delta > slew_max)                   /* 正向变化过大 */
        output = s_prev_steer_output + slew_max; /* 限制正向变化率 */
    else if (delta < -slew_max)             /* 负向变化过大 */
        output = s_prev_steer_output - slew_max; /* 限制负向变化率 */

    s_prev_steer_output = output;           /* 保存当前输出 */
    return output;                          /* 返回转向值 */
}

/* ======================== 速度PI控制 ======================== */

/* speed_pi_calc - 速度PI控制器
 * @target: 目标速度
 * @actual: 实际速度（左右轮平均）
 * @integral: 积分指针
 * @pos_err_abs: 位置误差绝对值（用于积分分离）
 * 返回: 速度PI输出
 *
 * 积分分离：位置误差大时不积分（防止弯道超调）
 * 积分限幅：防止积分饱和 */
static float speed_pi_calc(float target, float actual, float *integral, int16 pos_err_abs) /* 速度PI */
{
    float speed_err = target - actual;      /* 速度误差 = 目标 - 实际 */

    /* 积分分离：位置误差 < 阈值时才积分 */
    if (pos_err_abs < SPEED_I_SEPARATION)   /* 位置误差小于分离阈值(20) */
    {
        *integral += speed_err * PID_DT_SCALE; /* 累加积分项 */

        /* 积分限幅 */
        if (*integral > SPEED_I_MAX) *integral = SPEED_I_MAX;       /* 正向积分限幅 */
        else if (*integral < -SPEED_I_MAX) *integral = -SPEED_I_MAX; /* 负向积分限幅 */
    }

    return SPEED_KP * speed_err + SPEED_KI * (*integral); /* P项 + I项 */
}

/* ======================== 误差自适应速度 ======================== */

/* calc_adapted_speed - 根据位置误差自适应调整速度
 * @base: 基础速度
 * @pos_err_abs: 位置误差绝对值
 * 返回: 自适应后的速度
 *
 * 逻辑：误差小于t1时用ratio_1%，大于t2时用ratio_2%，中间线性插值
 * 通常ratio_1 > ratio_2（直道快，弯道慢） */
static int16 calc_adapted_speed(int16 base, int16 pos_err_abs) /* 误差自适应速度 */
{
    int16 t1 = sp_err_t1;                  /* 误差阈值下限（菜单可调） */
    int16 t2 = sp_err_t2;                  /* 误差阈值上限（菜单可调） */
    int16 r1 = sp_ratio_1;                 /* 低误差速度百分比（菜单可调） */
    int16 r2 = sp_ratio_2;                 /* 高误差速度百分比（菜单可调） */

    if (t2 <= t1)                           /* 阈值无效（上限≤下限） */
        t2 = t1 + 1;                       /* 修正为下限+1 */

    if (pos_err_abs <= t1)                  /* 误差小于下限 */
        return (int16)((int32)base * r1 / 100); /* 用高百分比（直道快） */

    if (pos_err_abs >= t2)                  /* 误差大于上限 */
        return (int16)((int32)base * r2 / 100); /* 用低百分比（弯道慢） */

    /* 中间线性插值 */
    int32 ratio = (int32)r1 + ((int32)(r2 - r1) * (pos_err_abs - t1)) / (t2 - t1); /* 线性插值 */
    return (int16)((int32)base * ratio / 100); /* 用插值百分比 */
}

/* ======================== 直道判定 ======================== */

/* straight_speed_candidate - 判断当前是否为直道加速候选
 * @pos_err_abs: 位置误差绝对值
 * 条件：未丢线、有效行足够、误差小、前瞻小、趋势小、无RA
 * 返回 1=是直道候选 */
static uint8 straight_speed_candidate(int16 pos_err_abs) /* 判断直道加速候选 */
{
    if (g_tf.line_lost != 0u)               /* 丢线 */
        return 0u;                          /* 不是候选 */
    if (g_tf.valid_row_count < SPEED_STRAIGHT_VALID_ROWS) /* 有效行不足(35) */
        return 0u;                          /* 不是候选 */
    if (pos_err_abs > SPEED_STRAIGHT_ERR_MAX) /* 位置误差过大(>6) */
        return 0u;                          /* 不是候选 */
    if (abs_i16(g_tf.lookahead_error) > SPEED_STRAIGHT_LOOKAHEAD_MAX) /* 前瞻误差过大(>8) */
        return 0u;                          /* 不是候选 */
    if (abs_i16(g_tf.error_trend) > SPEED_STRAIGHT_TREND_MAX) /* 趋势误差过大(>8) */
        return 0u;                          /* 不是候选 */
    if (g_ra_pre_flag != 0u || g_ra_flag != 0u) /* 有RA标志 */
        return 0u;                          /* 不是候选 */

    return 1u;                              /* 满足所有条件，是直道候选 */
}

/* sym_straight_speed_candidate - 判断当前是否为对称组件直道加速候选
 * @pos_err_abs: 位置误差绝对值
 * 对称组件（如三极管干扰区）被检测到时，说明赛道较直
 * 条件：有对称组件、未丢线、有效行足够、误差适中、无RA
 * 返回 1=是候选 */
static uint8 sym_straight_speed_candidate(int16 pos_err_abs) /* 对称组件直道候选 */
{
    if (g_sym_component_flag == 0u)         /* 未检测到对称组件 */
        return 0u;                          /* 不是候选 */
    if (g_tf.line_lost != 0u)               /* 丢线 */
        return 0u;                          /* 不是候选 */
    if (g_tf.valid_row_count < SPEED_SYM_VALID_ROWS) /* 有效行不足(30) */
        return 0u;                          /* 不是候选 */
    if (pos_err_abs > SPEED_SYM_ERR_MAX)   /* 位置误差过大(>12) */
        return 0u;                          /* 不是候选 */
    if (g_ra_pre_flag != 0u || g_ra_flag != 0u) /* 有RA标志 */
        return 0u;                          /* 不是候选 */

    return 1u;                              /* 满足所有条件，是候选 */
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
 * 真正转弯RA或预减速阶段不加速。 */
static uint8 single_edge_speed_candidate(int16 pos_err_abs)
{
    if (s_edge_active == 0u)
        return 0u;
    if (g_post_edge_side == EDGE_BOTH)
        return 0u;
    if (g_tf.line_lost != 0u)
        return 0u;
    if (g_tf.valid_row_count < SPEED_SINGLE_EDGE_VALID_ROWS)
        return 0u;
    if (pos_err_abs > SPEED_SINGLE_EDGE_ERR_MAX)
        return 0u;
    if (abs_i16(g_tf.lookahead_error) > SPEED_SINGLE_EDGE_LOOKAHEAD_MAX)
        return 0u;
    if (abs_i16(g_tf.error_trend) > SPEED_SINGLE_EDGE_TREND_MAX)
        return 0u;
    if (s_ra_state == RA_ST_ACTIVE && s_ra_straight == 0u)
        return 0u;
    if (s_ra_state == RA_ST_NONE &&
        (g_ra_pre_flag != 0u || g_ra_pre_slow_flag != 0u || g_ra_flag != 0u))
        return 0u;

    return 1u;
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
        target = 0;                         /* 限制为0 */

    /* 首次调用（负值表示未初始化），直接跳到目标 */
    if (s_base_speed_ramped < 0)            /* 首次调用 */
    {
        s_base_speed_ramped = target;       /* 直接设为目标 */
        return s_base_speed_ramped;         /* 返回 */
    }

    /* 按步长渐变 */
    if (target > s_base_speed_ramped + up_step) /* 需要加速 */
        s_base_speed_ramped += up_step;         /* 按上升步长增加 */
    else if (target < s_base_speed_ramped - down_step) /* 需要减速 */
        s_base_speed_ramped -= down_step;       /* 按下降步长减少 */
    else                                    /* 差值在步长范围内 */
        s_base_speed_ramped = target;       /* 直接设为目标 */

    return s_base_speed_ramped;             /* 返回斜坡处理后的速度 */
}

/* speed_ramp_apply_reason - 带调试信息的速度斜坡处理
 * @target: 目标速度
 * @reason: 速度变化原因编号
 * 直道/单边加速时使用更大的上升步长
 * 返回: 斜坡处理后的速度 */
static int16 speed_ramp_apply_reason(int16 target, uint8 reason) /* 带原因的斜坡 */
{
    int16 up_step = SPEED_RAMP_UP_STEP;     /* 上升步长 */
    int16 down_step = SPEED_RAMP_DOWN_STEP;

    speed_dbg_plan = target;                /* 记录规划速度 */
    speed_dbg_reason = reason;              /* 记录原因编号 */

    if (target < 0)                         /* 目标速度为负 */
        target = 0;                         /* 限制为0 */

    /* 直道/单边加速原因(4/6/9)使用更快的上升步长 */
    if (reason == 4u || reason == 6u || reason == 9u)
        up_step = SPEED_RAMP_STRAIGHT_UP_STEP; /* 使用更大的上升步长 */

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

    /* 按步长渐变 */
    if (target > s_base_speed_ramped + up_step) /* 需要加速 */
        s_base_speed_ramped += up_step;          /* 按上升步长增加 */
    else if (target < s_base_speed_ramped - down_step) /* 需要减速 */
        s_base_speed_ramped -= down_step; /* 按下降步长减少 */
    else                                    /* 差值在步长范围内 */
        s_base_speed_ramped = target;       /* 直接设为目标 */

    return s_base_speed_ramped;             /* 返回斜坡处理后的速度 */
}

/* ======================== 速度百分比计算 ======================== */

/* apply_speed_pct - 将目标速度乘以百分比缩放
 * @target: 目标速度
 * @pct: 百分比（0~120，100=不变）
 * 返回: 缩放后的速度 */
static int16 apply_speed_pct(int16 target, int16 pct) /* 速度百分比缩放 */
{
    if (pct < 0) pct = 0;                  /* 百分比下限为0 */
    if (pct > 120) pct = 120;              /* 百分比上限为120 */

    return (int16)((int32)target * pct / 100); /* 计算百分比 */
}

/* calc_signal_speed_pct - 根据信号值计算降速百分比
 * @signal: 信号值（如前瞻误差、趋势误差）
 * @t1: 信号阈值下限（低于此不降速）
 * @t2: 信号阈值上限（高于此最大降速）
 * @min_pct: 最大降速百分比
 * 返回: 速度百分比（100=不降，min_pct=最大降） */
static int16 calc_signal_speed_pct(int16 signal, int16 t1, int16 t2, int16 min_pct) /* 信号降速 */
{
    if (t2 <= t1)                           /* 阈值无效 */
        t2 = t1 + 1;                       /* 修正 */

    if (signal <= t1)                       /* 信号低于下限 */
        return 100;                         /* 不降速 */

    if (signal >= t2)                       /* 信号高于上限 */
        return min_pct;                     /* 最大降速 */

    /* 中间线性插值 */
    return (int16)(100 - ((int32)(100 - min_pct) * (signal - t1)) / (t2 - t1)); /* 线性插值 */
}

/* ======================== 前瞻/趋势降速 ======================== */

/* plan_lookahead_speed - 根据前瞻误差和趋势误差计算目标速度
 * @target: 原始目标速度
 * @pos_err_abs: 位置误差绝对值
 * 取前瞻降速和趋势降速中较小的（降速更多）的值
 * 误差极大时还叠加误差自适应降速
 * 返回: 降速后的目标速度 */
static int16 plan_lookahead_speed(int16 target, int16 pos_err_abs) /* 前瞻/趋势降速 */
{
    int16 la_abs = abs_i16(g_tf.lookahead_error); /* 前瞻误差绝对值 */
    int16 trend_abs = abs_i16(g_tf.error_trend);   /* 趋势误差绝对值 */
    /* 前瞻降速百分比 */
    int16 la_pct = calc_signal_speed_pct(la_abs,          /* 前瞻信号 */
                                         SPEED_LOOKAHEAD_SLOW_T1, /* 下限8 */
                                         SPEED_LOOKAHEAD_SLOW_T2, /* 上限28 */
                                         SPEED_LOOKAHEAD_MIN_PCT); /* 最低55% */
    /* 趋势降速百分比 */
    int16 trend_pct = calc_signal_speed_pct(trend_abs,     /* 趋势信号 */
                                            SPEED_TREND_SLOW_T1,   /* 下限6 */
                                            SPEED_TREND_SLOW_T2,   /* 上限24 */
                                            SPEED_TREND_MIN_PCT);  /* 最低60% */
    /* 取较小值（降速更多） */
    int16 pct = (la_pct < trend_pct) ? la_pct : trend_pct; /* 取降速更多的 */

    /* 误差极大时叠加误差自适应降速 */
    if (pos_err_abs > sp_err_t2)            /* 误差超过自适应上限 */
        pct = (pct < sp_ratio_2) ? pct : sp_ratio_2; /* 取更小值 */

    return apply_speed_pct(target, pct);    /* 返回降速后的目标 */
}

/* ======================== 视觉质量降速 ======================== */

/* plan_quality_speed - 根据视觉质量规划速度
 * @target: 原始目标速度
 * 降速因素：
 *   1. 有效行数少 → 降速（有效行越少，赛道信息越不可靠）
 *   2. 误差跳变大 → 降速（帧间误差突变说明跟踪不稳定）
 * 取两者中降速更多的值
 * 返回: 降速后的目标速度 */
static int16 plan_quality_speed(int16 target) /* 视觉质量降速 */
{
    int16 pct = 100;                        /* 初始百分比100%（不降速） */

    /* 有效行数不足时降速 */
    if (g_tf.valid_row_count < SPEED_QUALITY_GOOD_ROWS) /* 有效行不足(35) */
    {
        int16 row_pct = SPEED_QUALITY_ROW_MIN_PCT; /* 最低百分比(70%) */
        uint16 span = SPEED_QUALITY_GOOD_ROWS - SPEED_VISION_BAD_VALID_ROWS; /* 区间宽度 */

        if (g_tf.valid_row_count <= SPEED_VISION_BAD_VALID_ROWS) /* 有效行极少(≤18) */
        {
            /* 有效行极少，最大降速 */
            row_pct = SPEED_VISION_BAD_PCT; /* 最大降速(40%) */
        }
        else if (span > 0u)                 /* 区间有效 */
        {
            /* 线性插值 */
            row_pct = (int16)(SPEED_VISION_BAD_PCT + /* 40% + */
                      ((int32)(SPEED_QUALITY_ROW_MIN_PCT - SPEED_VISION_BAD_PCT) *
                      (int32)(g_tf.valid_row_count - SPEED_VISION_BAD_VALID_ROWS)) /
                      (int32)span);          /* 线性插值计算 */
        }

        if (row_pct < pct)                  /* 行降速更少 */
            pct = row_pct;                  /* 更新百分比 */
    }

    /* 误差跳变检测：当前帧与上帧误差差值 */
    if (s_prev_quality_err_valid)            /* 上帧误差有效 */
    {
        int16 err_jump = abs_i16((int16)(g_tf.error - s_prev_quality_err)); /* 误差跳变量 */
        int16 jump_pct = calc_signal_speed_pct(err_jump,   /* 跳变信号 */
                                               SPEED_ERR_JUMP_T1,   /* 下限10 */
                                               SPEED_ERR_JUMP_T2,   /* 上限28 */
                                               SPEED_ERR_JUMP_MIN_PCT); /* 最低60% */
        if (jump_pct < pct)                 /* 跳变降速更多 */
            pct = jump_pct;                 /* 更新百分比 */
    }

    s_prev_quality_err = g_tf.error;        /* 保存当前误差供下帧比较 */
    s_prev_quality_err_valid = 1u;          /* 标记上帧误差有效 */
    speed_dbg_vq_pct = (uint8)pct;          /* 记录视觉质量百分比 */

    return apply_speed_pct(target, pct);    /* 返回降速后的目标 */
}

/* ======================== 转向差速限制 ======================== */

/* limit_normal_steer - 限制转向输出（差速）的幅度
 * @steer: 转向值
 * @speed_out: 速度输出值
 * 差速限制 = 速度输出 * 百分比，再限幅到 [min, max]
 * 不同模式使用不同百分比：正常95%、直道75%、RECOVER 120%
 * 返回: 限幅后的转向值 */
static float limit_normal_steer(float steer, float speed_out) /* 转向差速限制 */
{
    int16 pct = STEER_DIFF_NORMAL_PCT;      /* 默认百分比 */

    if (s_straight_active)                  /* 直道模式 */
        pct = STEER_DIFF_STRAIGHT_PCT;      /* 直道百分比75% */

    if (s_ra_state == RA_ST_ACTIVE && s_ra_phase == RA_PH_RECOVER) /* RECOVER阶段 */
        pct = STEER_DIFF_RECOVER_PCT;       /* RECOVER百分比120% */

    float limit = abs_f(speed_out) * (float)pct * 0.01f; /* 差速限幅值 = 速度×百分比 */
    limit = clamp_f(limit, STEER_DIFF_MIN_LIMIT, STEER_DIFF_MAX_LIMIT); /* 限幅到[250, 1800] */

    return clamp_f(steer, -limit, limit);   /* 将转向值限幅到±limit */
}

/* ======================== RECOVER yaw修正 ======================== */

/* recover_yaw_correction - RECOVER阶段的yaw角修正
 * 在RECOVER阶段根据目标yaw角与实际yaw角的差值，输出一个修正量
 * 帮助车辆在出弯后更快回到正确朝向
 * 只在RECOVER阶段且IMU正常时有效
 * 返回: yaw修正量（正=右转修正，负=左转修正） */
static float recover_yaw_correction(void)   /* RECOVER yaw修正 */
{
    float target;                           /* 目标yaw角 */
    float yaw_err;                          /* yaw角误差 */
    float correction;                       /* 修正量 */

    if (s_ra_state != RA_ST_ACTIVE || s_ra_phase != RA_PH_RECOVER) /* 不在RECOVER阶段 */
        return 0.0f;                        /* 不修正 */
    if (s_ra_dir == 0u || !imu_ready || imu_error) /* 无方向或IMU异常 */
        return 0.0f;                        /* 不修正 */

    /* 目标yaw角：右转目标正角，左转目标负角 */
    target = (s_ra_dir == 1u) ?             /* 右转 */
             RA_RECOVER_YAW_TARGET_DEG :    /* 目标+84度 */
             -RA_RECOVER_YAW_TARGET_DEG;    /* 左转目标-84度 */
    yaw_err = normalize_angle(target - yaw_angle); /* yaw角误差（归一化） */

    /* 死区内不修正 */
    if (abs_f(yaw_err) <= RA_RECOVER_YAW_DEADZONE) /* 误差在死区内(±3度) */
        return 0.0f;                        /* 不修正 */

    /* P控制修正 */
    correction = -yaw_err * RA_RECOVER_YAW_KP; /* 修正量 = -误差×比例(4.2) */
    return clamp_f(correction, -RA_RECOVER_YAW_MAX, RA_RECOVER_YAW_MAX); /* 限幅到±280 */
}

/* ======================== 速度规划主函数 ======================== */

/* plan_base_speed - 基础速度规划（每PID周期调用）
 * @target: 原始目标速度
 * @pos_err_abs: 位置误差绝对值
 * @pre_slow_active: 预减速激活标志
 * 返回: 规划后的基础速度（经斜坡处理）
 *
 * 规划优先级：
 *   1. RA活跃或预减速 → 直接用目标速度
 *   2. 丢线 → 25%速度
 *   3. 有效行极少 → 40%速度
 *   4. 单边稳定通过 → 120%加速（用于最后T右直行）
 *   5. 元器件直线 → 120%加速（跳过质量/前瞻降速）
 *   6. 视觉质量降速（有效行/误差跳变）
 *   7. 对称组件直道 → 120%加速（立即）
 *   8. 前瞻/趋势降速
 *   9. 直道确认 → 120%加速（需连续N帧确认） */
static int16 plan_base_speed(int16 target, int16 pos_err_abs, uint8 pre_slow_active) /* 速度规划 */
{
    s_straight_active = 0u;                 /* 直道激活标志清零 */
    speed_dbg_vq_pct = 100u;               /* 视觉质量百分比重置 */

    /* RA活跃或预减速期间，不做额外规划 */
    if (s_ra_state != RA_ST_NONE || pre_slow_active) /* RA活跃或预减速 */
    {
        s_straight_cnt = 0u;                /* 直道计数清零 */

        if (!pre_slow_active && single_edge_speed_candidate(pos_err_abs))
        {
            s_straight_cnt = SPEED_STRAIGHT_CONFIRM_FRAMES;
            s_straight_active = 1u;
            target = (int16)((int32)target * SPEED_SINGLE_EDGE_BOOST_PCT / 100);
            return speed_ramp_apply_reason(target, 9u);
        }

        return speed_ramp_apply_reason(target, 1u); /* 直接用目标速度，原因=RA */
    }

    /* 丢线时大幅降速 */
    if (g_tf.line_lost != 0u)              /* 丢线 */
    {
        s_straight_cnt = 0u;                /* 直道计数清零 */
        speed_dbg_vq_pct = SPEED_LINE_LOST_PCT; /* 记录降速百分比(25%) */
        target = (int16)((int32)target * SPEED_LINE_LOST_PCT / 100); /* 降速到25% */
        return speed_ramp_apply_reason(target, 2u); /* 返回，原因=丢线 */
    }

    /* 有效行极少时降速 */
    if (g_tf.valid_row_count < SPEED_VISION_BAD_VALID_ROWS) /* 有效行极少(≤18) */
    {
        s_straight_cnt = 0u;                /* 直道计数清零 */
        speed_dbg_vq_pct = SPEED_VISION_BAD_PCT; /* 记录降速百分比(40%) */
        target = (int16)((int32)target * SPEED_VISION_BAD_PCT / 100); /* 降速到40% */
        return speed_ramp_apply_reason(target, 3u); /* 返回，原因=视觉差 */
    }

    if (component_fast_speed_candidate(pos_err_abs))
    {
        s_straight_cnt = SPEED_STRAIGHT_CONFIRM_FRAMES;
        s_straight_active = 1u;
        target = (int16)((int32)target * SPEED_STRAIGHT_BOOST_PCT / 100);
        return speed_ramp_apply_reason(target, 4u);
    }

    if (single_edge_speed_candidate(pos_err_abs))
    {
        s_straight_cnt = SPEED_STRAIGHT_CONFIRM_FRAMES;
        s_straight_active = 1u;
        target = (int16)((int32)target * SPEED_SINGLE_EDGE_BOOST_PCT / 100);
        return speed_ramp_apply_reason(target, 9u);
    }

    /* 视觉质量降速（有效行不足或误差跳变） */
    target = plan_quality_speed(target);    /* 视觉质量降速 */
    uint8 reason = (speed_dbg_vq_pct < 100u) ? 7u : 8u; /* 有降速=原因7，无降速=原因8 */

    /* 对称组件直道加速（立即生效，无需确认） */
    if (sym_straight_speed_candidate(pos_err_abs)) /* 是对称组件直道候选 */
    {
        s_straight_cnt = SPEED_STRAIGHT_CONFIRM_FRAMES; /* 直接确认 */
        s_straight_active = 1u;             /* 激活直道模式 */
        target = (int16)((int32)target * SPEED_STRAIGHT_BOOST_PCT / 100); /* 直道加速 */
        return speed_ramp_apply_reason(target, 4u); /* 返回，原因=对称加速 */
    }

    /* 前瞻/趋势降速 */
    {
        int16 before_lookahead = target;    /* 保存降速前的目标 */
        target = plan_lookahead_speed(target, pos_err_abs); /* 前瞻/趋势降速 */
        if (target < before_lookahead)      /* 确实降速了 */
            reason = 5u;                    /* 原因=前瞻降速 */
    }

    /* 直道加速确认（需连续N帧满足条件） */
    if (straight_speed_candidate(pos_err_abs)) /* 是直道候选 */
    {
        if (s_straight_cnt < 255u)          /* 防溢出 */
            s_straight_cnt++;               /* 直道计数加1 */

        if (s_straight_cnt >= SPEED_STRAIGHT_CONFIRM_FRAMES) /* 连续确认帧达标(4) */
        {
            s_straight_active = 1u;         /* 激活直道模式 */
            target = (int16)((int32)target * SPEED_STRAIGHT_BOOST_PCT / 100); /* 直道加速 */
            reason = 6u;                    /* 原因=直道加速 */
        }
    }
    else                                    /* 不满足直道条件 */
    {
        s_straight_cnt = 0u;                /* 直道计数清零 */
    }

    return speed_ramp_apply_reason(target, reason); /* 返回斜坡处理后的速度 */
}

/* ======================== 路口动作推断 ======================== */

/* route_action_from_flag - 根据flag类型推断默认转弯动作
 * @flag: 路口类型flag
 * 返回: 动作类型
 * flag=3(T左) → 左转
 * flag=4(T右) → 右转
 * flag=1(右直角) → 右转
 * flag=2(左直角) → 左转
 * 其他 → 直行 */
static uint8 route_action_from_flag(uint8 flag) /* 根据flag推断动作 */
{
    if (flag == 3u)                         /* T左路口 */
        return ACT_LEFT;                    /* 左转 */
    if (flag == 4u)                         /* T右路口 */
        return ACT_RIGHT;                   /* 右转 */
    if (flag == 1u)                         /* 右直角 */
        return ACT_RIGHT;                   /* 右转 */
    if (flag == 2u)                         /* 左直角 */
        return ACT_LEFT;                    /* 左转 */

    return ACT_STRAIGHT;                    /* 其他默认直行 */
}

/* fallback_intersection_decision - 保底路口决策（路线表未匹配时的默认行为）
 * @flag: 路口类型flag
 * 返回: 路口决策结构体 */
static RouteDecision fallback_intersection_decision(uint8 flag) /* 保底路口决策 */
{
    RouteDecision d = { ACT_STRAIGHT, EDGE_BOTH, 0u, 0u }; /* 初始化为直行，无单边 */

    d.action = route_action_from_flag(flag); /* 根据flag推断动作 */
    return d;                               /* 返回决策 */
}

/* ======================== 路线表匹配 ======================== */

/* select_intersection_decision - 根据当前flag从路线表中选择动作
 * @flag: 当前检测到的路口类型
 * 返回: 路口决策（包含动作、单边配置、有效标志）
 *
 * 流程：
 *   1. 先查路线表中flag+count匹配的条目
 *   2. 匹配到 → 消耗该条目，返回对应动作
 *   3. 未匹配 → 检查是否启用保底直转
 *   4. 都没有 → 返回默认决策（flag=1/2不消耗，flag=3/4/5不动作） */
static RouteDecision select_intersection_decision(uint8 flag) /* 路线表匹配 */
{
    RouteDecision d = fallback_intersection_decision(flag); /* 初始化保底决策 */
    uint8 next_count;                       /* 下一个期望的count */

    if (flag >= 7u)                         /* flag越界 */
        return d;                           /* 返回保底决策 */

    next_count = s_inter_count[flag] + 1u;  /* 下一个期望的count值 */

    /* 遍历路线表查找匹配 */
    for (uint8 i = 0u; i < USER_RULE_COUNT; i++) /* 遍历路线表 */
    {
        if (user_rules[i].flag == flag &&   /* flag类型匹配 */
            user_rules[i].count == next_count) /* count值匹配 */
        {
            /* 消耗该条目 */
            s_inter_count[flag] = next_count; /* 更新已匹配计数 */

            /* ACT_AUTO时根据flag推断动作 */
            d.action = (user_rules[i].action == ACT_AUTO) ? /* 自动模式 */
                       route_action_from_flag(flag) :        /* 根据flag推断 */
                       user_rules[i].action;                 /* 使用规则指定动作 */
            d.post_edge_side = user_rules[i].post_edge_side; /* 获取单边方向 */
            d.post_edge_ms = user_rules[i].post_edge_ms;    /* 获取单边时间 */
            d.valid = 1u;                   /* 标记决策有效 */

            /* 保存待提交的调试信息 */
            s_route_pending_valid = 1u;     /* 待提交有效 */
            s_route_pending_flag = flag;    /* 待提交flag */
            s_route_pending_count = next_count; /* 待提交count */
            s_route_pending_action = d.action; /* 待提交动作 */
            return d;                       /* 返回决策 */
        }
    }

    /* 路线表未匹配，检查保底直转 */
    if (ra_fallback_direct_enabled(flag))   /* 保底直转已启用 */
    {
        d.action = route_action_from_flag(flag); /* 根据flag推断动作 */
        d.post_edge_side = EDGE_BOTH;       /* 无单边 */
        d.post_edge_ms = 0u;                /* 无单边时间 */
        d.valid = 1u;                       /* 标记决策有效 */
        return d;                           /* 返回决策 */
    }

    /* 未匹配路线表时：直角1/2保底自动转，普通路口不消耗计数。 */
    return d;                               /* 返回保底决策 */
}

/* ======================== RA状态机主逻辑 ======================== */

/* ra_state_machine_step - RA状态机每帧执行
 * @pos_err_abs: 位置误差绝对值
 * 返回: RaResult（need_pid/should_return/speed_scale）
 *
 * 状态转换：
 *   检测到flag(1/2) → 从路线表选动作 → ra_start()
 *   检测到flag(3/4/5) → 从路线表选动作 → ra_start()
 *   WAIT阶段 → 等待拐点行达到阈值 → SLOW
 *   SLOW阶段 → 等待拐点行达到转弯阈值 → APPROACH
 *   APPROACH阶段 → 等待N帧 → HARD
 *   HARD阶段 → 固定duty驱动 → 摄像头恢复/yaw达标/超时 → RECOVER
 *   RECOVER阶段 → PID恢复巡线 → 确认稳定 → 结束 */
static RaResult ra_state_machine_step(int16 pos_err_abs) /* RA状态机主逻辑 */
{
    RaResult r = { 0u, 0u, 1.0f };          /* 初始化：需要PID，不跳过，速度缩放100% */

    if (s_ra_state == RA_ST_NONE &&
        g_ra_flag == 0u &&
        g_ra_pre_flag != 0u &&
        (g_ra_pre_dir == 1u || g_ra_pre_dir == 2u) &&
        route_next_rule_is(g_ra_pre_dir) &&
        base_speed >= RA_FAST_SPEED_START)
    {
        uint8 pre_flag = g_ra_pre_dir;
        RouteDecision d = select_intersection_decision(pre_flag);
        uint8 action = (d.action == ACT_RIGHT || d.action == ACT_LEFT) ?
                       d.action :
                       route_action_from_flag(pre_flag);

        if (d.valid)
        {
            g_ra_flag = pre_flag;
            ra_start((action == ACT_RIGHT) ? 1u : 2u,
                     pre_flag,
                     0u,
                     d.post_edge_side,
                     d.post_edge_ms);
            s_ra_phase = RA_PH_SLOW;
            s_ra_phase_cnt = 0u;
            s_ra_approach_cnt = 0u;
        }
    }

    /* 检测到直角flag(1/2)且RA空闲 → 启动RA */
    if ((g_ra_flag == 1u || g_ra_flag == 2u) && s_ra_state == RA_ST_NONE) /* 直角flag且空闲 */
    {
        RouteDecision d = select_intersection_decision((uint8)g_ra_flag); /* 路线表匹配 */
        uint8 action = (d.action == ACT_RIGHT || d.action == ACT_LEFT) ? /* 有明确方向 */
                       d.action :             /* 使用决策动作 */
                       route_action_from_flag((uint8)g_ra_flag); /* 从flag推断方向 */

        /* 无效决策 → 清除flag，不启动RA */
        if (!d.valid)                        /* 决策无效 */
        {
            g_ra_flag = 0u;                  /* 清除flag */
            ra_debug_update();               /* 更新调试 */
            return r;                        /* 返回默认结果 */
        }

        ra_start((action == ACT_RIGHT) ? 1u : 2u, /* 方向：1=右 2=左 */
                 (uint8)g_ra_flag,           /* 原始flag */
                 0u,                         /* 非直行 */
                 d.post_edge_side,           /* 结束后单边方向 */
                 d.post_edge_ms);            /* 结束后单边时间 */
    }

    /* 检测到普通路口flag(3/4/5)且RA空闲 → 启动RA */
    if ((g_ra_flag >= 3u && g_ra_flag <= 5u) && s_ra_state == RA_ST_NONE) /* 普通路口flag且空闲 */
    {
        RouteDecision d = select_intersection_decision((uint8)g_ra_flag); /* 路线表匹配 */

        /* 无效决策 → 清除flag */
        if (!d.valid)                        /* 决策无效 */
        {
            g_ra_flag = 0u;                  /* 清除flag */
            ra_debug_update();               /* 更新调试 */
            return r;                        /* 返回默认结果 */
        }

        if (d.action == ACT_RIGHT || d.action == ACT_LEFT) /* 有转弯方向 */
            ra_start((d.action == ACT_RIGHT) ? 1u : 2u, (uint8)g_ra_flag, 0u, /* 启动转弯RA */
                     d.post_edge_side, d.post_edge_ms);
        else                                /* 直行通过 */
            /* 直行通过：dir=0, straight=1 */
            ra_start(0u, (uint8)g_ra_flag, 1u, /* 启动直行RA */
                     d.post_edge_side, d.post_edge_ms);
    }

    /* RA未激活 → 返回默认值 */
    if (s_ra_state != RA_ST_ACTIVE)          /* RA未激活 */
    {
        ra_debug_update();                   /* 更新调试 */
        return r;                            /* 返回默认结果 */
    }

    /* ===== RA活跃状态 ===== */
    s_ra_timer++;                            /* 全局计时器加1 */
    s_ra_phase_cnt++;                        /* 阶段计数加1 */

    /* 更新最大拐点行号 */
    if (g_ip_max_row > s_ra_ip_row)          /* 当前拐点行更大 */
        s_ra_ip_row = g_ip_max_row;         /* 更新最大拐点行 */

    /* 全局超时保护 */
    if (s_ra_timer > RA_TIMEOUT_FRAMES)      /* 超过超时阈值(150帧=1.65秒) */
    {
        ra_finish();                         /* 强制结束RA */
        return r;                            /* 返回 */
    }

    /* 直行通过模式：固定帧数后结束 */
    if (s_ra_straight)                       /* 直行通过模式 */
    {
        if (s_ra_timer >= RA_STRAIGHT_FRAMES) /* 达到直行帧数(55) */
        {
            ra_finish();                     /* 结束RA */
            return r;                        /* 返回 */
        }

        /* 单边直行通过时不按RA慢速跑，交给速度规划做单边加速 */
        if (s_edge_active && g_post_edge_side != EDGE_BOTH)
            r.speed_scale = 1.0f;
        else
            r.speed_scale = (float)ra_slow_pct * 0.01f; /* 速度缩放到ra_slow_pct% */
        r.need_pid = 1u;                     /* 需要PID控制 */
        ra_debug_update();                   /* 更新调试 */
        return r;                            /* 返回 */
    }

    /* ===== RECOVER阶段 ===== */
    if (s_ra_phase == RA_PH_RECOVER)          /* RECOVER阶段 */
    {
        /* 恢复完成条件：线跟踪稳定（未丢线、有效行够、误差小） */
        uint8 recover_ok = (g_tf.line_lost == 0u &&        /* 未丢线 */
                            g_tf.valid_row_count >= RA_RECOVER_VALID_ROWS && /* 有效行≥25 */
                            pos_err_abs <= RA_RECOVER_ERROR_MAX && /* 误差≤12 */
                            abs_i16(g_tf.lookahead_error) <= RA_RECOVER_LOOKAHEAD_MAX && /* 前瞻≤14 */
                            abs_i16(g_tf.error_trend) <= RA_RECOVER_TREND_MAX) ? 1u : 0u; /* 趋势≤16 */

        s_ra_recover_cnt++;                  /* RECOVER帧计数加1 */

        /* RECOVER期间检测到下一个RA → 提前结束，保留flag给下一个RA */
        if (g_ra_flag != 0u &&               /* 检测到RA flag */
            s_ra_recover_cnt >= RA_RECOVER_NEAR_DETECT_MIN_FRAMES && /* 满足最小帧数(1) */
            ra_accept_near_flag((uint8)g_ra_flag)) /* 路线表匹配或保底可用 */
        {
            uint8 next_flag = (uint8)g_ra_flag; /* 保存下一个flag */
            ra_finish_ex(next_flag, 0u);     /* 结束RA，保留flag，无转弯屏蔽 */
            r.speed_scale = (float)RA_RECOVER_SPEED_PCT * 0.01f; /* 速度缩放到100% */
            r.need_pid = 1u;                 /* 需要PID */
            ra_debug_update();               /* 更新调试 */
            return r;                        /* 返回 */
        }

        /* 累计/重置恢复计数 */
        if (recover_ok) s_ra_recover_good_cnt++; /* 恢复条件满足，计数加1 */
        else s_ra_recover_good_cnt = 0u;    /* 不满足，计数清零 */

        /* 退出条件：满足最小帧数且连续确认帧达标，或超时 */
        if ((s_ra_recover_cnt >= RA_RECOVER_MIN_FRAMES && /* 满足最小帧数(4) */
             s_ra_recover_good_cnt >= RA_RECOVER_CONFIRM_FRAMES) || /* 连续确认帧达标(2) */
            s_ra_recover_cnt >= RA_RECOVER_MAX_FRAMES) /* 超时(14帧) */
        {
            ra_finish();                     /* 结束RA */
            return r;                        /* 返回 */
        }

        /* RECOVER期间降速+PID巡线 */
        r.speed_scale = (float)RA_RECOVER_SPEED_PCT * 0.01f; /* 速度缩放到100% */
        r.need_pid = 1u;                     /* 需要PID */
        ra_debug_update();                   /* 更新调试 */
        return r;                            /* 返回 */
    }

    /* ===== WAIT阶段 ===== */
    if (s_ra_phase == RA_PH_WAIT)             /* WAIT阶段 */
    {
        uint8 wait_timeout = (s_ra_orig_flag < 3u &&
                              s_ra_phase_cnt >= RA_WAIT_TIMEOUT) ? 1u : 0u;

        /* 拐点行达到减速阈值或超时 → 进入SLOW */
        if (s_ra_ip_row >= (uint8)ra_slow_row || wait_timeout) /* 条件满足 */
        {
            s_ra_phase = RA_PH_SLOW;         /* 切换到SLOW阶段 */
            s_ra_phase_cnt = 0u;             /* 阶段计数清零 */
            s_speed_integral = 0.0f;        /* 速度积分清零 */
        }
    }
    /* ===== SLOW阶段 ===== */
    else if (s_ra_phase == RA_PH_SLOW)        /* SLOW阶段 */
    {
        uint8 turn_row = (uint8)ra_turn_row;
        uint8 slow_timeout = (s_ra_orig_flag < 3u &&
                              s_ra_phase_cnt >= RA_SLOW_TIMEOUT) ? 1u : 0u;

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

        /* 拐点行达到转弯阈值或超时 → 进入APPROACH */
        if (s_ra_ip_row >= turn_row || slow_timeout) /* 条件满足 */
        {
            s_ra_phase = RA_PH_APPROACH;     /* 切换到APPROACH阶段 */
            s_ra_approach_cnt = 0u;          /* 接近计数清零 */
            s_ra_phase_cnt = 0u;             /* 阶段计数清零 */
            s_speed_integral = 0.0f;        /* 速度积分清零 */
        }
    }
    /* ===== APPROACH阶段 ===== */
    else if (s_ra_phase == RA_PH_APPROACH)    /* APPROACH阶段 */
    {
        /* 等待指定帧数后进入HARD */
        s_ra_approach_cnt++;                 /* 接近帧计数加1 */
        if (s_ra_approach_cnt >= (uint16)ra_approach_frames) /* 达到接近帧数 */
            ra_enter_hard();                 /* 进入HARD阶段 */
    }

    /* ===== HARD阶段 ===== */
    if (s_ra_phase == RA_PH_HARD)             /* HARD硬转弯阶段 */
    {
        /* 是否使用快速直角模式（高速+纯直角时更激进） */
        uint8 direct_fast = (s_ra_orig_flag < 3u && /* 纯直角（flag=1或2） */
                             base_speed >= RA_FAST_SPEED_START) ? 1u : 0u; /* 速度≥520 */
        /* 最小HARD帧数：路口需要更少帧 */
        uint8 min_hard = (s_ra_orig_flag >= 3u) ? /* 普通路口(3/4/5) */
                         RA_CROSS_HARD_MIN_FRAMES : RA_HARD_MIN_FRAMES; /* 路口10帧，直角12帧 */
        /* HARD超时帧数 */
        uint8 hard_limit = (s_ra_orig_flag >= 3u) ? /* 普通路口 */
                           RA_CROSS_HARD_TIMEOUT :  /* 路口超时18帧 */
                           (direct_fast ? RA_FAST_HARD_TIMEOUT : RA_HARD_TIMEOUT); /* 快速22帧，普通16帧 */
        /* 摄像头恢复条件 */
        uint8 line_ok = (g_tf.line_lost == 0u &&      /* 未丢线 */
                         g_tf.valid_row_count >= RA_EXIT_VALID_ROWS && /* 有效行≥12 */
                         pos_err_abs <= RA_EXIT_ERROR_MAX) ? 1u : 0u; /* 误差≤18 */

        s_ra_hard_cnt++;                     /* HARD帧计数加1 */

        /* 累计/重置摄像头恢复计数 */
        if (line_ok) s_ra_exit_good_cnt++;   /* 摄像头条件满足，计数加1 */
        else s_ra_exit_good_cnt = 0u;        /* 不满足，计数清零 */

        uint8 yaw_done = 0u;                /* yaw退出标志 */
        /* 根据模式选择yaw目标角度 */
        float hard_yaw_target = (s_ra_orig_flag >= 3u) ? /* 普通路口 */
                                RA_CROSS_HARD_YAW_DEG :  /* 路口yaw目标58度 */
                                (direct_fast ? RA_FAST_DIRECT_YAW_DEG : (float)ra_hard_yaw); /* 快速82度或菜单值 */
        float yaw_progress = ra_yaw_progress(); /* 当前yaw进度 */
        /* 摄像头恢复退出条件 */
        uint8 camera_done = (s_ra_hard_cnt >= min_hard && /* 满足最小HARD帧数 */
                             s_ra_exit_good_cnt >= RA_EXIT_CONFIRM_FRAMES) ? 1u : 0u; /* 连续确认帧(3) */

        /* 直角模式：yaw未达到最低角度时不允许摄像头提前退出 */
        if (s_ra_orig_flag < 3u && imu_ready && !imu_error &&
            yaw_progress < (direct_fast ? RA_FAST_CAMERA_MIN_YAW : RA_DIRECT_CAMERA_MIN_YAW))
        {
            camera_done = 0u;                /* 禁止摄像头退出 */
        }

        if (s_ra_orig_flag >= 3u && imu_ready && !imu_error &&
            yaw_progress < RA_CROSS_CAMERA_MIN_YAW)
        {
            camera_done = 0u;
        }

        /* yaw角度退出条件 */
        if (imu_ready && !imu_error && hard_yaw_target > 0.0f && /* IMU正常且目标有效 */
            s_ra_hard_cnt >= min_hard &&     /* 满足最小HARD帧数 */
            yaw_progress >= hard_yaw_target) /* yaw进度达到目标 */
        {
            yaw_done = 1u;                   /* 设置yaw退出标志 */
        }

        if (hard_limit < min_hard)           /* 超时帧数不应小于最小帧数 */
            hard_limit = min_hard;           /* 修正为最小帧数 */

        uint8 hard_timeout = (s_ra_hard_cnt > hard_limit) ? 1u : 0u; /* 是否超时 */

        /* 任一退出条件满足 → 进入RECOVER */
        if (camera_done || yaw_done || hard_timeout) /* 摄像头恢复/yaw达标/超时 */
        {
            ra_enter_recover();              /* 进入RECOVER阶段 */
            return r;                        /* 返回 */
        }

        /* 计算内外轮duty */
        float outer = (float)ra_hard_outer;  /* 外侧轮duty（菜单可调） */
        float inner = (float)ra_hard_inner;  /* 内侧轮duty（菜单可调） */
        float out_l;                         /* 左轮输出 */
        float out_r;                         /* 右轮输出 */

        if (s_ra_orig_flag >= 3u)
        {
            outer = outer * (float)RA_COMPLEX_DUTY_PCT * 0.01f;
            inner = inner * (float)RA_COMPLEX_DUTY_PCT * 0.01f;
        }

        /* 右转：左轮=内侧（慢），右轮=外侧（快） */
        if (s_ra_dir == 1u)                  /* 右转 */
        {
            out_l = inner;                   /* 左轮=内侧 */
            out_r = outer;                   /* 右轮=外侧 */
        }
        else                                /* 左转 */
        {
            out_l = outer;                   /* 左轮=外侧 */
            out_r = inner;                   /* 右轮=内侧 */
        }

        /* HARD初期斜坡启动（前几帧逐渐增大，防止打滑） */
        if (s_ra_hard_cnt <= RA_HARD_RAMP_FRAMES) /* 前5帧 */
        {
            float ramp = (float)s_ra_hard_cnt / (float)RA_HARD_RAMP_FRAMES; /* 斜坡比例 */
            if (ramp < 0.20f)               /* 最小20% */
                ramp = 0.20f;               /* 限制最小值 */
            out_l *= ramp;                   /* 左轮按斜坡缩放 */
            out_r *= ramp;                   /* 右轮按斜坡缩放 */
        }

        /* 记录种子值供RECOVER阶段使用 */
        s_ra_hard_speed_seed = (out_l + out_r) * 0.5f; /* 速度种子 = 左右平均 */
        s_ra_hard_steer_seed = (out_l - out_r) * 0.5f; /* 转向种子 = 左右差/2 */
        speed_dbg_out = (int16)s_ra_hard_speed_seed;   /* 记录速度调试值 */
        steer_dbg_out = (int16)s_ra_hard_steer_seed;   /* 记录转向调试值 */

        /* 直接输出电机duty */
        small_driver_set_duty(clamp_duty(out_l), clamp_duty(out_r)); /* 输出限幅后的duty */
        r.should_return = 1u;                /* 标记已直接输出，跳过PID */
        ra_debug_update();                   /* 更新调试 */
        return r;                            /* 返回 */
    }

    /* SLOW/APPROACH阶段降速 */
    if (s_ra_phase == RA_PH_SLOW || s_ra_phase == RA_PH_APPROACH) /* SLOW或APPROACH阶段 */
        r.speed_scale = (float)ra_slow_pct * 0.01f; /* 速度缩放到ra_slow_pct% */

    r.need_pid = 1u;                         /* 需要PID控制 */
    ra_debug_update();                       /* 更新调试 */
    return r;                                /* 返回结果 */
}

/* ======================== 正常PID执行 ======================== */

/* normal_pid_step - 正常巡线PID控制（非HARD阶段时调用）
 * @pos_err: 位置误差
 * @pos_err_abs: 位置误差绝对值
 *
 * 流程：
 *   1. 计算转向增益调度参数
 *   2. 计算前馈信号（基于前瞻误差）
 *   3. 选择cascade或普通PD计算转向
 *   4. 计算自适应速度 + 速度PI + 速度前馈
 *   5. 速度因子缩放转向（高速时温和补偿）
 *   6. 直道/RECOVER模式转向缩放
 *   7. 差速限幅
 *   8. 输出：左轮=速度+转向，右轮=速度-转向 */
static void normal_pid_step(int16 pos_err, int16 pos_err_abs) /* 正常PID控制 */
{
    /* 计算增益调度参数 */
    SteerSchedule sch = steer_schedule_calc(pos_err_abs); /* 根据速度和弯道计算增益 */

    /* 前馈信号：KP * ff_scale * 前瞻误差 */
    float ff_raw = STEER_KP * sch.ff_scale * (float)g_tf.lookahead_error; /* 原始前馈 */
    float steer_ff = 0.0f;                  /* 有效前馈值，初始为0 */

    ff_raw = clamp_f(ff_raw, -STEER_FF_MAX, STEER_FF_MAX); /* 前馈限幅 */
    /* 前馈低通滤波 */
    s_steer_ff_filtered = s_steer_ff_filtered * STEER_FF_FILTER_ALPHA + /* 上一帧×0.55 */
                          ff_raw * (1.0f - STEER_FF_FILTER_ALPHA);      /* 新值×0.45 */

    /* 路口附近抑制前馈 */
    if (g_ra_pre_flag == 0u && g_ra_flag == 0u) /* 无RA标志 */
        steer_ff = s_steer_ff_filtered;     /* 使用滤波后的前馈 */

    float steer;                            /* 转向输出值 */

    /* 选择转向控制模式：cascade（IMU内环）或普通PD */
    if (cascade_en && imu_ready && !imu_error && /* 串级启用且IMU正常 */
        !(s_ra_state == RA_ST_ACTIVE && s_ra_phase == RA_PH_RECOVER)) /* 且不在RECOVER阶段 */
    {
        steer = cascade_steer_calc(pos_err, sch.slew_max); /* 使用串级PID */
    }
    else                                    /* 使用普通PD */
    {
        steer = steer_pd_calc(pos_err,      /* 位置误差 */
                              sch.kp_scale, /* 比例增益缩放 */
                              sch.kd_scale, /* 微分增益缩放 */
                              steer_ff,     /* 前馈信号 */
                              sch.slew_max); /* 变化率限制 */

#if YAW_COMP_ENABLE                         /* yaw角补偿（默认关闭） */
        /* yaw角补偿（默认关闭） */
        {
            float yaw_kp_val = (float)yaw_kp / 10.0f; /* yaw比例增益 */
            float yaw_abs = abs_f(yaw_angle);          /* yaw角绝对值 */
            if (yaw_abs > YAW_DEADZONE)                 /* 超过死区(1度) */
                steer += yaw_kp_val * yaw_angle;        /* 添加yaw补偿 */
        }
#endif

        /* Independent yaw damping (cascade_en=0 only). 独立yaw阻尼 */
        if (yaw_damp_gain != 0 && imu_ready && !imu_error && /* 阻尼增益非零且IMU正常 */
            !(s_ra_state == RA_ST_ACTIVE && s_ra_phase == RA_PH_RECOVER)) /* 不在RECOVER */
        {
            steer -= (float)yaw_damp_gain * YAW_DAMP_SCALE * (float)yaw_rate; /* 减去阻尼项 */
        }
    }

    /* ===== 速度计算 ===== */
    int16 trend_abs = abs_i16(g_tf.error_trend); /* 趋势误差绝对值 */
    int16 speed_err = pos_err_abs;          /* 速度误差基准 = 位置误差 */
    uint8 component_fast = component_fast_speed_candidate(pos_err_abs);

    /* 高速时将趋势误差叠加到速度误差中（高速弯道更谨慎） */
    if (base_speed > 200)                   /* 速度超过200 */
    {
        float trend_factor = (float)(base_speed - 200) / 800.0f; /* 趋势因子 */
        if (trend_factor > 0.5f) trend_factor = 0.5f; /* 上限50% */
        speed_err = pos_err_abs + (int16)((float)trend_abs * trend_factor); /* 叠加趋势 */
    }

    /* 自适应速度目标 */
    float target_speed = component_fast ?
                         (float)base_speed :
                         (float)calc_adapted_speed(base_speed, speed_err); /* 误差自适应速度 */
    float actual_l = (float)motor_value.receive_left_speed_data;  /* 左轮实际速度 */
    float actual_r = (float)motor_value.receive_right_speed_data; /* 右轮实际速度 */
    float avg_actual = (actual_l + actual_r) * 0.5f; /* 左右轮平均速度 */

    /* 加速度前馈：目标速度变化量 * 增益 */
    float accel_ff = 0.0f;                  /* 加速度前馈，初始为0 */
    if (s_speed_ff_ready)                   /* 前馈就绪（非首周期） */
        accel_ff = (target_speed - s_prev_target_speed) * PID_D_SCALE * SPEED_ACCEL_FF_GAIN; /* 目标变化量前馈 */
    else                                    /* 首周期 */
        s_speed_ff_ready = 1u;              /* 标记前馈就绪 */

    accel_ff = clamp_f(accel_ff, -SPEED_ACCEL_FF_LIMIT, SPEED_ACCEL_FF_LIMIT); /* 限幅加速度前馈 */
    s_prev_target_speed = target_speed;     /* 保存当前目标速度 */

    /* 速度输出 = 前馈 + PI */
    float speed_ff = target_speed * SPEED_FF_RATIO + accel_ff; /* 速度前馈 + 加速度前馈 */
    float speed_out = speed_ff + speed_pi_calc(target_speed,   /* 前馈 + 速度PI */
                                               avg_actual,     /* 实际速度 */
                                               &s_speed_integral, /* 积分指针 */
                                               pos_err_abs);  /* 位置误差（积分分离用） */

    /* 速度因子：高速时温和增加转向补偿 */
    int16 turn_signal = pos_err_abs;
    int16 la_abs_for_turn = abs_i16(g_tf.lookahead_error);
    if (la_abs_for_turn > turn_signal) turn_signal = la_abs_for_turn;
    if (trend_abs > turn_signal) turn_signal = trend_abs;

    float curve_boost = range_ratio_i16(turn_signal,
                                        STEER_GAIN_CURVE_T1,
                                        STEER_GAIN_CURVE_T2) * 0.35f;
    float speed_factor = 1.0f +
                         (float)base_speed * (float)steer_speed_k * 0.00025f +
                         curve_boost;       /* 速度因子 */
    if (speed_factor > SPEED_FACTOR_MAX)    /* 超过上限 */
        speed_factor = SPEED_FACTOR_MAX;    /* 限幅 */

    steer *= speed_factor;                  /* 转向乘以速度因子 */

    /* 直道时转向缩放 */
    if (s_straight_active)                  /* 直道加速模式 */
        steer *= (float)SPEED_STRAIGHT_STEER_PCT * 0.01f; /* 转向缩放(100%) */

    /* RECOVER阶段：转向缩放 + yaw修正 */
    if (s_ra_state == RA_ST_ACTIVE && s_ra_phase == RA_PH_RECOVER) /* RECOVER阶段 */
    {
        steer *= (float)RA_RECOVER_STEER_PCT * 0.01f; /* 转向缩放到140% */
        steer += recover_yaw_correction();   /* 添加yaw修正 */
    }

    /* 差速限幅 */
    steer = limit_normal_steer(steer, speed_out); /* 限制差速幅度 */
    speed_dbg_out = (int16)speed_out;       /* 记录速度调试值 */
    steer_dbg_out = (int16)steer;           /* 记录转向调试值 */

    /* 最终输出：左轮=速度+转向，右轮=速度-转向 */
    small_driver_set_duty(clamp_duty(speed_out + steer), /* 左轮 = 速度+转向 */
                          clamp_duty(speed_out - steer)); /* 右轮 = 速度-转向 */
}

/* line_pid_control - 主PID控制函数，由PID_PERIOD_MS PIT中断调用（isr.c:cc60_pit_ch0_isr）
 * 这是整个控制系统的核心，每PID_PERIOD_MS执行一次
 *
 * 执行流程：
 *   1. 电机未使能 → 清零所有状态并返回
 *   2. 运行超时 → 停机
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

    /* ===== 电机未使能：全部复位 ===== */
    if (motor_enable == 0)                   /* 电机未使能 */
    {
        small_driver_set_duty(0, 0);         /* 电机输出清零 */
        vacuum_set(0u);                      /* 关闭负压 */
        base_speed = 0;                      /* 基础速度清零 */
        speed_dbg_out = 0;                   /* 速度调试清零 */
        steer_dbg_out = 0;                   /* 转向调试清零 */
        speed_dbg_raw = 0;                   /* 原始速度清零 */
        speed_dbg_plan = 0;                  /* 规划速度清零 */
        speed_dbg_reason = 0u;               /* 速度原因清零 */
        s_speed_integral = 0.0f;            /* 速度积分清零 */
        s_motor_run_counter = 0u;            /* 运行计数清零 */
        turn_shield_reset();                 /* 复位转弯屏蔽 */
        single_edge_reset();                 /* 复位单边巡线 */
        lost_search_reset();                 /* 复位丢线搜索 */
        reset_speed_planner();               /* 复位速度规划器 */
        reset_speed_ff_state();              /* 复位速度前馈 */
        g_ra_pre_slow_flag = 0u;             /* 清除预减速专用标志 */
        ra_reset();                          /* 复位RA状态机 */
        return;                              /* 返回 */
    }

    /* ===== 运行超时保护 ===== */
    s_motor_run_counter++;                   /* 运行帧计数加1 */
    if (s_motor_run_counter >= PID_SECONDS_TO_TICKS((uint32)motor_run_time)) /* 超过运行时间 */
    {
        motor_enable = 0;                    /* 关闭电机使能 */
        small_driver_set_duty(0, 0);         /* 电机输出清零 */
        vacuum_set(0u);                      /* 关闭负压 */
        base_speed = 0;                      /* 基础速度清零 */
        speed_dbg_out = 0;                   /* 速度调试清零 */
        steer_dbg_out = 0;                   /* 转向调试清零 */
        speed_dbg_raw = 0;                   /* 原始速度清零 */
        speed_dbg_plan = 0;                  /* 规划速度清零 */
        speed_dbg_reason = 0u;               /* 速度原因清零 */
        s_speed_integral = 0.0f;            /* 速度积分清零 */
        s_motor_run_counter = 0u;            /* 运行计数清零 */
        turn_shield_reset();                 /* 复位转弯屏蔽 */
        single_edge_reset();                 /* 复位单边巡线 */
        lost_search_reset();                 /* 复位丢线搜索 */
        reset_speed_planner();               /* 复位速度规划器 */
        reset_speed_ff_state();              /* 复位速度前馈 */
        g_ra_pre_slow_flag = 0u;             /* 清除预减速专用标志 */
        ra_reset();                          /* 复位RA状态机 */
        return;                              /* 返回 */
    }

    /* ===== 路线完成延迟停机 ===== */
    if (s_rules_done)                        /* 路线全部完成 */
    {
        s_rules_done_timer++;                /* 延迟计时器加1 */
        if (s_rules_done_timer >= RULES_DONE_DELAY) /* 达到延迟帧数(136=1.5秒) */
        {
            motor_enable = 0;                /* 关闭电机使能 */
            small_driver_set_duty(0, 0);     /* 电机输出清零 */
            vacuum_set(0u);                  /* 关闭负压 */
            base_speed = 0;                  /* 基础速度清零 */
            speed_dbg_out = 0;               /* 速度调试清零 */
            steer_dbg_out = 0;               /* 转向调试清零 */
            speed_dbg_raw = 0;               /* 原始速度清零 */
            speed_dbg_plan = 0;              /* 规划速度清零 */
            speed_dbg_reason = 0u;           /* 速度原因清零 */
            s_speed_integral = 0.0f;        /* 速度积分清零 */
            s_motor_run_counter = 0u;        /* 运行计数清零 */
            s_rules_done = 0u;               /* 路线完成标志清零 */
            s_rules_done_timer = 0u;         /* 延迟计时器清零 */
            turn_shield_reset();             /* 复位转弯屏蔽 */
            single_edge_reset();             /* 复位单边巡线 */
            lost_search_reset();             /* 复位丢线搜索 */
            reset_speed_planner();           /* 复位速度规划器 */
            reset_speed_ff_state();          /* 复位速度前馈 */
            g_ra_pre_slow_flag = 0u;         /* 清除预减速专用标志 */
            ra_reset();                      /* 复位RA状态机 */
            return;                          /* 返回 */
        }
    }

#if VACUUM_RUN_ENABLE
    vacuum_set(1u);
#else
    vacuum_set(0u);
#endif

    /* ===== 转弯屏蔽更新 ===== */
    turn_shield_update();                    /* 更新转弯屏蔽状态 */

    int16 pos_err = g_tf.error;             /* 获取当前位置误差 */
    int16 pos_err_abs = abs_i16(pos_err);   /* 位置误差绝对值 */

    /* ===== RA状态机 ===== */
    RaResult ra = ra_state_machine_step(pos_err_abs); /* 执行RA状态机 */
    /* HARD阶段RA已直接输出电机，本帧不再执行PID */
    if (ra.should_return)                    /* RA已直接输出电机 */
        return;                              /* 跳过本帧PID */

    /* ===== 丢线搜索 ===== */
    /* 丢线搜索已直接输出电机，本帧不再执行PID */
    if (lost_search_step(pos_err))           /* 丢线搜索正在执行 */
        return;                              /* 跳过本帧PID */

    /* ===== 速度规划 ===== */
    /* 原始目标速度 = 菜单速度 * 8 * RA速度缩放 */
    int16 target_base_speed = (int16)((float)motor_speed * 8.0f * ra.speed_scale); /* 原始目标速度 */
    uint8 pre_slow_active = 0u;              /* 预减速激活标志 */
    speed_dbg_pre_lock = 0u;                 /* 预减速锁调试标志清零 */
    speed_dbg_raw = target_base_speed;       /* 记录原始目标速度 */

    /* ===== 预减速逻辑 ===== */
    /* 当 right_angle_pre_detect() 检测到直角但还未正式触发RA时
     * 提前降低速度，防止冲出赛道 */
    if (s_ra_state == RA_ST_NONE)            /* RA未激活 */
    {
        /* 预减速锁定标志和超时计数（局部静态变量） */
        static uint8 s_pre_lock = 0u;        /* 预减速锁定标志 */
        static uint8 s_pre_timeout = 0u;     /* 预减速超时计数 */

        /* 转弯屏蔽期间不锁定预减速 */
        if (s_turn_shield_frames != 0u)      /* 转弯屏蔽激活中 */
        {
            s_pre_lock = 0u;                 /* 清除锁定 */
            s_pre_timeout = 0u;              /* 超时清零 */
        }

        /* 检测到方向预判或远场预减速信号，且无正式flag → 锁定预减速 */
        if ((g_ra_pre_flag || g_ra_pre_slow_flag) && g_ra_flag == 0u)
        {
            s_pre_lock = 1u;                 /* 锁定预减速 */
            s_pre_timeout = 0u;              /* 超时清零 */
        }

        /* 正式flag到来 → 解除预减速（让RA接管） */
        if (g_ra_flag != 0u)                 /* 有正式RA flag */
        {
            s_pre_lock = 0u;                 /* 解除锁定 */
            g_ra_pre_slow_flag = 0u;         /* 正式RA接管后清除预减速专用标志 */
        }

        /* 对称组件（三极管干扰区） → 解除预减速 */
        if (g_sym_component_flag)            /* 检测到对称组件 */
        {
            s_pre_lock = 0u;                 /* 解除锁定 */
            s_pre_timeout = 0u;              /* 超时清零 */
            g_ra_pre_slow_flag = 0u;         /* 防止元器件远场特征锁住预减速 */
        }

        /* 锁定状态下，预减速信号消失后超时解除 */
        if (s_pre_lock)                      /* 处于锁定状态 */
        {
            if (!g_ra_pre_flag && !g_ra_pre_slow_flag)
            {
                /* 恢复直道 → 解除 */
                if (straight_speed_candidate(pos_err_abs)) /* 满足直道条件 */
                {
                    s_pre_lock = 0u;         /* 解除锁定 */
                    s_pre_timeout = 0u;      /* 超时清零 */
                }
                else                         /* 不满足直道条件 */
                {
                    /* 超时3帧后解除 */
                    s_pre_timeout++;         /* 超时计数加1 */
                    if (s_pre_timeout > 3u)  /* 超过3帧 */
                    {
                        s_pre_lock = 0u;     /* 解除锁定 */
                        s_pre_timeout = 0u;  /* 超时清零 */
                    }
                }
            }

            /* 仍在锁定 → 降速 */
            if (s_pre_lock)                  /* 仍在锁定 */
            {
                pre_slow_active = 1u;        /* 标记预减速激活 */
                target_base_speed = (int16)((int32)target_base_speed * ra_slow_pct / 100); /* 降速 */
            }
        }

        speed_dbg_pre_lock = s_pre_lock;     /* 记录预减速锁状态 */
    }

    /* 规划基础速度（含各种降速/加速策略） */
    base_speed = plan_base_speed(target_base_speed, pos_err_abs, pre_slow_active); /* 速度规划 */

    /* 单边巡线倒计时 */
    single_edge_tick();                      /* 更新单边巡线状态 */
    /* 执行正常PID控制（转向+速度） */
    normal_pid_step(pos_err, pos_err_abs);   /* 执行正常PID控制 */
}
