#include "Menu.h"              /* 包含菜单模块头文件，提供MenuItem、MenuPage等结构体定义 */
#include "Track_funsion.h"     /* 包含视觉融合模块头文件，提供阈值偏移、拐点检测等外部变量 */
#include "Pid.h"               /* 包含PID控制模块头文件，提供电机运行时间等外部变�?*/
#include "IMU.h"
#include "Battery.h"
#include "TFT_show_image.h"    /* 包含TFT显示模块头文件，提供draw_line()等显示函�?*/

/* ==================== 按键与拨码开关GPIO引脚定义 ==================== */
#define KEY1 P11_3           /* 按键1：上一�?/ 翻页（低电平有效，内部上拉） */
#define KEY2 P11_2           /* 按键2：下一�?/ 翻页（低电平有效，内部上拉） */
#define KEY3 P20_6           /* 按键3：选择模式�?光标上移，调节模式下=增加�?*/
#define KEY4 P20_7           /* 按键4：选择模式�?光标下移，调节模式下=减少�?*/

#define TURN_RIGHT_LED P20_8 /* 右转指示LED引脚，低电平点亮 */
#define RACE_KEY P33_10
#define RACE_LED_STOP P33_4
#define RACE_LED_READY P33_5
#define RACE_LED_RUN P33_6
#define SWITCH1 P33_11       /* 拨码开�?：与SWITCH2配合决定选择/调节模式 */
#define SWITCH2 P33_12       /* 拨码开�?：与SWITCH1配合决定选择/调节模式 */
                             /* 两个开关均未拨�?选择模式，任一拨下=调节模式 */

/* ==================== 按键消抖与显示刷新参�?==================== */
#define DEBOUNCE_CNT 3       /* 按键消抖计数阈值，达到此值才确认按键按下（每次扫�?1�?*/
#define RELEASE_CNT 3        /* 按键释放消抖计数阈值，确认按键已松开 */
#define KEY_REPEAT_START_CNT 20u /* 长按开始连调前的等待计�?*/
#define KEY_REPEAT_INTERVAL_CNT 4u /* 长按连调间隔计数 */
#define MAIN_DRAW_DIV_STOP 1u  /* 停止状态下主页TFT刷新分频系数�?=每个新视觉帧都刷新） */
#define MAIN_DRAW_DIV_RUN  4u  /* 运行状态下主页TFT刷新分频系数�?=每个新视觉帧都刷新） */

/* ==================== 菜单可调参数变量 ==================== */
/* 每个变量都绑定到菜单的某个条目，用户可通过按键实时修改 */

int16 motor_speed =300;       /* 目标电机速度（PWM占空比单位），范�?~600，步�?0 */
int16 motor_enable = 0;       /* 电机使能开关：0=禁用（停止）�?=启用（允许运行） */
int16 motor_run_time =12;     /* 电机最大运行时间（秒） */
int16 run_quiet_enable =RUN_QUIET_STOP_KEY; /* 运行静默模式：运行时关闭TFT/UART图传/普通按�?*///RUN_QUIET_STOP_KEY   RUN_QUIET_DEFAULT_ENABLE
uint8 race_state = RACE_STATE_STOP;

int16 cam_exposure = 200;      /* 摄像头曝光时间（行周期数），值越大画面越�?*/

int16 pid_kp = 11;             /* 转向PD控制器的比例系数Kp */
int16 pid_ki = 2;              /* 速度PI控制器的积分系数Ki */
int16 pid_kd = 20;             /* 转向PD控制器的微分系数Kd */

int16 sp_err_t1 = 6;          /* 速度规划：直线判定误差阈值，|error|<=此值视为直�?*/
int16 sp_err_t2 = 16 ;         /* 速度规划：弯道判定误差阈值，|error|>=此值视为急弯 */
int16 sp_ratio_1 = 100;       /* 速度规划：直道目标速度百分比（100%=满速） */
int16 sp_ratio_2 = 74;        /* 速度规划：弯道目标速度百分比（35%=降速过弯） */
int16 steer_speed_k = 8;      /* 转向速度耦合系数：速度越快，转向补偿增�?*/
int16 steer_ff_k = 12;        /* 前瞻前馈系数：根据前瞻误差提前施加转向补�?*/

/* ==================== 直角弯RA状态机参数 ==================== */
int16 ra_hard_inner = 320;       /* 直角弯HARD阶段内侧电机占空比，0=内侧轮停�?*/
int16 ra_hard_outer = 1850;    /* 直角弯HARD阶段外侧电机占空比，外侧轮推动车身转�?*/
int16 ra_hard_rate = 500;      /* HARD target yaw-rate, deg/s */
int16 ra_hard_yaw = 90;        /* 直角弯HARD阶段退出航向角阈值（度），IMU累计转过此角度退�?*/
int16 ra_slow_row = 50;        /* 直角弯SLOW阶段触发行号：IP最大行>=此值时进入减�?*/
int16 ra_slow_pct = 85;        /* 直角弯SLOW阶段速度百分比（20%=更快刹住�?*/
int16 ra_turn_row = 112;        /* 直角弯APPROACH阶段触发行号：IP最大行>=此值时准备转弯 */
int16 ra_approach_frames =1;  /* 直角弯APPROACH阶段刹车稳车帧数，结束后进入HARD急转 */

int16 yaw_kp = 12;            /* IMU级联控制的航向角比例系数Kp */

/* ==================== 菜单条目定义�?==================== */
/* 每个MenuItem包含：显示标签、绑定变量指针、最小值、最大值、步进�?*/

/* 主页菜单条目：常用运行开关和速度档位 */
static MenuItem items_main[] = {
    {"Quiet", &run_quiet_enable, 0, 1, 1},    /* 运行静默�?=运行时关闭TFT/UART/普通按�?*/
    {"Speed", &motor_speed, 0, 600, 20},      /* 速度设定�?~600，步�?0 */
};

/* 综合调参页：电机、摄像头、PID和速度规划�?*/
static MenuItem items_tune[] = {
    {"Bias", &threshold_bias, -50, 50, 5},    /* Otsu阈值偏移补偿：-50~50，步�? */
    {"Expose", &cam_exposure, 100, 800, 10},  /* 曝光时间�?00~800，步�?0 */
    {"Kp", &pid_kp, 0, 100, 1},              /* 比例系数�?~100，步�? */
    {"Kd", &pid_kd, 0, 50, 1},               /* 微分系数�?~50，步�? */
};

/* 直角弯RA页菜单条目：直角弯全部阶段的可调参数 */
static MenuItem items_ra[] = {
    {"SlwPct", &ra_slow_pct, 10, 100, 5},    /* SLOW速度百分比：减速后的速度 */
    {"TrnRow", &ra_turn_row, 40, 115, 5},    /* APPROACH触发行号：IP行号>=此值准备转�?*/
    {"AproF", &ra_approach_frames, 1, 6, 1}, /* APPROACH刹车稳车帧数：进入HARD前主动收�?*/
    {"Inner", &ra_hard_inner, 0, 1500, 50},
    {"Outer", &ra_hard_outer, 500, 5000, 100},/* HARD外侧电机占空比：转弯时外侧轮驱动�?*/
    {"Rate", &ra_hard_rate, 220, 720, 20},
    {"Yaw", &ra_hard_yaw, 50, 96, 2},       /* HARD退出航向角：IMU累计转过的角度阈�?*/
};

/* IMU级联控制页菜单条目：航向角闭环控制参�?*/
static MenuItem items_imu[] = {
    {"CasEn",   &cascade_en,    0, 1, 1},    /* 级联控制使能�?=关闭�?=开�?*/
    {"YawKp",   &yaw_kp,        0, 30, 1},
    {"YawKd",   &yaw_kd,        0, 20, 1},
};

/* ==================== 页面定义�?==================== */
/* g_pages数组定义了菜单页面，每个页面包含标题、条目数组、条目数和自定义绘制函数 */
/* draw为NULL表示使用default_draw默认绘制函数 */
static MenuPageDef g_pages[PAGE_MAX] = {
    { .title = "MAIN",  .items = items_main,  .item_count = 2,  .draw = NULL },  /* �?页：主页，显示静默和速度 */
    { .title = "TUNE",  .items = items_tune,  .item_count = 4, .draw = NULL },  /* �?页：电机/摄像�?PID/速度规划 */
    { .title = "RA",    .items = items_ra,    .item_count = 7, .draw = NULL },  /* �?页：直角弯页，调整RA状态机各阶段参�?*/
    { .title = "IMU",   .items = items_imu,   .item_count = 3,  .draw = NULL },  /* �?页：IMU页，调整航向角级联控制参�?*/
};

MenuPage now_page = PAGE_MAIN;   /* 当前显示的页面编号，初始为主�?*/
uint8 menu_cursor = 0u;          /* 当前页面内的光标位置（指向第几个菜单条目），�?开�?*/
static uint8 s_page_cursor[PAGE_MAX] = {0u}; /* 每页记住自己的光标位�?*/
static uint8 s_repeat_key = 0u;              /* 当前长按连调按键 */
static uint8 s_repeat_start_cnt = 0u;        /* 长按开始等待计�?*/
static uint8 s_repeat_interval_cnt = 0u;     /* 长按连调间隔计数 */

static void cursor_clamp(void);

/* ==================== 拨码开关模式检�?==================== */
/**
 * dip_is_adjust_mode() - 检测拨码开关状态，判断当前是选择模式还是调节模式
 *
 * 读取P33_11和P33_12两个拨码开关的电平（低电平有效）�? * 两个开关均未拨下（均为高电平）-> 返回0 = 选择模式（KEY3/KEY4移动光标�? * 任一开关拨下（低电平）-> 返回1 = 调节模式（KEY3/KEY4增减当前参数值）
 *
 * 返回值：0=选择模式�?=调节模式
 */
static uint8 dip_is_adjust_mode(void)
{
    uint8 sw1 = (gpio_get_level(SWITCH1) == 0) ? 1u : 0u;  /* 读取拨码开�?状态，低电�?拨下=1 */
    uint8 sw2 = (gpio_get_level(SWITCH2) == 0) ? 1u : 0u;  /* 读取拨码开�?状态，低电�?拨下=1 */
    return sw1 | sw2;  /* 任一拨下即为调节模式（逻辑或） */
}

/* ==================== 参数修改后的实时生效处理 ==================== */
/**
 * menu_apply_adjusted_value() - 在调节模式下修改参数后，立即应用到硬�? *
 * 目前仅处理摄像头曝光时间的实时更新：当用户在CAM页面修改Expose参数后，
 * 立即调用mt9v03x_set_exposure_time()使新曝光值生效，无需重启�? * 其他参数（如PID增益、速度等）在控制循环中直接读取变量值，无需额外应用�? */
static void menu_apply_adjusted_value(void)
{
    cursor_clamp();

    /* 当前调节曝光条目时，立即设置曝光时间 */
    if (g_pages[now_page].items[menu_cursor].value == &cam_exposure)
        mt9v03x_set_exposure_time((uint16)cam_exposure);  /* 调用摄像头驱动设置新的曝光�?*/
}

static uint8 key_raw_read(void)
{
    if (gpio_get_level(KEY1) == 0)       /* 检测KEY1是否按下（低电平有效�?*/
        return 1u;                       /* KEY1按下，记录编�? */
    if (gpio_get_level(KEY2) == 0)       /* 检测KEY2是否按下 */
        return 2u;                       /* KEY2按下，记录编�? */
    if (gpio_get_level(KEY3) == 0)       /* 检测KEY3是否按下 */
        return 3u;                       /* KEY3按下，记录编�? */
    if (gpio_get_level(KEY4) == 0)       /* 检测KEY4是否按下 */
        return 4u;                       /* KEY4按下，记录编�? */

    return 0u;
}

static void key_repeat_reset(void)
{
    s_repeat_key = 0u;
    s_repeat_start_cnt = 0u;
    s_repeat_interval_cnt = 0u;
}

static uint8 key_repeat_adjust_event(uint8 key_event, uint8 adjust_mode)
{
    uint8 raw_key = key_raw_read();

    if (!adjust_mode || (raw_key != 3u && raw_key != 4u))
    {
        key_repeat_reset();
        return key_event;
    }

    if (key_event == 3u || key_event == 4u)
    {
        s_repeat_key = key_event;
        s_repeat_start_cnt = 0u;
        s_repeat_interval_cnt = 0u;
        return key_event;
    }

    if (key_event != 0u)
        return key_event;

    if (s_repeat_key != raw_key)
    {
        s_repeat_key = raw_key;
        s_repeat_start_cnt = 0u;
        s_repeat_interval_cnt = 0u;
        return 0u;
    }

    if (s_repeat_start_cnt < KEY_REPEAT_START_CNT)
    {
        s_repeat_start_cnt++;
        return 0u;
    }

    s_repeat_interval_cnt++;
    if (s_repeat_interval_cnt >= KEY_REPEAT_INTERVAL_CNT)
    {
        s_repeat_interval_cnt = 0u;
        return raw_key;
    }

    return 0u;
}

/* ==================== 四状态机按键扫描 ==================== */
/**
 * key_scan() - 按键扫描状态机，实现消抖和单次触发
 *
 * 使用四状态机实现完整的按键检测流程：
 *   IDLE -> DEBOUNCE -> HOLD -> RELEASE -> IDLE
 *
 * 在IDLE状态检测到任意按键按下后进入DEBOUNCE，连续检测DEBOUNCE_CNT�? * 确认为有效按下，此时返回按键编号�?~4）。之后进入HOLD等待释放�? * 再经过RELEASE消抖确认完全松开，回到IDLE。整个过程中只在DEBOUNCE->HOLD
 * 转换时返回一次按键值，避免重复触发�? *
 * 返回值：0=无按键事件，1=KEY1按下�?=KEY2按下�?=KEY3按下�?=KEY4按下
 */
static uint8 key_scan(void)
{
    /* 按键状态机的四个状态枚�?*/
    typedef enum
    {
        KEY_STATE_IDLE = 0,    /* 空闲状态：等待按键按下 */
        KEY_STATE_DEBOUNCE,    /* 消抖状态：持续检测同一按键是否保持按下 */
        KEY_STATE_HOLD,        /* 按住状态：已确认按下，等待松开 */
        KEY_STATE_RELEASE,     /* 释放消抖状态：确认按键已完全松开 */
    } KeyState;                /* 按键状态枚举类型定�?*/

    static KeyState state = KEY_STATE_IDLE;  /* 当前状态机状态，函数间保持（静态局部变量） */
    static uint16 cnt = 0u;                  /* 消抖/释放计数器，用于计数连续检测次�?*/
    static uint8 last_key = 0u;              /* 记录正在消抖的按键编号，用于对比一致�?*/

    uint8 cur_key = key_raw_read();  /* 当前扫描到的按键编号�?表示无按键按�?*/

    switch (state)  /* 根据当前状态机状态执行对应逻辑 */
    {
    case KEY_STATE_IDLE:
        /* 空闲状态：检测到任意按键按下，记录按键编号并进入消抖 */
        if (cur_key != 0u)  /* 有按键按�?*/
        {
            last_key = cur_key;    /* 记录当前按下的按键编�?*/
            cnt = 0u;              /* 重置消抖计数�?*/
            state = KEY_STATE_DEBOUNCE;  /* 切换到消抖状�?*/
        }
        break;  /* 无按键则保持IDLE状�?*/

    case KEY_STATE_DEBOUNCE:
        /* 消抖状态：持续检测同一按键是否保持按下 */
        if (cur_key == last_key)  /* 当前按键与记录的按键一致，说明按键持续按下 */
        {
            cnt++;  /* 消抖计数器递增 */
            if (cnt >= DEBOUNCE_CNT)  /* 连续DEBOUNCE_CNT次检测到同一按键，确认有效按�?*/
            {
                cnt = 0u;                    /* 重置计数�?*/
                state = KEY_STATE_HOLD;      /* 消抖通过，进入按住状�?*/
                return last_key;             /* 返回确认的按键编号（唯一一次返回非0值） */
            }
        }
        else
        {
            /* 消抖期间检测到不同按键或无按键，视为抖动，回到空闲 */
            state = KEY_STATE_IDLE;  /* 回到空闲状�?*/
            cnt = 0u;                /* 重置计数�?*/
        }
        break;

    case KEY_STATE_HOLD:
        /* 按住状态：等待按键松开（cur_key变为0�?*/
        if (cur_key == 0u)  /* 所有按键均已松开 */
        {
            cnt = 0u;                      /* 重置释放消抖计数�?*/
            state = KEY_STATE_RELEASE;     /* 按键松开，进入释放消抖状�?*/
        }
        break;

    case KEY_STATE_RELEASE:
        /* 释放消抖状态：确认按键已完全松开，防止抖动误触发 */
        if (cur_key == 0u)  /* 持续无按键，说明确实已松开 */
        {
            cnt++;  /* 释放消抖计数器递增 */
            if (cnt >= RELEASE_CNT)  /* 连续RELEASE_CNT次无按键 */
            {
                cnt = 0u;                /* 重置计数�?*/
                state = KEY_STATE_IDLE;  /* 确认松开，回到空闲状�?*/
            }
        }
        else
        {
            /* 释放消抖期间又检测到按键，回到按住状态（用户未真正松开�?*/
            state = KEY_STATE_HOLD;  /* 回到按住状�?*/
            cnt = 0u;                /* 重置计数�?*/
        }
        break;

    default:
        /* 未知状态，安全回到空闲 */
        state = KEY_STATE_IDLE;  /* 强制回到空闲状态，防止状态机卡死 */
        break;
    }

    return 0u;  /* 无确认的按键事件，返�? */
}

/* ==================== 光标位置钳位 ==================== */
/**
 * cursor_clamp() - 将菜单光标位置限制在当前页面的有效范围内
 *
 * 防止页面切换后光标越界。例如从条目多的RA页面�?0项）切换�? * 条目少的MAIN页面�?项）时，光标可能超出范围，需要钳位到有效值�? */
static void cursor_clamp(void)
{
    uint8 max_items = g_pages[now_page].item_count;  /* 获取当前页面的菜单条目总数 */

    if (max_items == 0u)               /* 页面无条目（防御性检查） */
        menu_cursor = 0u;              /* 光标归零 */
    else if (menu_cursor >= max_items) /* 光标超出范围 */
        menu_cursor = max_items - 1u;  /* 钳位到最后一个条�?*/
}

static void race_led_apply(void)
{
    if (battery_is_low())
    {
        gpio_set_level(RACE_LED_STOP, GPIO_HIGH);
        gpio_set_level(RACE_LED_READY, GPIO_HIGH);
        gpio_set_level(RACE_LED_RUN, GPIO_HIGH);
        return;
    }

    gpio_set_level(RACE_LED_STOP, (race_state == RACE_STATE_STOP || race_state == RACE_STATE_DONE) ? GPIO_HIGH : GPIO_LOW);
    gpio_set_level(RACE_LED_READY, (race_state == RACE_STATE_READY || race_state == RACE_STATE_ARMED) ? GPIO_HIGH : GPIO_LOW);
    gpio_set_level(RACE_LED_RUN, (race_state == RACE_STATE_RUN) ? GPIO_HIGH : GPIO_LOW);
}

static uint8 race_key_event(void)
{
    typedef enum
    {
        RACE_KEY_IDLE = 0,
        RACE_KEY_DEBOUNCE_DOWN,
        RACE_KEY_HOLD,
        RACE_KEY_DEBOUNCE_UP
    } RaceKeyState;

    static RaceKeyState state = RACE_KEY_IDLE;
    static uint8 cnt = 0u;
    uint8 pressed = (gpio_get_level(RACE_KEY) == 0) ? 1u : 0u;

    switch (state)
    {
    case RACE_KEY_IDLE:
        if (pressed)
        {
            cnt = 0u;
            state = RACE_KEY_DEBOUNCE_DOWN;
        }
        break;

    case RACE_KEY_DEBOUNCE_DOWN:
        if (pressed)
        {
            if (++cnt >= DEBOUNCE_CNT)
            {
                cnt = 0u;
                state = RACE_KEY_HOLD;
            }
        }
        else
        {
            cnt = 0u;
            state = RACE_KEY_IDLE;
        }
        break;

    case RACE_KEY_HOLD:
        if (!pressed)
        {
            cnt = 0u;
            state = RACE_KEY_DEBOUNCE_UP;
        }
        break;

    case RACE_KEY_DEBOUNCE_UP:
        if (!pressed)
        {
            if (++cnt >= RELEASE_CNT)
            {
                cnt = 0u;
                state = RACE_KEY_IDLE;
                return 1u;
            }
        }
        else
        {
            cnt = 0u;
            state = RACE_KEY_HOLD;
        }
        break;

    default:
        cnt = 0u;
        state = RACE_KEY_IDLE;
        break;
    }

    return 0u;
}

static void race_control_prepare(void)
{
    motor_enable = 0;
    line_pid_init();
    imu_reset_yaw();
    tft180_clear();
    race_state = RACE_STATE_ARMED;
    race_led_apply();
}



void race_control_process(void)
{
    uint8 event = race_key_event();

    if (motor_enable != 0 && race_state != RACE_STATE_RUN)
    {
        race_state = RACE_STATE_RUN;
        race_led_apply();
    }
    else if (motor_enable == 0 && race_state == RACE_STATE_RUN)
    {
        race_state = RACE_STATE_DONE;
        race_led_apply();
    }



    if (!event)
        return;

    switch (race_state)
    {
    case RACE_STATE_STOP:
        motor_enable = 0;
        race_state = RACE_STATE_READY;
        race_led_apply();
        break;

    case RACE_STATE_READY:
        race_control_prepare();
        break;

    case RACE_STATE_ARMED:
        motor_enable = 1;
        race_state = RACE_STATE_RUN;
        race_led_apply();
        break;

    case RACE_STATE_RUN:
        motor_enable = 0;
        race_state = RACE_STATE_DONE;
        race_led_apply();
        break;

    case RACE_STATE_DONE:
    default:
        motor_enable = 0;
        race_state = RACE_STATE_READY;
        race_led_apply();
        break;
    }
}

uint8 run_quiet_active(void)
{
    return (motor_enable != 0 && run_quiet_enable != 0u) ? 1u : 0u;
}

uint8 ui_raw_input_state(void)
{
    uint8 state = 0u;

    if (gpio_get_level(KEY1) == 0) state |= 0x01u;
    if (gpio_get_level(KEY2) == 0) state |= 0x02u;
    if (gpio_get_level(KEY3) == 0) state |= 0x04u;
    if (gpio_get_level(KEY4) == 0) state |= 0x08u;
    if (gpio_get_level(SWITCH1) == 0) state |= 0x10u;
    if (gpio_get_level(SWITCH2) == 0) state |= 0x20u;
    if (gpio_get_level(RACE_KEY) == 0) state |= 0x40u;

    return state;
}

/* ==================== GPIO初始�?==================== */
/**
 * key_init_all() - 初始化所有按键、拨码开关和LED的GPIO引脚
 *
 * 4个按键和2个拨码开关均配置为输入模式（GPI），高电平默认，内部上拉�? * 未按下时读取高电平，按下时接地读取低电平�? * LED配置为输出模式（GPO），默认高电平（LED熄灭）�? */
void key_init_all(void)
{
    gpio_init(KEY1, GPI, GPIO_HIGH, GPI_PULL_UP);      /* KEY1：输入模式，上拉，未按下为高电平 */
    gpio_init(KEY2, GPI, GPIO_HIGH, GPI_PULL_UP);      /* KEY2：输入模式，上拉 */
    gpio_init(KEY3, GPI, GPIO_HIGH, GPI_PULL_UP);      /* KEY3：输入模式，上拉 */
    gpio_init(KEY4, GPI, GPIO_HIGH, GPI_PULL_UP);      /* KEY4：输入模式，上拉 */
    gpio_init(RACE_KEY, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(SWITCH1, GPI, GPIO_HIGH, GPI_PULL_UP);   /* 拨码开�?：输入模式，上拉 */
    gpio_init(SWITCH2, GPI, GPIO_HIGH, GPI_PULL_UP);   /* 拨码开�?：输入模式，上拉 */
    gpio_init(TURN_RIGHT_LED, GPO, GPIO_HIGH, GPI_PULL_UP); /* 右转LED：输出模式，默认高电平（LED熄灭�?*/
    gpio_init(RACE_LED_STOP, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(RACE_LED_READY, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(RACE_LED_RUN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    race_led_apply();
}

/* ==================== 按键事件处理主函�?==================== */
/**
 * key_process() - 处理按键事件，实现菜单导航和参数调节
 *
 * 每次被主循环调用时扫描一次按键，根据当前模式（选择/调节）执行不同操作：
 *
 * 通用功能（不受模式影响）�? *   KEY1 -> 切换到下一页（循环�? *   KEY2 -> 切换到上一页（循环�? *
 * 选择模式（拨码开关未拨下）：
 *   KEY3 -> 光标上移（到头则循环到底部）
 *   KEY4 -> 光标下移（到底则循环到顶部）
 *
 * 调节模式（拨码开关任一拨下）：
 *   KEY3 -> 当前参数值增加一个步进（不超过最大值）
 *   KEY4 -> 当前参数值减少一个步进（不低于最小值）
 *   修改后自动调用menu_apply_adjusted_value()使参数生�? *
 * 非RACE_MODE时，翻页会清屏以避免残留显示�? */
void key_process(void)
{
    uint8 key = key_scan();  /* 调用按键扫描状态机，获取确认的按键事件编号 */
    uint8 adjust_mode = dip_is_adjust_mode();   /* 读取拨码开关，判断当前是选择模式还是调节模式 */

    key = key_repeat_adjust_event(key, adjust_mode); /* 调节模式下支持KEY3/KEY4长按连调 */

    if (run_quiet_active())
        return;

    if (key == 0u)           /* 无按键事�?*/
        return;              /* 直接返回，不执行任何操作 */

    /* KEY1：切换到下一页（循环翻页�?*/
    if (key == 1u)
    {
        s_page_cursor[now_page] = menu_cursor;                /* 保存当前页面光标 */
        now_page = (MenuPage)((now_page + 1u) % PAGE_MAX);  /* 页码+1，到末尾循环回第0�?*/
        menu_cursor = s_page_cursor[now_page];                /* 恢复目标页面上次光标 */
        cursor_clamp();                                      /* 防御页面条目数变�?*/
        s_page_cursor[now_page] = menu_cursor;                /* 保存钳位后的光标 */
#if !RACE_MODE
        tft180_clear();  /* 非比赛模式下清屏，避免上一页的残留内容叠加显示 */
#endif
        return;  /* 翻页后直接返回，本帧不再处理其他按键 */
    }

    /* KEY2：切换到上一页（循环翻页�?*/
    if (key == 2u)
    {
        s_page_cursor[now_page] = menu_cursor;                /* 保存当前页面光标 */
        now_page = (now_page == 0u) ? (MenuPage)(PAGE_MAX - 1u) : (MenuPage)(now_page - 1u);  /* 页码-1，第0页循环到末页 */
        menu_cursor = s_page_cursor[now_page];                /* 恢复目标页面上次光标 */
        cursor_clamp();                                      /* 防御页面条目数变�?*/
        s_page_cursor[now_page] = menu_cursor;                /* 保存钳位后的光标 */
#if !RACE_MODE
        tft180_clear();  /* 非比赛模式下清屏 */
#endif
        return;  /* 翻页后直接返�?*/
    }

    /* 以下处理KEY3/KEY4，根据拨码开关状态决定是移动光标还是调节参数 */
    MenuPageDef *page = &g_pages[now_page];     /* 获取当前页面的定义结构体指针 */
    uint8 item_cnt = page->item_count;          /* 当前页面的菜单条目总数 */

    if (item_cnt == 0u)     /* 页面无条目（防御性检查） */
        return;             /* 无需处理，直接返�?*/

    if (!adjust_mode)       /* 拨码开关未拨下 = 选择模式 */
    {
        /* ===== 选择模式：KEY3/KEY4移动光标 ===== */
        if (key == 3u)
            menu_cursor = (menu_cursor == 0u) ? (item_cnt - 1u) : (menu_cursor - 1u);  /* KEY3光标上移，到顶循环到底部 */
        if (key == 4u)
            menu_cursor = (menu_cursor + 1u) % item_cnt;  /* KEY4光标下移，到底循环到顶部 */

        s_page_cursor[now_page] = menu_cursor;  /* 记住当前页面光标 */
    }
    else                    /* 拨码开关拨�?= 调节模式 */
    {
        /* ===== 调节模式：KEY3/KEY4增减当前参数�?===== */
        cursor_clamp();                              /* 先确保光标在有效范围内，防止越界访问 */
        MenuItem *cur = &page->items[menu_cursor];  /* 获取光标指向的菜单条目结构体指针 */

        if (key == 3u)      /* KEY3：增加参数�?*/
        {
            /* KEY3：增加值，不超过最大�?*/
            if (*cur->value + cur->step <= cur->max)  /* 步进增加后不超过最大�?*/
                *cur->value += cur->step;             /* 执行步进增加 */
            else
                *cur->value = cur->max;               /* 已达最大值，钳位到最大�?*/
        }

        if (key == 4u)      /* KEY4：减少参数�?*/
        {
            /* KEY4：减少值，不低于最小�?*/
            if (*cur->value - cur->step >= cur->min)  /* 步进减少后不低于最小�?*/
                *cur->value -= cur->step;             /* 执行步进减少 */
            else
                *cur->value = cur->min;               /* 已达最小值，钳位到最小�?*/
        }

        menu_apply_adjusted_value();  /* 参数修改后立即应用到硬件（如曝光时间�?*/
    }
}

uint8 quiet_stop_key_pressed(void)
{
    uint8 stop_pressed = 0u;

    switch (RUN_QUIET_STOP_KEY)
    {
    case 1u:
        stop_pressed = (gpio_get_level(KEY1) == 0) ? 1u : 0u;
        break;
    case 2u:
        stop_pressed = (gpio_get_level(KEY2) == 0) ? 1u : 0u;
        break;
    case 3u:
        stop_pressed = (gpio_get_level(KEY3) == 0) ? 1u : 0u;
        break;
    case 4u:
        stop_pressed = (gpio_get_level(KEY4) == 0) ? 1u : 0u;
        break;
    default:
        break;
    }

    return stop_pressed;
}

void key_process_quiet(void)
{
    if (quiet_stop_key_pressed())
        motor_enable = 0;
}

void ui_process_keys(void)
{
    race_control_process();

    if (motor_enable != 0)
        return;

    if (run_quiet_active())
    {
        key_process_quiet();
    }
    else
    {
        key_process();
    }
}

/* ==================== 字符串拼接辅助函�?==================== */
/**
 * append_str() - 将源字符串追加到目标缓冲区的指定位置
 *
 * @dst  目标缓冲�? * @pos  当前写入位置（追加起始偏移）
 * @src  要追加的源字符串（以'\0'结尾�? *
 * 返回值：追加完成后的新写入位�? */
static uint8 append_str(char *dst, uint8 pos, const char *src)
{
    while (*src)                         /* 遍历源字符串直到遇到结束�?\0' */
        dst[pos++] = *src++;             /* 逐字符复制到目标缓冲区，同时推进源指�?*/

    return pos;                          /* 返回新的写入位置（追加后的偏移量�?*/
}

/* ==================== 整数转字符串辅助函数 ==================== */
/**
 * append_int() - 将int16整数转换为十进制字符串并追加到缓冲区
 *
 * @dst  目标缓冲�? * @pos  当前写入位置
 * @val  要转换的int16整数�? *
 * 处理负号，使用反序生成法：先逐位取余存入临时数组，再反向写入目标缓冲区�? * 支持-32768~32767范围内的所有int16值�? *
 * 返回值：转换完成后的新写入位�? */
static uint8 append_int(char *dst, uint8 pos, int16 val)
{
    int32 v = val;    /* 提升为int32避免取负溢出�?32768取反仍为int32范围�?*/
    char tmp[8];      /* 临时存储各位数字字符（反序存放），最�?位数�?负号 */
    uint8 n = 0u;     /* 数字位数计数�?*/

    if (v < 0)        /* 处理负数 */
    {
        dst[pos++] = '-';  /* 负数先写入负�?*/
        v = -v;            /* 取绝对值，后续按正数处�?*/
    }

    /* 逐位取余，从个位开始存入tmp数组（反序） */
    do
    {
        tmp[n++] = (char)('0' + (v % 10));  /* 取最低位数字，转为ASCII字符存入临时数组 */
        v /= 10;                             /* 去掉已处理的最低位，准备处理下一�?*/
    } while (v > 0);                         /* 直到所有位都处理完�?*/

    /* 将tmp中的数字字符反向写入目标缓冲区（恢复正序�?*/
    for (uint8 i = n; i > 0u; i--)           /* 从最高位到最低位遍历 */
        dst[pos++] = tmp[i - 1u];            /* 写入数字字符到目标缓冲区 */

    return pos;  /* 返回新的写入位置 */
}

/* ==================== 默认页面绘制函数 ==================== */
/**
 * default_draw() - 绘制当前菜单页面内容到TFT180显示�? *
 * @page  指向当前页面定义结构体的指针
 *
 * 根据当前页面类型执行不同的绘制逻辑�? *
 * MAIN页面（特殊处理）�? *   - 显示电机使能状态（EN:0/1�? *   - 显示路线调试信息：当前步�?总步骤（L:x/x�? *   - 显示预判标志和动作（PF:x, PA:x�? *   - 使用简短标�?EN"以节省显示空�? *
 * 其他页面（通用处理）：
 *   - 顶部显示页面标题
 *   - 标题下方显示当前模式状态（[SW:SELECT]或[SW:ADJUST]�? *   - 列表形式显示所有菜单条目，光标所在行前面显示">"标记
 *   - 每行格式�?>标签�?   数�?
 *   - 底部显示操作提示"K1/K2:Page K3/K4:+-"
 *
 * 在非RACE_MODE下才实际执行绘制�? */
static void default_draw(MenuPageDef *page)
{
    char buf[32];                              /* 用于拼接显示字符串的缓冲区，最�?1字符+结束�?*/
    uint8 adjust_mode = dip_is_adjust_mode();  /* 获取当前拨码开关模式，用于显示模式提示 */

    if (now_page == PAGE_MAIN)  /* 判断当前是否为主�?*/
    {
        /* 主页�?draw_line() 统一绘制，避免菜单文字与 ERR/ROW/THR 等调试值重�?*/
        return;  /* 主页绘制完毕，直接返�?*/
    }

    /* ===== 非主页通用绘制逻辑 ===== */

    /* 顶部显示页面标题（如"MOTOR"�?CAM"�?PID"等） */
    tft180_show_string(0, 0, (char *)page->title);  /* 在左上角显示当前页面标题 */

    /* 标题下方显示当前操作模式提示 */
    tft180_show_string(0, 8, adjust_mode ? "[SW:ADJUST]" : "[SW:SELECT]");  /* 根据拨码开关状态显示模式提�?*/

    /* 计算菜单列表的起始Y坐标，使列表整体垂直居中 */
    uint8 start_y = 120u - page->item_count * 8u;  /* 从底部向上排列，每项�?像素高度 */

    /* 遍历绘制当前页面的所有菜单条�?*/
    for (uint8 i = 0u; i < page->item_count; i++)  /* 循环遍历每个菜单条目 */
    {
        MenuItem *item = &page->items[i];  /* 获取第i个菜单条目指�?*/
        uint8 pos = 0u;                    /* 字符串缓冲区当前写入位置 */

        /* 拼接光标指示符：当前选中项显�?>"，否则显示空�?*/
        buf[pos++] = (i == menu_cursor) ? '>' : ' ';  /* 写入光标指示字符 */

        /* 拼接条目标签名（�?Kp"�?Speed"等） */
        pos = append_str(buf, pos, item->label);  /* 将标签字符串追加到缓冲区 */

        /* 用空格填充到固定宽度11字符，保证数值列对齐 */
        while (pos < 11u)       /* 如果当前写入位置未达�?1字符 */
            buf[pos++] = ' ';   /* 用空格填�?*/

        /* 拼接当前参数值的十进制字符串 */
        pos = append_int(buf, pos, *item->value);  /* 将参数值转为字符串追加到缓冲区 */
        buf[pos] = '\0';                           /* 写入字符串终止符 */

        /* 在TFT上显示该行菜单项，x=0兼容128像素宽屏幕，避免参数行越界触发assert */
        tft180_show_string(0, start_y + i * 8u, buf);  /* 显示拼接好的菜单行字符串 */
    }

    /* 底部显示操作提示文字 */
    /* 底部提示行暂不显示，避免部分TFT驱动在y=120边界处触发assert */
}

/* ==================== 菜单显示主函�?==================== */
/**
 * menu_show() - 菜单页面显示主函数，由主循环周期性调�? *
 * 根据RACE_MODE和运行静默模式决定是否执行绘制：
 *   - RACE_MODE=1时直接返回，比赛模式下不更新TFT以节省时�? *   - run_quiet_active()=1时，运行期间完全关闭TFT显示
 *
 * 对于MAIN页面，使用draw_line()在TFT上显示二值化图像和叠加信息，
 * 并通过分频计数器控制刷新频率（运行和停止状态可设不同频率）�? * 对于其他页面，调用default_draw()绘制参数列表�? *
 * 分频机制：s_main_draw_cnt�?递增到draw_div-1时才执行一次绘制，
 * 中间的调用直接跳过，从而降低TFT刷新率，减少对实时控制的干扰�? */
void menu_show(void)
{
#if RACE_MODE
    return;  /* 比赛模式下跳过所有TFT更新，节省CPU时间给控制任�?*/
#else
    MenuPageDef *page = &g_pages[now_page];  /* 获取当前页面定义结构体指�?*/
    static uint8 s_main_draw_cnt = 0u;       /* 主页绘制分频计数器，每次调用递增 */
    static uint8 s_tft_run_off = 0u;         /* 运行时TFT关闭标志，用于状态切换时清屏一�?*/
    static uint32 s_last_main_frame = 0xFFFFFFFFu; /* 上次主页TFT已显示的视觉帧号 */
    uint8 need_default_draw = 1u;             /* 是否需要绘制菜单文�?*/

    /* Running display-off mode: motor enabled means the car is running. */
    if (run_quiet_active())
    {
        if (s_tft_run_off == 0u)  /* 刚进入运行状态（之前TFT是开着的） */
        {
            tft180_clear();          /* 刚进入运行状态时清屏一次，消除残留画面 */
            s_main_draw_cnt = 0u;    /* 重置分频计数�?*/
            s_tft_run_off = 1u;      /* 标记TFT已关闭，防止重复清屏 */
        }
        return;  /* 运行期间不再绘制，直接返�?*/
    }

    /* 电机停止后恢复TFT显示 */
    if (s_tft_run_off != 0u)  /* 之前TFT处于关闭状态，现在电机停止�?*/
    {
        tft180_clear();          /* 刚停止时清屏一次，准备恢复显示 */
        s_main_draw_cnt = 0u;    /* 重置分频计数�?*/
        s_tft_run_off = 0u;      /* 标记TFT已恢复显�?*/
    }

    if (now_page == PAGE_MAIN)  /* 当前为主�?*/
    {
        /* 主页：显示摄像头图像叠加信息，使用分频控制刷新率 */
        uint8 draw_div = (motor_enable != 0) ? MAIN_DRAW_DIV_RUN : MAIN_DRAW_DIV_STOP;  /* 根据运行状态选择分频系数 */

        need_default_draw = 0u;           /* 主页文字跟随图像新帧刷新，避免主循环空转时反复写�?*/

        if (g_tf_frame_count != s_last_main_frame)  /* 只有视觉处理完成新帧后才刷新TFT */
        {
            s_last_main_frame = g_tf_frame_count;   /* 记录当前已消费的视觉帧号 */

            if (s_main_draw_cnt == 0u)              /* 分频计数器归零时执行一次绘�?*/
            {
                draw_line();                        /* 调用draw_line()显示二值化图像+边界�?叠加信息 */
                need_default_draw = 1u;             /* 图像刷新后同步刷新主页菜单文�?*/
            }

            s_main_draw_cnt++;                      /* 分频计数器递增 */
            if (s_main_draw_cnt >= draw_div)        /* 计数器达到分频�?*/
                s_main_draw_cnt = 0u;               /* 归零，触发下次绘�?*/
        }
    }
    else                            /* 当前非主�?*/
    {
        s_main_draw_cnt = 0u;      /* 非主页时重置计数器，确保切回主页时立即绘�?*/
        s_last_main_frame = 0xFFFFFFFFu; /* 切回主页时强制立刻按新帧刷新 */
    }

    if (need_default_draw != 0u)
        default_draw(page);  /* 绘制当前页面的菜单内容（主页绘制菜单项，其他页绘制参数列表） */
#endif
}

/* ==================== 右转LED控制函数 ==================== */
/**
 * turn_right_led_on() - 点亮右转指示LED
 *
 * LED为低电平点亮（共阳极接法），设置GPIO输出低电平即可点亮�? * 用于在检测到右转分支时点亮LED作为视觉指示�? */
void turn_right_led_on(void)
{
    gpio_set_level(TURN_RIGHT_LED, GPIO_LOW);  /* 输出低电平，LED点亮 */
}

/**
 * turn_right_led_off() - 熄灭右转指示LED
 *
 * 设置GPIO输出高电平，LED熄灭�? * 在右转分支处理完毕后调用�? */
void turn_right_led_off(void)
{
    gpio_set_level(TURN_RIGHT_LED, GPIO_HIGH);  /* 输出高电平，LED熄灭 */
}
