#include "Menu.h"              /* 包含菜单模块头文件，提供MenuItem、MenuPage等结构体定义 */
#include "Track_funsion.h"     /* 包含视觉融合模块头文件，提供阈值偏移、拐点检测等外部变量 */
#include "Pid.h"               /* 包含PID控制模块头文件，提供电机运行时间等外部变量 */
#include "TFT_show_image.h"    /* 包含TFT显示模块头文件，提供draw_line()等显示函数 */

/* ==================== 按键与拨码开关GPIO引脚定义 ==================== */
#define KEY1 P11_3           /* 按键1：上一页 / 翻页（低电平有效，内部上拉） */
#define KEY2 P11_2           /* 按键2：下一页 / 翻页（低电平有效，内部上拉） */
#define KEY3 P20_6           /* 按键3：选择模式下=光标上移，调节模式下=增加值 */
#define KEY4 P20_7           /* 按键4：选择模式下=光标下移，调节模式下=减少值 */

#define TURN_RIGHT_LED P20_8 /* 右转指示LED引脚，低电平点亮 */
#define SWITCH1 P33_11       /* 拨码开关1：与SWITCH2配合决定选择/调节模式 */
#define SWITCH2 P33_12       /* 拨码开关2：与SWITCH1配合决定选择/调节模式 */
                             /* 两个开关均未拨下=选择模式，任一拨下=调节模式 */

/* ==================== 按键消抖与显示刷新参数 ==================== */
#define DEBOUNCE_CNT 20      /* 按键消抖计数阈值，达到此值才确认按键按下（每次扫描+1） */
#define RELEASE_CNT 10       /* 按键释放消抖计数阈值，确认按键已松开 */
#define MAIN_DRAW_DIV_STOP 1u  /* 停止状态下主页TFT刷新分频系数（1=每个新视觉帧都刷新） */
#define MAIN_DRAW_DIV_RUN  1u  /* 运行状态下主页TFT刷新分频系数（1=每个新视觉帧都刷新） */
#define TFT_OFF_WHEN_RUNNING 1u /* 电机和负压都开启后关闭TFT显示以节省CPU资源，0=不关闭 */

/* ==================== 菜单可调参数变量 ==================== */
/* 每个变量都绑定到菜单的某个条目，用户可通过按键实时修改 */

int16 motor_speed = 200;       /* 目标电机速度（PWM占空比单位），范围0~600，步进20 */
int16 motor_dir = 1;           /* 电机方向：0=正转，1=反转 */
int16 motor_enable = 0;       /* 电机使能开关：0=禁用（停止），1=启用（允许运行） */
int16 motor_run_time =12;     /* 电机最大运行时间（秒） */

int16 cam_exposure = 600;      /* 摄像头曝光时间（行周期数），值越大画面越亮 */

int16 pid_kp = 7;              /* 转向PD控制器的比例系数Kp */
int16 pid_ki = 2;              /* 速度PI控制器的积分系数Ki */
int16 pid_kd = 20;             /* 转向PD控制器的微分系数Kd */

int16 sp_err_t1 = 4;          /* 速度规划：直线判定误差阈值，|error|<=此值视为直线 */
int16 sp_err_t2 = 20;         /* 速度规划：弯道判定误差阈值，|error|>=此值视为急弯 */
int16 sp_ratio_1 = 100;       /* 速度规划：直道目标速度百分比（100%=满速） */
int16 sp_ratio_2 = 48;        /* 速度规划：弯道目标速度百分比（48%=降速过弯） */
int16 steer_speed_k = 3;      /* 转向速度耦合系数：速度越快，转向增益自动降低 */
int16 steer_ff_k = 12;        /* 前瞻前馈系数：根据前瞻误差提前施加转向补偿 */

/* ==================== 直角弯RA状态机参数 ==================== */
int16 ra_hard_inner = 0;       /* 直角弯HARD阶段内侧电机占空比，0=内侧轮停住 */
int16 ra_hard_outer = 1500;    /* 直角弯HARD阶段外侧电机占空比，外侧轮推动车身转向 */
int16 ra_hard_yaw = 52;        /* 直角弯HARD阶段退出航向角阈值（度），IMU累计转过此角度退出 */
int16 ra_slow_row = 38;        /* 直角弯SLOW阶段触发行号：IP最大行>=此值时进入减速 */
int16 ra_slow_pct = 45;        /* 直角弯SLOW阶段速度百分比（45%=大幅降速） */
int16 ra_turn_row = 96;        /* 直角弯APPROACH阶段触发行号：IP最大行>=此值时准备转弯 */
int16 ra_approach_frames = 1;  /* 直角弯APPROACH阶段等待帧数，等待完毕后进入HARD急转 */

int16 yaw_kp = 8;             /* IMU级联控制的航向角比例系数Kp */

/* ==================== 菜单条目定义表 ==================== */
/* 每个MenuItem包含：显示标签、绑定变量指针、最小值、最大值、步进值 */

/* 主页菜单条目：电机使能开关 */
static MenuItem items_main[] = {
    {"Enable", &motor_enable, 0, 1, 1},       /* 使能开关：0=关，1=开，步进1 */
};

/* 电机页菜单条目：电机速度与方向 */
static MenuItem items_motor[] = {
    {"Speed", &motor_speed, 0, 600, 20},      /* 速度设定：0~600，步进20 */
    {"Dir", &motor_dir, 0, 1, 1},             /* 方向设定：0或1，步进1 */
};

/* 摄像头页菜单条目：二值化偏移量与曝光时间 */
static MenuItem items_cam[] = {
    {"Bias", &threshold_bias, -50, 50, 5},    /* Otsu阈值偏移补偿：-50~50，步进5 */
    {"Expose", &cam_exposure, 100, 800, 10},  /* 曝光时间：100~800，步进10 */
};

/* PID页菜单条目：转向PD控制器三个参数 */
static MenuItem items_pid[] = {
    {"Kp", &pid_kp, 0, 100, 1},              /* 比例系数：0~100，步进1 */
    {"Ki", &pid_ki, 0, 50, 1},               /* 积分系数：0~50，步进1 */
    {"Kd", &pid_kd, 0, 50, 1},               /* 微分系数：0~50，步进1 */
};

/* 速度规划页菜单条目：速度分段控制参数 */
static MenuItem items_speed[] = {
    {"StrErr", &sp_err_t1, 1, 50, 1},        /* 直线误差阈值：用于判断是否在直线段 */
    {"CrvErr", &sp_err_t2, 10, 80, 1},       /* 弯道误差阈值：用于判断是否在急弯 */
    {"StrSpd%", &sp_ratio_1, 20, 100, 5},    /* 直线速度百分比：直道时的目标速度 */
    {"CrvSpd%", &sp_ratio_2, 20, 100, 5},    /* 弯道速度百分比：弯道时的目标速度 */
    {"SpdCpl", &steer_speed_k, 0, 50, 1},    /* 速度耦合系数：速度越快转向增益越小 */
    {"LaFF", &steer_ff_k, 0, 50, 1},         /* 前瞻前馈系数：根据前瞻误差的前馈补偿 */
};

/* 直角弯RA页菜单条目：直角弯全部阶段的可调参数 */
static MenuItem items_ra[] = {
    {"SlwRow", &ra_slow_row, 30, 110, 5},    /* SLOW触发行号：IP行号>=此值开始减速 */
    {"SlwPct", &ra_slow_pct, 10, 100, 5},    /* SLOW速度百分比：减速后的速度 */
    {"TrnRow", &ra_turn_row, 40, 115, 5},    /* APPROACH触发行号：IP行号>=此值准备转弯 */
    {"AproF", &ra_approach_frames, 1, 30, 1},/* APPROACH等待帧数：进入HARD前的等待时间 */
    {"Outer", &ra_hard_outer, 500, 5000, 100},/* HARD外侧电机占空比：转弯时外侧轮驱动力 */
    {"Inner", &ra_hard_inner, -2000, 5000, 100},/* HARD内侧电机占空比：转弯时内侧轮制动力（负值） */
    {"Yaw", &ra_hard_yaw, 30, 85, 5},        /* HARD退出航向角：IMU累计转过的角度阈值 */
    {"IpCol", &ip_col_offset, 0, 7, 1},      /* 拐点检测列偏移：调整拐点检测区域的起始列 */
    {"IpL", &ip_left_col, 0, 47, 1},         /* 左侧拐点搜索列：左拐点检测的参考列号 */
    {"IpR", &ip_right_col, 47, 93, 1},       /* 右侧拐点搜索列：右拐点检测的参考列号 */
};

/* IMU级联控制页菜单条目：航向角闭环控制参数 */
static MenuItem items_imu[] = {
    {"CasEn",   &cascade_en,    0, 1, 1},    /* 级联控制使能：0=关闭，1=开启 */
    {"YAW Kp",  &yaw_kp,        0, 100, 1},  /* 航向角比例系数Kp */
    {"YawKd",   &yaw_kd,        0, 30, 1},   /* 航向角微分系数Kd */
    {"Damp",    &yaw_damp_gain, 0, 30, 1},   /* 航向角阻尼增益：抑制航向角振荡 */
};

/* ==================== 页面定义表 ==================== */
/* g_pages数组定义了7个菜单页面，每个页面包含标题、条目数组、条目数和自定义绘制函数 */
/* draw为NULL表示使用default_draw默认绘制函数 */
static MenuPageDef g_pages[PAGE_MAX] = {
    { .title = "MAIN",  .items = items_main,  .item_count = 1,  .draw = NULL },  /* 第0页：主页，显示使能开关和路线调试信息 */
    { .title = "MOTOR", .items = items_motor, .item_count = 2,  .draw = NULL },  /* 第1页：电机页，设置速度和方向 */
    { .title = "CAM",   .items = items_cam,   .item_count = 2,  .draw = NULL },  /* 第2页：摄像头页，调整二值化偏移和曝光 */
    { .title = "PID",   .items = items_pid,   .item_count = 3,  .draw = NULL },  /* 第3页：PID页，调整转向PD三个参数 */
    { .title = "SPEED", .items = items_speed, .item_count = 6,  .draw = NULL },  /* 第4页：速度规划页，调整分段速度和耦合参数 */
    { .title = "RA",    .items = items_ra,    .item_count = 10, .draw = NULL },  /* 第5页：直角弯页，调整RA状态机各阶段参数 */
    { .title = "IMU",   .items = items_imu,   .item_count = 4,  .draw = NULL },  /* 第6页：IMU页，调整航向角级联控制参数 */
};

MenuPage now_page = PAGE_MAIN;   /* 当前显示的页面编号，初始为主页 */
uint8 menu_cursor = 0u;          /* 当前页面内的光标位置（指向第几个菜单条目），从0开始 */

/* ==================== 拨码开关模式检测 ==================== */
/**
 * dip_is_adjust_mode() - 检测拨码开关状态，判断当前是选择模式还是调节模式
 *
 * 读取P33_11和P33_12两个拨码开关的电平（低电平有效）。
 * 两个开关均未拨下（均为高电平）-> 返回0 = 选择模式（KEY3/KEY4移动光标）
 * 任一开关拨下（低电平）-> 返回1 = 调节模式（KEY3/KEY4增减当前参数值）
 *
 * 返回值：0=选择模式，1=调节模式
 */
static uint8 dip_is_adjust_mode(void)
{
    uint8 sw1 = (gpio_get_level(SWITCH1) == 0) ? 1u : 0u;  /* 读取拨码开关1状态，低电平=拨下=1 */
    uint8 sw2 = (gpio_get_level(SWITCH2) == 0) ? 1u : 0u;  /* 读取拨码开关2状态，低电平=拨下=1 */
    return sw1 | sw2;  /* 任一拨下即为调节模式（逻辑或） */
}

/* ==================== 参数修改后的实时生效处理 ==================== */
/**
 * menu_apply_adjusted_value() - 在调节模式下修改参数后，立即应用到硬件
 *
 * 目前仅处理摄像头曝光时间的实时更新：当用户在CAM页面修改Expose参数后，
 * 立即调用mt9v03x_set_exposure_time()使新曝光值生效，无需重启。
 * 其他参数（如PID增益、速度等）在控制循环中直接读取变量值，无需额外应用。
 */
static void menu_apply_adjusted_value(void)
{
    /* 当前在摄像头页面且光标在Expose条目（索引1）上时，立即设置曝光时间 */
    if (now_page == PAGE_CAM && menu_cursor == 1u)
        mt9v03x_set_exposure_time((uint16)cam_exposure);  /* 调用摄像头驱动设置新的曝光值 */
}

/* ==================== 四状态机按键扫描 ==================== */
/**
 * key_scan() - 按键扫描状态机，实现消抖和单次触发
 *
 * 使用四状态机实现完整的按键检测流程：
 *   IDLE -> DEBOUNCE -> HOLD -> RELEASE -> IDLE
 *
 * 在IDLE状态检测到任意按键按下后进入DEBOUNCE，连续检测DEBOUNCE_CNT次
 * 确认为有效按下，此时返回按键编号（1~4）。之后进入HOLD等待释放，
 * 再经过RELEASE消抖确认完全松开，回到IDLE。整个过程中只在DEBOUNCE->HOLD
 * 转换时返回一次按键值，避免重复触发。
 *
 * 返回值：0=无按键事件，1=KEY1按下，2=KEY2按下，3=KEY3按下，4=KEY4按下
 */
static uint8 key_scan(void)
{
    /* 按键状态机的四个状态枚举 */
    typedef enum
    {
        KEY_STATE_IDLE = 0,    /* 空闲状态：等待按键按下 */
        KEY_STATE_DEBOUNCE,    /* 消抖状态：持续检测同一按键是否保持按下 */
        KEY_STATE_HOLD,        /* 按住状态：已确认按下，等待松开 */
        KEY_STATE_RELEASE,     /* 释放消抖状态：确认按键已完全松开 */
    } KeyState;                /* 按键状态枚举类型定义 */

    static KeyState state = KEY_STATE_IDLE;  /* 当前状态机状态，函数间保持（静态局部变量） */
    static uint16 cnt = 0u;                  /* 消抖/释放计数器，用于计数连续检测次数 */
    static uint8 last_key = 0u;              /* 记录正在消抖的按键编号，用于对比一致性 */

    uint8 cur_key = 0u;  /* 当前扫描到的按键编号，0表示无按键按下 */

    /* 轮询四个按键GPIO引脚，低电平表示按下（按键接地，内部上拉） */
    /* 优先级：KEY1 > KEY2 > KEY3 > KEY4（同时按下时取编号小的） */
    if (gpio_get_level(KEY1) == 0)       /* 检测KEY1是否按下（低电平有效） */
        cur_key = 1u;                     /* KEY1按下，记录编号1 */
    else if (gpio_get_level(KEY2) == 0)  /* 检测KEY2是否按下 */
        cur_key = 2u;                     /* KEY2按下，记录编号2 */
    else if (gpio_get_level(KEY3) == 0)  /* 检测KEY3是否按下 */
        cur_key = 3u;                     /* KEY3按下，记录编号3 */
    else if (gpio_get_level(KEY4) == 0)  /* 检测KEY4是否按下 */
        cur_key = 4u;                     /* KEY4按下，记录编号4 */

    switch (state)  /* 根据当前状态机状态执行对应逻辑 */
    {
    case KEY_STATE_IDLE:
        /* 空闲状态：检测到任意按键按下，记录按键编号并进入消抖 */
        if (cur_key != 0u)  /* 有按键按下 */
        {
            last_key = cur_key;    /* 记录当前按下的按键编号 */
            cnt = 0u;              /* 重置消抖计数器 */
            state = KEY_STATE_DEBOUNCE;  /* 切换到消抖状态 */
        }
        break;  /* 无按键则保持IDLE状态 */

    case KEY_STATE_DEBOUNCE:
        /* 消抖状态：持续检测同一按键是否保持按下 */
        if (cur_key == last_key)  /* 当前按键与记录的按键一致，说明按键持续按下 */
        {
            cnt++;  /* 消抖计数器递增 */
            if (cnt >= DEBOUNCE_CNT)  /* 连续DEBOUNCE_CNT次检测到同一按键，确认有效按下 */
            {
                cnt = 0u;                    /* 重置计数器 */
                state = KEY_STATE_HOLD;      /* 消抖通过，进入按住状态 */
                return last_key;             /* 返回确认的按键编号（唯一一次返回非0值） */
            }
        }
        else
        {
            /* 消抖期间检测到不同按键或无按键，视为抖动，回到空闲 */
            state = KEY_STATE_IDLE;  /* 回到空闲状态 */
            cnt = 0u;                /* 重置计数器 */
        }
        break;

    case KEY_STATE_HOLD:
        /* 按住状态：等待按键松开（cur_key变为0） */
        if (cur_key == 0u)  /* 所有按键均已松开 */
        {
            cnt = 0u;                      /* 重置释放消抖计数器 */
            state = KEY_STATE_RELEASE;     /* 按键松开，进入释放消抖状态 */
        }
        break;

    case KEY_STATE_RELEASE:
        /* 释放消抖状态：确认按键已完全松开，防止抖动误触发 */
        if (cur_key == 0u)  /* 持续无按键，说明确实已松开 */
        {
            cnt++;  /* 释放消抖计数器递增 */
            if (cnt >= RELEASE_CNT)  /* 连续RELEASE_CNT次无按键 */
            {
                cnt = 0u;                /* 重置计数器 */
                state = KEY_STATE_IDLE;  /* 确认松开，回到空闲状态 */
            }
        }
        else
        {
            /* 释放消抖期间又检测到按键，回到按住状态（用户未真正松开） */
            state = KEY_STATE_HOLD;  /* 回到按住状态 */
            cnt = 0u;                /* 重置计数器 */
        }
        break;

    default:
        /* 未知状态，安全回到空闲 */
        state = KEY_STATE_IDLE;  /* 强制回到空闲状态，防止状态机卡死 */
        break;
    }

    return 0u;  /* 无确认的按键事件，返回0 */
}

/* ==================== 光标位置钳位 ==================== */
/**
 * cursor_clamp() - 将菜单光标位置限制在当前页面的有效范围内
 *
 * 防止页面切换后光标越界。例如从条目多的RA页面（10项）切换到
 * 条目少的MAIN页面（1项）时，光标可能超出范围，需要钳位到有效值。
 */
static void cursor_clamp(void)
{
    uint8 max_items = g_pages[now_page].item_count;  /* 获取当前页面的菜单条目总数 */

    if (max_items == 0u)               /* 页面无条目（防御性检查） */
        menu_cursor = 0u;              /* 光标归零 */
    else if (menu_cursor >= max_items) /* 光标超出范围 */
        menu_cursor = max_items - 1u;  /* 钳位到最后一个条目 */
}

/* ==================== GPIO初始化 ==================== */
/**
 * key_init_all() - 初始化所有按键、拨码开关和LED的GPIO引脚
 *
 * 4个按键和2个拨码开关均配置为输入模式（GPI），高电平默认，内部上拉。
 * 未按下时读取高电平，按下时接地读取低电平。
 * LED配置为输出模式（GPO），默认高电平（LED熄灭）。
 */
void key_init_all(void)
{
    gpio_init(KEY1, GPI, GPIO_HIGH, GPI_PULL_UP);      /* KEY1：输入模式，上拉，未按下为高电平 */
    gpio_init(KEY2, GPI, GPIO_HIGH, GPI_PULL_UP);      /* KEY2：输入模式，上拉 */
    gpio_init(KEY3, GPI, GPIO_HIGH, GPI_PULL_UP);      /* KEY3：输入模式，上拉 */
    gpio_init(KEY4, GPI, GPIO_HIGH, GPI_PULL_UP);      /* KEY4：输入模式，上拉 */
    gpio_init(SWITCH1, GPI, GPIO_HIGH, GPI_PULL_UP);   /* 拨码开关1：输入模式，上拉 */
    gpio_init(SWITCH2, GPI, GPIO_HIGH, GPI_PULL_UP);   /* 拨码开关2：输入模式，上拉 */
    gpio_init(TURN_RIGHT_LED, GPO, GPIO_HIGH, GPI_PULL_UP); /* 右转LED：输出模式，默认高电平（LED熄灭） */
}

/* ==================== 按键事件处理主函数 ==================== */
/**
 * key_process() - 处理按键事件，实现菜单导航和参数调节
 *
 * 每次被主循环调用时扫描一次按键，根据当前模式（选择/调节）执行不同操作：
 *
 * 通用功能（不受模式影响）：
 *   KEY1 -> 切换到下一页（循环）
 *   KEY2 -> 切换到上一页（循环）
 *
 * 选择模式（拨码开关未拨下）：
 *   KEY3 -> 光标上移（到头则循环到底部）
 *   KEY4 -> 光标下移（到底则循环到顶部）
 *
 * 调节模式（拨码开关任一拨下）：
 *   KEY3 -> 当前参数值增加一个步进（不超过最大值）
 *   KEY4 -> 当前参数值减少一个步进（不低于最小值）
 *   修改后自动调用menu_apply_adjusted_value()使参数生效
 *
 * 非RACE_MODE时，翻页会清屏以避免残留显示。
 */
void key_process(void)
{
    uint8 key = key_scan();  /* 调用按键扫描状态机，获取确认的按键事件编号 */

    if (key == 0u)           /* 无按键事件 */
        return;              /* 直接返回，不执行任何操作 */

    /* KEY1：切换到下一页（循环翻页） */
    if (key == 1u)
    {
        now_page = (MenuPage)((now_page + 1u) % PAGE_MAX);  /* 页码+1，到末尾循环回第0页 */
        menu_cursor = 0u;                                    /* 翻页后光标重置到第一个条目 */
#if !RACE_MODE
        tft180_clear();  /* 非比赛模式下清屏，避免上一页的残留内容叠加显示 */
#endif
        return;  /* 翻页后直接返回，本帧不再处理其他按键 */
    }

    /* KEY2：切换到上一页（循环翻页） */
    if (key == 2u)
    {
        now_page = (now_page == 0u) ? (MenuPage)(PAGE_MAX - 1u) : (MenuPage)(now_page - 1u);  /* 页码-1，第0页循环到末页 */
        menu_cursor = 0u;                                    /* 翻页后光标重置到第一个条目 */
#if !RACE_MODE
        tft180_clear();  /* 非比赛模式下清屏 */
#endif
        return;  /* 翻页后直接返回 */
    }

    /* 以下处理KEY3/KEY4，根据拨码开关状态决定是移动光标还是调节参数 */
    uint8 adjust_mode = dip_is_adjust_mode();   /* 读取拨码开关，判断当前是选择模式还是调节模式 */
    MenuPageDef *page = &g_pages[now_page];     /* 获取当前页面的定义结构体指针 */
    uint8 item_cnt = page->item_count;          /* 当前页面的菜单条目总数 */

    if (item_cnt == 0u)     /* 页面无条目（防御性检查） */
        return;             /* 无需处理，直接返回 */

    if (!adjust_mode)       /* 拨码开关未拨下 = 选择模式 */
    {
        /* ===== 选择模式：KEY3/KEY4移动光标 ===== */
        if (key == 3u)
            menu_cursor = (menu_cursor == 0u) ? (item_cnt - 1u) : (menu_cursor - 1u);  /* KEY3光标上移，到顶循环到底部 */
        if (key == 4u)
            menu_cursor = (menu_cursor + 1u) % item_cnt;  /* KEY4光标下移，到底循环到顶部 */
    }
    else                    /* 拨码开关拨下 = 调节模式 */
    {
        /* ===== 调节模式：KEY3/KEY4增减当前参数值 ===== */
        cursor_clamp();                              /* 先确保光标在有效范围内，防止越界访问 */
        MenuItem *cur = &page->items[menu_cursor];  /* 获取光标指向的菜单条目结构体指针 */

        if (key == 3u)      /* KEY3：增加参数值 */
        {
            /* KEY3：增加值，不超过最大值 */
            if (*cur->value + cur->step <= cur->max)  /* 步进增加后不超过最大值 */
                *cur->value += cur->step;             /* 执行步进增加 */
            else
                *cur->value = cur->max;               /* 已达最大值，钳位到最大值 */
        }

        if (key == 4u)      /* KEY4：减少参数值 */
        {
            /* KEY4：减少值，不低于最小值 */
            if (*cur->value - cur->step >= cur->min)  /* 步进减少后不低于最小值 */
                *cur->value -= cur->step;             /* 执行步进减少 */
            else
                *cur->value = cur->min;               /* 已达最小值，钳位到最小值 */
        }

        menu_apply_adjusted_value();  /* 参数修改后立即应用到硬件（如曝光时间） */
    }
}

/* ==================== 字符串拼接辅助函数 ==================== */
/**
 * append_str() - 将源字符串追加到目标缓冲区的指定位置
 *
 * @dst  目标缓冲区
 * @pos  当前写入位置（追加起始偏移）
 * @src  要追加的源字符串（以'\0'结尾）
 *
 * 返回值：追加完成后的新写入位置
 */
static uint8 append_str(char *dst, uint8 pos, const char *src)
{
    while (*src)                         /* 遍历源字符串直到遇到结束符'\0' */
        dst[pos++] = *src++;             /* 逐字符复制到目标缓冲区，同时推进源指针 */

    return pos;                          /* 返回新的写入位置（追加后的偏移量） */
}

/* ==================== 整数转字符串辅助函数 ==================== */
/**
 * append_int() - 将int16整数转换为十进制字符串并追加到缓冲区
 *
 * @dst  目标缓冲区
 * @pos  当前写入位置
 * @val  要转换的int16整数值
 *
 * 处理负号，使用反序生成法：先逐位取余存入临时数组，再反向写入目标缓冲区。
 * 支持-32768~32767范围内的所有int16值。
 *
 * 返回值：转换完成后的新写入位置
 */
static uint8 append_int(char *dst, uint8 pos, int16 val)
{
    int32 v = val;    /* 提升为int32避免取负溢出（-32768取反仍为int32范围） */
    char tmp[8];      /* 临时存储各位数字字符（反序存放），最多7位数字+负号 */
    uint8 n = 0u;     /* 数字位数计数器 */

    if (v < 0)        /* 处理负数 */
    {
        dst[pos++] = '-';  /* 负数先写入负号 */
        v = -v;            /* 取绝对值，后续按正数处理 */
    }

    /* 逐位取余，从个位开始存入tmp数组（反序） */
    do
    {
        tmp[n++] = (char)('0' + (v % 10));  /* 取最低位数字，转为ASCII字符存入临时数组 */
        v /= 10;                             /* 去掉已处理的最低位，准备处理下一位 */
    } while (v > 0);                         /* 直到所有位都处理完毕 */

    /* 将tmp中的数字字符反向写入目标缓冲区（恢复正序） */
    for (uint8 i = n; i > 0u; i--)           /* 从最高位到最低位遍历 */
        dst[pos++] = tmp[i - 1u];            /* 写入数字字符到目标缓冲区 */

    return pos;  /* 返回新的写入位置 */
}

/* ==================== 默认页面绘制函数 ==================== */
/**
 * default_draw() - 绘制当前菜单页面内容到TFT180显示屏
 *
 * @page  指向当前页面定义结构体的指针
 *
 * 根据当前页面类型执行不同的绘制逻辑：
 *
 * MAIN页面（特殊处理）：
 *   - 显示电机使能状态（EN:0/1）
 *   - 显示路线调试信息：当前步骤/总步骤（L:x/x）
 *   - 显示预判标志和动作（PF:x, PA:x）
 *   - 使用简短标签"EN"以节省显示空间
 *
 * 其他页面（通用处理）：
 *   - 顶部显示页面标题
 *   - 标题下方显示当前模式状态（[SW:SELECT]或[SW:ADJUST]）
 *   - 列表形式显示所有菜单条目，光标所在行前面显示">"标记
 *   - 每行格式：">标签名    数值"
 *   - 底部显示操作提示"K1/K2:Page K3/K4:+-"
 *
 * 在非RACE_MODE下才实际执行绘制。
 */
static void default_draw(MenuPageDef *page)
{
    char buf[32];                              /* 用于拼接显示字符串的缓冲区，最大31字符+结束符 */
    uint8 adjust_mode = dip_is_adjust_mode();  /* 获取当前拨码开关模式，用于显示模式提示 */

    if (now_page == PAGE_MAIN)  /* 判断当前是否为主页 */
    {
        /* 主页由 draw_line() 统一绘制，避免菜单文字与 ERR/ROW/THR 等调试值重叠 */
        return;  /* 主页绘制完毕，直接返回 */
    }

    /* ===== 非主页通用绘制逻辑 ===== */

    /* 顶部显示页面标题（如"MOTOR"、"CAM"、"PID"等） */
    tft180_show_string(0, 0, (char *)page->title);  /* 在左上角显示当前页面标题 */

    /* 标题下方显示当前操作模式提示 */
    tft180_show_string(0, 8, adjust_mode ? "[SW:ADJUST]" : "[SW:SELECT]");  /* 根据拨码开关状态显示模式提示 */

    /* 计算菜单列表的起始Y坐标，使列表整体垂直居中 */
    uint8 start_y = 120u - page->item_count * 8u;  /* 从底部向上排列，每项占8像素高度 */

    /* 遍历绘制当前页面的所有菜单条目 */
    for (uint8 i = 0u; i < page->item_count; i++)  /* 循环遍历每个菜单条目 */
    {
        MenuItem *item = &page->items[i];  /* 获取第i个菜单条目指针 */
        uint8 pos = 0u;                    /* 字符串缓冲区当前写入位置 */

        /* 拼接光标指示符：当前选中项显示">"，否则显示空格 */
        buf[pos++] = (i == menu_cursor) ? '>' : ' ';  /* 写入光标指示字符 */

        /* 拼接条目标签名（如"Kp"、"Speed"等） */
        pos = append_str(buf, pos, item->label);  /* 将标签字符串追加到缓冲区 */

        /* 用空格填充到固定宽度11字符，保证数值列对齐 */
        while (pos < 11u)       /* 如果当前写入位置未达到11字符 */
            buf[pos++] = ' ';   /* 用空格填充 */

        /* 拼接当前参数值的十进制字符串 */
        pos = append_int(buf, pos, *item->value);  /* 将参数值转为字符串追加到缓冲区 */
        buf[pos] = '\0';                           /* 写入字符串终止符 */

        /* 在TFT上显示该行菜单项，x=0兼容128像素宽屏幕，避免参数行越界触发assert */
        tft180_show_string(0, start_y + i * 8u, buf);  /* 显示拼接好的菜单行字符串 */
    }

    /* 底部显示操作提示文字 */
    /* 底部提示行暂不显示，避免部分TFT驱动在y=120边界处触发assert */
}

/* ==================== 菜单显示主函数 ==================== */
/**
 * menu_show() - 菜单页面显示主函数，由主循环周期性调用
 *
 * 根据RACE_MODE和TFT_OFF_WHEN_RUNNING配置决定是否执行绘制：
 *   - RACE_MODE=1时直接返回，比赛模式下不更新TFT以节省时间
 *   - TFT_OFF_WHEN_RUNNING=1时，运行期间完全关闭TFT显示
 *
 * 对于MAIN页面，使用draw_line()在TFT上显示二值化图像和叠加信息，
 * 并通过分频计数器控制刷新频率（运行和停止状态可设不同频率）。
 * 对于其他页面，调用default_draw()绘制参数列表。
 *
 * 分频机制：s_main_draw_cnt从0递增到draw_div-1时才执行一次绘制，
 * 中间的调用直接跳过，从而降低TFT刷新率，减少对实时控制的干扰。
 */
void menu_show(void)
{
#if RACE_MODE
    return;  /* 比赛模式下跳过所有TFT更新，节省CPU时间给控制任务 */
#else
    MenuPageDef *page = &g_pages[now_page];  /* 获取当前页面定义结构体指针 */
    static uint8 s_main_draw_cnt = 0u;       /* 主页绘制分频计数器，每次调用递增 */
    static uint8 s_tft_run_off = 0u;         /* 运行时TFT关闭标志，用于状态切换时清屏一次 */
    static uint32 s_last_main_frame = 0xFFFFFFFFu; /* 上次主页TFT已显示的视觉帧号 */
    uint8 need_default_draw = 1u;             /* 是否需要绘制菜单文字 */

#if TFT_OFF_WHEN_RUNNING
    /* 运行时关闭TFT模式：电机和负压都开启后关闭TFT显示以节省CPU资源 */
    if ((motor_enable != 0) && (vacuum_enable != 0u))  /* 电机和负压均已使能，进入运行状态 */
    {
        if (s_tft_run_off == 0u)  /* 刚进入运行状态（之前TFT是开着的） */
        {
            tft180_clear();          /* 刚进入运行状态时清屏一次，消除残留画面 */
            s_main_draw_cnt = 0u;    /* 重置分频计数器 */
            s_tft_run_off = 1u;      /* 标记TFT已关闭，防止重复清屏 */
        }
        return;  /* 运行期间不再绘制，直接返回 */
    }

    /* 电机停止后恢复TFT显示 */
    if (s_tft_run_off != 0u)  /* 之前TFT处于关闭状态，现在电机停止了 */
    {
        tft180_clear();          /* 刚停止时清屏一次，准备恢复显示 */
        s_main_draw_cnt = 0u;    /* 重置分频计数器 */
        s_tft_run_off = 0u;      /* 标记TFT已恢复显示 */
    }
#endif

    if (now_page == PAGE_MAIN)  /* 当前为主页 */
    {
        /* 主页：显示摄像头图像叠加信息，使用分频控制刷新率 */
        uint8 draw_div = (motor_enable != 0) ? MAIN_DRAW_DIV_RUN : MAIN_DRAW_DIV_STOP;  /* 根据运行状态选择分频系数 */

        need_default_draw = 0u;           /* 主页文字跟随图像新帧刷新，避免主循环空转时反复写屏 */

        if (g_tf_frame_count != s_last_main_frame)  /* 只有视觉处理完成新帧后才刷新TFT */
        {
            s_last_main_frame = g_tf_frame_count;   /* 记录当前已消费的视觉帧号 */

            if (s_main_draw_cnt == 0u)              /* 分频计数器归零时执行一次绘制 */
            {
                draw_line();                        /* 调用draw_line()显示二值化图像+边界线+叠加信息 */
                need_default_draw = 1u;             /* 图像刷新后同步刷新主页菜单文字 */
            }

            s_main_draw_cnt++;                      /* 分频计数器递增 */
            if (s_main_draw_cnt >= draw_div)        /* 计数器达到分频值 */
                s_main_draw_cnt = 0u;               /* 归零，触发下次绘制 */
        }
    }
    else                            /* 当前非主页 */
    {
        s_main_draw_cnt = 0u;      /* 非主页时重置计数器，确保切回主页时立即绘制 */
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
 * LED为低电平点亮（共阳极接法），设置GPIO输出低电平即可点亮。
 * 用于在检测到右转分支时点亮LED作为视觉指示。
 */
void turn_right_led_on(void)
{
    gpio_set_level(TURN_RIGHT_LED, GPIO_LOW);  /* 输出低电平，LED点亮 */
}

/**
 * turn_right_led_off() - 熄灭右转指示LED
 *
 * 设置GPIO输出高电平，LED熄灭。
 * 在右转分支处理完毕后调用。
 */
void turn_right_led_off(void)
{
    gpio_set_level(TURN_RIGHT_LED, GPIO_HIGH);  /* 输出高电平，LED熄灭 */
}
