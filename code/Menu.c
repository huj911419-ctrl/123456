#include "Menu.h"
#include "Track_funsion.h"

// ================================================================
//  硬件引脚定义
// ================================================================
#define KEY1 P11_3  // 上一页
#define KEY2 P11_2 // 下一页
#define KEY3 P20_6  // 确定键 / 增加值
#define KEY4 P20_7 // 取消键 / 减少值

#define TURN_RIGHT_LED P20_8 // 右转指示灯

#define SWITCH1 P33_11 // 拨码开关1位 (bit1)
#define SWITCH2 P33_12 // 拨码开关0位 (bit0)

// ================================================================
//  按键消抖逻辑说明
//  1. 检测到按键变化后，需要持续检测 DEBOUNCE_CNT 次确认
//  2. 确认后才返回键值，避免抖动误触
//  3. 按键释放也要检测 RELEASE_CNT 次，防止按键粘滞
// ================================================================

#define DEBOUNCE_CNT 20 // 按键消抖计数 循环1ms检测一次，20次=20ms消抖
#define RELEASE_CNT 10  // 按键释放计数

// ================================================================
//  变量说明：
//  1. 所有需要掉电保存的变量放在相应 .c 文件中声明
//  2. 如需在其他文件使用，在对应 .h 中 extern 声明
// ================================================================

// ---------- 主页参数 ----------
int16 motor_speed = 50;       // 电机速度   范围 0~100，base_speed = motor_speed*8
int16 motor_enable = 0;       // 电机使能   0=停止 1=运行
int16 detect_count = 0;       // 路口检测计数目标（0=禁用）
int16 route_seq[20] = {0};    // 路线序列: 0=直行 1=右转 2=左转
int16 route_len = 4;          // 路线长度 1~20
int16 route_step = 0;         // 当前步骤 0~(len-1)

// ---------- 摄像头参数 ----------
int16 threshold_bias = -10;    // Otsu阈值偏移量 范围 -50~50
int16 cam_exposure = 350;      // 曝光时间   范围 100~500（偏振片补偿，默认350）

// ---------- PID参数 ----------
int16 pid_kp = 4;             // Kp，实际 STEER_KP = pid_kp*0.8
int16 pid_ki = 2;              // Ki，实际 SPEED_KI = pid_ki*0.25
int16 pid_kd = 20;             // Kd，实际 STEER_KD = pid_kd*0.6

// ---------- 速度自适应参数 ----------
int16 sp_err_t1 = 5;           // 偏差阈值1（直道）
int16 sp_err_t2 = 30;          // 偏差阈值2（急弯）
int16 sp_ratio_1 = 100;        // 直道速度百分比
int16 sp_ratio_2 = 40;         // 弯道速度百分比
int16 steer_speed_k = 5;       // 转向速度耦合系数

// ---------- 直角弯参数 ----------
int16 ra_hard_inner = 0;       // 内轮duty（0=停转，负=反转更猛）
int16 ra_hard_outer = 500;    // 外轮duty
int16 ra_hard_yaw   = 60;      // 退出yaw角度 (30~85)
int16 ra_slow_row   = 70;      // 减速触发行号 (30~110)
int16 ra_slow_pct   = 60;      // 减速百分比 (10~100)
int16 ra_turn_row   = 100;     // 硬转触发行号 (50~115)
int16 ra_approach_frames = 15; // APPROACH阶段持续帧数 (5ms/帧=75ms，快速进弯)

// ================================================================
//  每个页面定义对应的 MenuItem 数组
//  格式：{ "显示名", &变量, 最小值, 最大值, 步进, "说明" }
// ================================================================

// 主页面（4个参数）
static MenuItem items_main[] = {
    {"Enable", &motor_enable, 0,   1,  1, "0=Stop 1=Run"},
    {"Speed",  &motor_speed,  0, 400, 20, "Speed (x8=actual)"},
    {"DetCnt", &detect_count, 0,  20,  1, "Stop after N (0=off)"},
    {"Exp",    &cam_exposure,100, 500, 10, "Exposure time"},
};

// PID页面（3个参数）
static MenuItem items_pid[] = {
    {"Kp",     &pid_kp, 0, 100, 1, "Steer P: bigger=sharper"},
    {"Ki",     &pid_ki, 0,  50, 1, "Speed I: bigger=faster"},
    {"Kd",     &pid_kd, 0,  50, 1, "Steer D: damp oscillation"},
};

// 速度自适应页面（5个参数，改名）
static MenuItem items_speed[] = {
    {"StrErr",  &sp_err_t1,      1,  50,  1, "Straight err threshold"},
    {"CrvErr",  &sp_err_t2,     10,  80,  1, "Curve err threshold"},
    {"StrSpd%", &sp_ratio_1,    20, 100,  5, "Straight speed %"},
    {"CrvSpd%", &sp_ratio_2,    20, 100,  5, "Curve speed %"},
    {"SpdCpl",  &steer_speed_k,  0,  50,  1, "Speed-steer coupling"},
};

// 直角弯参数页面（7个参数）
static MenuItem items_ra[] = {
    {"SlwRow",  &ra_slow_row,        30,  110,   5, "Slowdown IP row"},
    {"SlwPct",  &ra_slow_pct,        10,  100,   5, "Slowdown speed %"},
    {"TrnRow",  &ra_turn_row,        50,  115,   5, "Approach IP row"},
    {"AproF",   &ra_approach_frames,  5,  100,   5, "Approach frames (5ms)"},
    {"Outer",   &ra_hard_outer,     500, 5000, 100, "Outer wheel duty"},
    {"Inner",   &ra_hard_inner,   -2000, 5000, 100, "Inner wheel duty"},
    {"Yaw",     &ra_hard_yaw,        30,   85,   5, "Exit yaw angle(deg)"},
};

// 路线序列页面（Len + 10个方向步骤）
static MenuItem items_route[] = {
    {"Len",    &route_len,    1, 20, 1, "Total route steps"},
    {"Step00", &route_seq[0], 0,  2, 1, "0=Straight 1=Right 2=Left"},
    {"Step01", &route_seq[1], 0,  2, 1, "0=Straight 1=Right 2=Left"},
    {"Step02", &route_seq[2], 0,  2, 1, "0=Straight 1=Right 2=Left"},
    {"Step03", &route_seq[3], 0,  2, 1, "0=Straight 1=Right 2=Left"},
    {"Step04", &route_seq[4], 0,  2, 1, "0=Straight 1=Right 2=Left"},
    {"Step05", &route_seq[5], 0,  2, 1, "0=Straight 1=Right 2=Left"},
    {"Step06", &route_seq[6], 0,  2, 1, "0=Straight 1=Right 2=Left"},
    {"Step07", &route_seq[7], 0,  2, 1, "0=Straight 1=Right 2=Left"},
    {"Step08", &route_seq[8], 0,  2, 1, "0=Straight 1=Right 2=Left"},
    {"Step09", &route_seq[9], 0,  2, 1, "0=Straight 1=Right 2=Left"},
};

// ================================================================
//  页面定义
// ================================================================

static MenuPageDef g_pages[PAGE_MAX] = {

    // PAGE_MAIN - 主页面
    {
        .title = "MAIN",
        .items = items_main,
        .item_count = 4,
        .draw = NULL,
    },

    // PAGE_PID - PID参数（含IMU）
    {
        .title = "PID SET",
        .items = items_pid,
        .item_count = 3,
        .draw = NULL,
    },

    // PAGE_SPEED - 速度自适应
    {
        .title = "SPEED SET",
        .items = items_speed,
        .item_count = 5,
        .draw = NULL,
    },

    // PAGE_RA - 直角弯参数
    {
        .title = "RA SET",
        .items = items_ra,
        .item_count = 7,
        .draw = NULL,
    },

    // PAGE_ROUTE - 路线序列
    {
        .title = "ROUTE",
        .items = items_route,
        .item_count = 11,
        .draw = NULL,
    },

};

// ================================================================
//  全局状态
// ================================================================
MenuPage now_page = PAGE_MAIN;
uint8 menu_cursor = 0; // 当前选中的 MenuItem 索引

// ================================================================
//  内部函数：判断是否为调整模式
//  返回 0 = 选择模式（KEY3/KEY4 移动光标）
//  返回 非0 = 调整模式（KEY3/KEY4 修改选中参数值）
// ================================================================
static uint8 dip_is_adjust_mode(void)
{
    // 拨码开关任意一位拨到低电平 → 调整模式
    uint8 sw1 = (gpio_get_level(SWITCH1) == 0) ? 1 : 0;
    uint8 sw2 = (gpio_get_level(SWITCH2) == 0) ? 1 : 0;
    return sw1 | sw2;
}

// ================================================================
//  内部函数：按键扫描（含消抖处理）
//  返回 1~4 表示对应按键，0 = 无按键
// ================================================================
static uint8 key_scan(void)
{
    // 状态机定义
    typedef enum
    {
        KEY_STATE_IDLE = 0,      // 空闲，等待按键按下
        KEY_STATE_DEBOUNCE,       // 按键消抖检测
        KEY_STATE_HOLD,           // 确认按下，等待释放
        KEY_STATE_RELEASE,        // 按键释放检测
    } KeyState;

    static KeyState state = KEY_STATE_IDLE;
    static uint16 cnt = 0;
    static uint8 last_key = 0; // 记录上次按下的键值

    // 获取当前按下的按键
    uint8 cur_key = 0;
    if (gpio_get_level(KEY1) == 0)
        cur_key = 1;
    else if (gpio_get_level(KEY2) == 0)
        cur_key = 2;
    else if (gpio_get_level(KEY3) == 0)
        cur_key = 3;
    else if (gpio_get_level(KEY4) == 0)
        cur_key = 4;

    switch (state)
    {
    // ---- 状态1：空闲，等待按键按下 ----
    case KEY_STATE_IDLE:
        if (cur_key != 0)
        {
            last_key = cur_key; // 记录哪个键
            cnt = 0;
            state = KEY_STATE_DEBOUNCE;
        }
        break;

    // ---- 状态2：消抖检测，持续 DEBOUNCE_CNT 次确认 ----
    case KEY_STATE_DEBOUNCE:
        if (cur_key == last_key) // 按键相同
        {
            cnt++;
            if (cnt >= DEBOUNCE_CNT)
            {
                cnt = 0;
                state = KEY_STATE_HOLD;
                return last_key; // 唯一一次返回键值
            }
        }
        else // 按键松开了，放弃本次检测
        {
            state = KEY_STATE_IDLE;
            cnt = 0;
        }
        break;

    // ---- 状态3：按住不放，等待释放 ----
    case KEY_STATE_HOLD:
        if (cur_key == 0) // 检测到释放
        {
            cnt = 0;
            state = KEY_STATE_RELEASE;
        }
        // 一直按住什么也不做，不重复触发
        break;

    // ---- 状态4：释放消抖，等待 RELEASE_CNT 次确认 ----
    case KEY_STATE_RELEASE:
        if (cur_key == 0)
        {
            cnt++;
            if (cnt >= RELEASE_CNT)
            {
                cnt = 0;
                state = KEY_STATE_IDLE; // 回到初始状态
            }
        }
        else // 没有释放完成又按下了
        {
            state = KEY_STATE_HOLD;
            cnt = 0;
        }
        break;

    default:
        state = KEY_STATE_IDLE;
        break;
    }

    return 0; // 其他状态返回0，无按键
}

// ================================================================
//  内部函数：限制光标范围（换页时自动校正）
// ================================================================
static void cursor_clamp(void)
{
    uint8 max_items = g_pages[now_page].item_count;
    if (max_items == 0)
        menu_cursor = 0;
    else if (menu_cursor >= max_items)
        menu_cursor = max_items - 1;
}

// ================================================================
//  函数接口：初始化按键 & 拨码开关
// ================================================================
void key_init_all(void)
{
    gpio_init(KEY1, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY2, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY3, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY4, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(SWITCH1, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(SWITCH2, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(TURN_RIGHT_LED, GPO, GPIO_HIGH,GPI_PULL_UP );
}

// ================================================================
//  函数接口：按键处理（在循环中调用，10ms调用一次）
// ================================================================
void key_process(void)
{
    uint8 key = key_scan();
    if (key == 0)
        return;

    // ---- KEY1 / KEY2：翻页，任何模式都有效 ----
    if (key == 1) // 上一页
    {
        now_page = (now_page + 1) % PAGE_MAX;
        menu_cursor = 0;
        tft180_clear();
        return;
    }
    if (key == 2) // 下一页
    {
        now_page = (now_page == 0) ? (PAGE_MAX - 1) : (now_page - 1);
        menu_cursor = 0;
        tft180_clear();
        return;
    }

    // ---- KEY3 / KEY4：根据模式执行不同功能 ----
    uint8 adjust_mode = dip_is_adjust_mode();
    MenuPageDef *page = &g_pages[now_page];
    uint8 item_cnt = page->item_count;

    if (!adjust_mode)
    {
        // ---- 选择模式：KEY3/KEY4 移动光标 ----
        if (item_cnt == 0)
            return; // 无参数页面跳过

        if (key == 3) // 上移光标
            menu_cursor = (menu_cursor == 0) ? (item_cnt - 1) : (menu_cursor - 1);

        if (key == 4) // 下移光标
            menu_cursor = (menu_cursor + 1) % item_cnt;
    }
    else
    {
        // ---- 调整模式：KEY3/KEY4 修改当前选中参数值 ----
        if (item_cnt == 0)
            return; // 无参数页面跳过

        cursor_clamp();
        MenuItem *cur = &page->items[menu_cursor];

        if (key == 3) // 增加数值
        {
            if (*cur->value + cur->step <= cur->max)
                *cur->value += cur->step;
            else
                *cur->value = cur->max;
        }
        if (key == 4) // 减少数值
        {
            if (*cur->value - cur->step >= cur->min)
                *cur->value -= cur->step;
            else
                *cur->value = cur->min;
        }
    }
}

// ================================================================
//  内部函数：手动拼接字符串（避免 sprintf 开销）
// ================================================================
static uint8 append_str(char *dst, uint8 pos, const char *src)
{
    while (*src) dst[pos++] = *src++;
    return pos;
}

static uint8 append_int(char *dst, uint8 pos, int16 val)
{
    int32 v = val;
    if (v < 0) { dst[pos++] = '-'; v = -v; }
    char tmp[6];
    uint8 n = 0;
    do { tmp[n++] = '0' + (v % 10); v /= 10; } while (v > 0);
    for (uint8 i = n; i > 0; i--) dst[pos++] = tmp[i - 1];
    return pos;
}

// ================================================================
//  内部函数：默认绘制（标题 + 参数列表 + 底部提示）
// ================================================================
static void default_draw(MenuPageDef *page)
{
    char buf[32];
    uint8 adjust_mode = dip_is_adjust_mode();

    // 行0：显示标题（非主页面）
    if (now_page != PAGE_MAIN)
        tft180_show_string(0, 0, (char *)page->title);

    // 行1：显示当前模式提示（非主页面）
    if (now_page != PAGE_MAIN)
        tft180_show_string(0, 8, adjust_mode ? "[SW:ADJUST]" : "[SW:SELECT]");

    // 参数列表：从底部向上排版，避免多项时越界
    {
        uint8 start_y = 120 - page->item_count * 8;
        for (uint8 i = 0; i < page->item_count; i++)
        {
            MenuItem *item = &page->items[i];
            uint8 pos = 0;

            // 光标标记
            buf[pos++] = (i == menu_cursor) ? '>' : ' ';

            // 标签（左对齐，补空格到10字符）
            pos = append_str(buf, pos, item->label);
            while (pos < 11) buf[pos++] = ' ';

            // 数值
            pos = append_int(buf, pos, *item->value);
            buf[pos] = '\0';

            tft180_show_string(70, start_y + i * 8, buf);
        }
    }

    // 底部：显示当前选中参数的说明
    if (now_page != PAGE_MAIN && page->item_count > 0 && menu_cursor < page->item_count)
        tft180_show_string(0, 120, (char *)page->items[menu_cursor].desc);
}

// ================================================================
//  函数接口：菜单刷新显示（在主循环中调用）
// ================================================================
void menu_show(void)
{
    // 等待摄像头帧采集
    while (!mt9v03x_finish_flag)
        ;
    mt9v03x_finish_flag = 0;

    MenuPageDef *page = &g_pages[now_page];

    if (now_page == PAGE_MAIN)
    {
        draw_line();
        default_draw(page);
    }
    else
    {
        default_draw(page);
    }
}

void turn_right_led_on(void)
{
    gpio_set_level(TURN_RIGHT_LED, GPIO_LOW);
}

void turn_right_led_off(void)
{
    gpio_set_level(TURN_RIGHT_LED, GPIO_HIGH);
}
