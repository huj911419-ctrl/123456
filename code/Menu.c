#include "Menu.h"
#include "Track_funsion.h"
#include "Pid.h"
#include "TFT_show_image.h"

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

#define DEBOUNCE_CNT 20
#define RELEASE_CNT 10

// ---------- 主页参数 ----------
int16 motor_speed = 80;
int16 motor_enable = 0;
int16 motor_run_time = 60;

// ---------- 摄像头参数 ----------
int16 cam_exposure = 200;

// ---------- PID参数 ----------
int16 pid_kp = 6;
int16 pid_ki = 2;
int16 pid_kd = 8;

// ---------- 速度自适应参数 ----------
int16 sp_err_t1 = 5;
int16 sp_err_t2 = 30;
int16 sp_ratio_1 = 100;
int16 sp_ratio_2 = 40;
int16 steer_speed_k = 5;

// ---------- 直角弯参数 ----------
int16 ra_hard_inner = 0;
int16 ra_hard_outer = 500;
int16 ra_hard_yaw   = 60;
int16 ra_slow_row   = 40;
int16 ra_slow_pct   = 60;
int16 ra_turn_row   = 55;//作用：
int16 ra_approach_frames = 5;//作用：

// ---------- IMU参数 ----------
int16 yaw_kp = 0;

// ================================================================
//  菜单项定义
// ================================================================

static MenuItem items_main[] = {
    {"Enable", &motor_enable, 0, 1, 1},
    {"Speed", &motor_speed, 0, 400, 20},
    {"RunTime", &motor_run_time, 3, 60, 1},
};

static MenuItem items_pid[] = {
    {"Kp", &pid_kp, 0, 100, 1},
    {"Ki", &pid_ki, 0, 50, 1},
    {"Kd", &pid_kd, 0, 50, 1},
};

static MenuItem items_speed[] = {
    {"StrErr",  &sp_err_t1,      1,  50,  1},
    {"CrvErr",  &sp_err_t2,     10,  80,  1},
    {"StrSpd%", &sp_ratio_1,    20, 100,  5},
    {"CrvSpd%", &sp_ratio_2,    20, 100,  5},
    {"SpdCpl",  &steer_speed_k,  0,  50,  1},
};

static MenuItem items_ra[] = {
    {"SlwRow",  &ra_slow_row,        30,  110,   5},
    {"SlwPct",  &ra_slow_pct,        10,  100,   5},
    {"TrnRow",  &ra_turn_row,        50,  115,   5},
    {"AproF",   &ra_approach_frames,  5,  100,   5},
    {"Outer",   &ra_hard_outer,     500, 5000, 100},
    {"Inner",   &ra_hard_inner,   -2000, 5000, 100},
    {"Yaw",     &ra_hard_yaw,        30,   85,   5},
    {"IpCol",   &ip_col_offset,       0,    7,   1},
    {"IpL",     &ip_left_col,         0,   47,   1},
    {"IpR",     &ip_right_col,       47,   93,   1},
};

// ================================================================
//  页面定义
// ================================================================

static MenuPageDef g_pages[PAGE_MAX] = {
    { .title = "MAIN",     .items = items_main,   .item_count = 3, .draw = NULL },
    { .title = "PID",      .items = items_pid,    .item_count = 3, .draw = NULL },
    { .title = "SPEED",    .items = items_speed,  .item_count = 5, .draw = NULL },
    { .title = "RA",       .items = items_ra,     .item_count = 10, .draw = NULL },
};

// ================================================================
//  全局状态
// ================================================================
MenuPage now_page = PAGE_MAIN;
uint8 menu_cursor = 0;

// ================================================================
//  内部函数
// ================================================================
static uint8 dip_is_adjust_mode(void)
{
    uint8 sw1 = (gpio_get_level(SWITCH1) == 0) ? 1 : 0;
    uint8 sw2 = (gpio_get_level(SWITCH2) == 0) ? 1 : 0;
    return sw1 | sw2;
}

static uint8 key_scan(void)
{
    typedef enum
    {
        KEY_STATE_IDLE = 0,
        KEY_STATE_DEBOUNCE,
        KEY_STATE_HOLD,
        KEY_STATE_RELEASE,
    } KeyState;

    static KeyState state = KEY_STATE_IDLE;
    static uint16 cnt = 0;
    static uint8 last_key = 0;

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
    case KEY_STATE_IDLE:
        if (cur_key != 0)
        {
            last_key = cur_key;
            cnt = 0;
            state = KEY_STATE_DEBOUNCE;
        }
        break;

    case KEY_STATE_DEBOUNCE:
        if (cur_key == last_key)
        {
            cnt++;
            if (cnt >= DEBOUNCE_CNT)
            {
                cnt = 0;
                state = KEY_STATE_HOLD;
                return last_key;
            }
        }
        else
        {
            state = KEY_STATE_IDLE;
            cnt = 0;
        }
        break;

    case KEY_STATE_HOLD:
        if (cur_key == 0)
        {
            cnt = 0;
            state = KEY_STATE_RELEASE;
        }
        break;

    case KEY_STATE_RELEASE:
        if (cur_key == 0)
        {
            cnt++;
            if (cnt >= RELEASE_CNT)
            {
                cnt = 0;
                state = KEY_STATE_IDLE;
            }
        }
        else
        {
            state = KEY_STATE_HOLD;
            cnt = 0;
        }
        break;

    default:
        state = KEY_STATE_IDLE;
        break;
    }

    return 0;
}

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
    gpio_init(TURN_RIGHT_LED, GPO, GPIO_HIGH, GPI_PULL_UP);
}

// ================================================================
//  函数接口：按键处理
// ================================================================
void key_process(void)
{
    uint8 key = key_scan();
    if (key == 0)
        return;

    if (key == 1)
    {
        now_page = (now_page + 1) % PAGE_MAX;
        menu_cursor = 0;
        tft180_clear();
        return;
    }
    if (key == 2)
    {
        now_page = (now_page == 0) ? (PAGE_MAX - 1) : (now_page - 1);
        menu_cursor = 0;
        tft180_clear();
        return;
    }

    uint8 adjust_mode = dip_is_adjust_mode();
    MenuPageDef *page = &g_pages[now_page];
    uint8 item_cnt = page->item_count;

    if (!adjust_mode)
    {
        if (item_cnt == 0)
            return;

        if (key == 3)
            menu_cursor = (menu_cursor == 0) ? (item_cnt - 1) : (menu_cursor - 1);

        if (key == 4)
            menu_cursor = (menu_cursor + 1) % item_cnt;
    }
    else
    {
        if (item_cnt == 0)
            return;

        cursor_clamp();
        MenuItem *cur = &page->items[menu_cursor];

        if (key == 3)
        {
            if (*cur->value + cur->step <= cur->max)
                *cur->value += cur->step;
            else
                *cur->value = cur->max;
        }
        if (key == 4)
        {
            if (*cur->value - cur->step >= cur->min)
                *cur->value -= cur->step;
            else
                *cur->value = cur->min;
        }
    }
}

// ================================================================
//  字符串拼接辅助
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
//  默认绘制
// ================================================================
static void default_draw(MenuPageDef *page)
{
    char buf[32];
    uint8 adjust_mode = dip_is_adjust_mode();

    if (now_page != PAGE_MAIN)
        tft180_show_string(0, 0, (char *)page->title);

    if (now_page != PAGE_MAIN)
        tft180_show_string(0, 8, adjust_mode ? "[SW:ADJUST]" : "[SW:SELECT]");

    {
        uint8 start_y = 120 - page->item_count * 8;
        for (uint8 i = 0; i < page->item_count; i++)
        {
            MenuItem *item = &page->items[i];
            uint8 pos = 0;

            buf[pos++] = (i == menu_cursor) ? '>' : ' ';

            pos = append_str(buf, pos, item->label);
            while (pos < 11) buf[pos++] = ' ';

            pos = append_int(buf, pos, *item->value);
            buf[pos] = '\0';

            tft180_show_string(70, start_y + i * 8, buf);
        }
    }

    if (now_page != PAGE_MAIN)
        tft180_show_string(0, 120, "K1/K2:Page K3/K4:+-");
}

// ================================================================
//  函数接口：菜单刷新显示
// ================================================================
void menu_show(void)
{
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
