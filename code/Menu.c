#include "Menu.h"
#include "Track_funsion.h"
#include "Pid.h"
#include "TFT_show_image.h"

#define KEY1 P11_3
#define KEY2 P11_2
#define KEY3 P20_6
#define KEY4 P20_7

#define TURN_RIGHT_LED P20_8
#define SWITCH1 P33_11
#define SWITCH2 P33_12

#define DEBOUNCE_CNT 20
#define RELEASE_CNT 10
#define MAIN_DRAW_DIV_STOP 1u
#define MAIN_DRAW_DIV_RUN  5u

int16 motor_speed = 100;
int16 motor_dir = 1;
int16 motor_enable = 0;
int16 motor_run_time = 60;

int16 cam_exposure = 200;

int16 pid_kp = 7;
int16 pid_ki = 2;
int16 pid_kd = 20;

int16 sp_err_t1 = 4;
int16 sp_err_t2 = 24;
int16 sp_ratio_1 = 100;
int16 sp_ratio_2 = 55;
int16 steer_speed_k = 3;

int16 ra_hard_inner = -200;
int16 ra_hard_outer = 1100;
int16 ra_hard_yaw = 60;
int16 ra_slow_row = 55;
int16 ra_slow_pct = 55;
int16 ra_turn_row = 75;
int16 ra_approach_frames = 6;

int16 yaw_kp = 0;

static MenuItem items_main[] = {
    {"Enable", &motor_enable, 0, 1, 1},
    {"RunTime", &motor_run_time, 3, 60, 1},
};

static MenuItem items_motor[] = {
    {"Speed", &motor_speed, 0, 400, 20},
    {"Dir", &motor_dir, 0, 1, 1},
};

static MenuItem items_cam[] = {
    {"Bias", &threshold_bias, -50, 50, 5},
    {"Expose", &cam_exposure, 100, 500, 10},
};

static MenuItem items_pid[] = {
    {"Kp", &pid_kp, 0, 100, 1},
    {"Ki", &pid_ki, 0, 50, 1},
    {"Kd", &pid_kd, 0, 50, 1},
};

static MenuItem items_speed[] = {
    {"StrErr", &sp_err_t1, 1, 50, 1},
    {"CrvErr", &sp_err_t2, 10, 80, 1},
    {"StrSpd%", &sp_ratio_1, 20, 100, 5},
    {"CrvSpd%", &sp_ratio_2, 20, 100, 5},
    {"SpdCpl", &steer_speed_k, 0, 50, 1},
};

static MenuItem items_ra[] = {
    {"SlwRow", &ra_slow_row, 30, 110, 5},
    {"SlwPct", &ra_slow_pct, 10, 100, 5},
    {"TrnRow", &ra_turn_row, 50, 115, 5},
    {"AproF", &ra_approach_frames, 5, 100, 5},
    {"Outer", &ra_hard_outer, 500, 5000, 100},
    {"Inner", &ra_hard_inner, -2000, 5000, 100},
    {"Yaw", &ra_hard_yaw, 30, 85, 5},
    {"IpCol", &ip_col_offset, 0, 7, 1},
    {"IpL", &ip_left_col, 0, 47, 1},
    {"IpR", &ip_right_col, 47, 93, 1},
};

static MenuItem items_imu[] = {
    {"YAW Kp", &yaw_kp, 0, 100, 1},
};

static MenuPageDef g_pages[PAGE_MAX] = {
    { .title = "MAIN", .items = items_main, .item_count = 2, .draw = NULL },
    { .title = "MOTOR", .items = items_motor, .item_count = 2, .draw = NULL },
    { .title = "CAM", .items = items_cam, .item_count = 2, .draw = NULL },
    { .title = "PID", .items = items_pid, .item_count = 3, .draw = NULL },
    { .title = "SPEED", .items = items_speed, .item_count = 5, .draw = NULL },
    { .title = "RA", .items = items_ra, .item_count = 10, .draw = NULL },
    { .title = "IMU", .items = items_imu, .item_count = 1, .draw = NULL },
};

MenuPage now_page = PAGE_MAIN;
uint8 menu_cursor = 0u;

static uint8 dip_is_adjust_mode(void)
{
    uint8 sw1 = (gpio_get_level(SWITCH1) == 0) ? 1u : 0u;
    uint8 sw2 = (gpio_get_level(SWITCH2) == 0) ? 1u : 0u;
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
    static uint16 cnt = 0u;
    static uint8 last_key = 0u;

    uint8 cur_key = 0u;

    if (gpio_get_level(KEY1) == 0)
        cur_key = 1u;
    else if (gpio_get_level(KEY2) == 0)
        cur_key = 2u;
    else if (gpio_get_level(KEY3) == 0)
        cur_key = 3u;
    else if (gpio_get_level(KEY4) == 0)
        cur_key = 4u;

    switch (state)
    {
    case KEY_STATE_IDLE:
        if (cur_key != 0u)
        {
            last_key = cur_key;
            cnt = 0u;
            state = KEY_STATE_DEBOUNCE;
        }
        break;

    case KEY_STATE_DEBOUNCE:
        if (cur_key == last_key)
        {
            cnt++;
            if (cnt >= DEBOUNCE_CNT)
            {
                cnt = 0u;
                state = KEY_STATE_HOLD;
                return last_key;
            }
        }
        else
        {
            state = KEY_STATE_IDLE;
            cnt = 0u;
        }
        break;

    case KEY_STATE_HOLD:
        if (cur_key == 0u)
        {
            cnt = 0u;
            state = KEY_STATE_RELEASE;
        }
        break;

    case KEY_STATE_RELEASE:
        if (cur_key == 0u)
        {
            cnt++;
            if (cnt >= RELEASE_CNT)
            {
                cnt = 0u;
                state = KEY_STATE_IDLE;
            }
        }
        else
        {
            state = KEY_STATE_HOLD;
            cnt = 0u;
        }
        break;

    default:
        state = KEY_STATE_IDLE;
        break;
    }

    return 0u;
}

static void cursor_clamp(void)
{
    uint8 max_items = g_pages[now_page].item_count;

    if (max_items == 0u)
        menu_cursor = 0u;
    else if (menu_cursor >= max_items)
        menu_cursor = max_items - 1u;
}

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

void key_process(void)
{
    uint8 key = key_scan();

    if (key == 0u)
        return;

    if (key == 1u)
    {
        now_page = (MenuPage)((now_page + 1u) % PAGE_MAX);
        menu_cursor = 0u;
#if !RACE_MODE
        tft180_clear();
#endif
        return;
    }

    if (key == 2u)
    {
        now_page = (now_page == 0u) ? (MenuPage)(PAGE_MAX - 1u) : (MenuPage)(now_page - 1u);
        menu_cursor = 0u;
#if !RACE_MODE
        tft180_clear();
#endif
        return;
    }

    uint8 adjust_mode = dip_is_adjust_mode();
    MenuPageDef *page = &g_pages[now_page];
    uint8 item_cnt = page->item_count;

    if (item_cnt == 0u)
        return;

    if (!adjust_mode)
    {
        if (key == 3u)
            menu_cursor = (menu_cursor == 0u) ? (item_cnt - 1u) : (menu_cursor - 1u);

        if (key == 4u)
            menu_cursor = (menu_cursor + 1u) % item_cnt;
    }
    else
    {
        cursor_clamp();
        MenuItem *cur = &page->items[menu_cursor];

        if (key == 3u)
        {
            if (*cur->value + cur->step <= cur->max)
                *cur->value += cur->step;
            else
                *cur->value = cur->max;
        }

        if (key == 4u)
        {
            if (*cur->value - cur->step >= cur->min)
                *cur->value -= cur->step;
            else
                *cur->value = cur->min;
        }
    }
}

static uint8 append_str(char *dst, uint8 pos, const char *src)
{
    while (*src)
        dst[pos++] = *src++;

    return pos;
}

static uint8 append_int(char *dst, uint8 pos, int16 val)
{
    int32 v = val;
    char tmp[8];
    uint8 n = 0u;

    if (v < 0)
    {
        dst[pos++] = '-';
        v = -v;
    }

    do
    {
        tmp[n++] = (char)('0' + (v % 10));
        v /= 10;
    } while (v > 0);

    for (uint8 i = n; i > 0u; i--)
        dst[pos++] = tmp[i - 1u];

    return pos;
}

static void default_draw(MenuPageDef *page)
{
    char buf[32];
    uint8 adjust_mode = dip_is_adjust_mode();

    if (now_page == PAGE_MAIN)
    {
        const char *short_label[2] = {"EN", "RT"};

        for (uint8 i = 0u; i < page->item_count && i < 2u; i++)
        {
            MenuItem *item = &page->items[i];
            uint8 y = 82u + i * 16u;

            tft180_show_string(68, y, (i == menu_cursor) ? ">" : " ");
            tft180_show_string(74, y, (char *)short_label[i]);
            tft180_show_string(86, y, ":");
            tft180_show_int(92, y, *item->value, (i == 0u) ? 1u : 3u);
        }
        return;
    }

    tft180_show_string(0, 0, (char *)page->title);
    tft180_show_string(0, 8, adjust_mode ? "[SW:ADJUST]" : "[SW:SELECT]");

    uint8 start_y = 120u - page->item_count * 8u;

    for (uint8 i = 0u; i < page->item_count; i++)
    {
        MenuItem *item = &page->items[i];
        uint8 pos = 0u;

        buf[pos++] = (i == menu_cursor) ? '>' : ' ';
        pos = append_str(buf, pos, item->label);

        while (pos < 11u)
            buf[pos++] = ' ';

        pos = append_int(buf, pos, *item->value);
        buf[pos] = '\0';

        tft180_show_string(70, start_y + i * 8u, buf);
    }

    tft180_show_string(0, 120, "K1/K2:Page K3/K4:+-");
}

void menu_show(void)
{
#if RACE_MODE
    return;
#else
    MenuPageDef *page = &g_pages[now_page];
    static uint8 s_main_draw_cnt = 0u;

    if (now_page == PAGE_MAIN)
    {
        uint8 draw_div = (motor_enable != 0) ? MAIN_DRAW_DIV_RUN : MAIN_DRAW_DIV_STOP;

        if (s_main_draw_cnt == 0u)
            draw_line();

        s_main_draw_cnt++;
        if (s_main_draw_cnt >= draw_div)
            s_main_draw_cnt = 0u;
    }
    else
    {
        s_main_draw_cnt = 0u;
    }

    default_draw(page);
#endif
}

void turn_right_led_on(void)
{
    gpio_set_level(TURN_RIGHT_LED, GPIO_LOW);
}

void turn_right_led_off(void)
{
    gpio_set_level(TURN_RIGHT_LED, GPIO_HIGH);
}
