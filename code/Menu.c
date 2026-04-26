#include "Menu.h"
#include "Track_funsion.h"
#include "string.h"
#include "stdio.h"

// ================================================================
//  硬件引脚定义
// ================================================================
#define KEY1 P20_6 // 下一页
#define KEY2 P20_7 // 上一页
#define KEY3 P11_3 // 光标上移 / 数值增加
#define KEY4 P11_2 // 光标下移 / 数值减少

#define SWITCH1 P33_11 // 拨码高位 (bit1)
#define SWITCH2 P33_12 // 拨码低位 (bit0)

// ================================================================
//  消抖逻辑：
//  1. 检测到按键按下后，必须连续稳定 DEBOUNCE_CNT 次才确认
//  2. 确认后立即返回键值，并锁定——必须完全松手才能响应下一次
//  3. 松手也要稳定 RELEASE_CNT 次才算真正松开（防止松手抖动）
// ================================================================

#define DEBOUNCE_CNT 20 // 按下消抖计数（次数 × 调用周期 = 消抖时间）
                        // 例如主循环 1ms 调用一次，20次 = 20ms 消抖
#define RELEASE_CNT 10  // 松手消抖计数
//  【用户配置区 2/2】：定义你的实际变量 & 页面内容
//
//  步骤：
//  1. 在下方声明你需要调节的变量
//  2. 为每个页面填写 MenuItem 数组（无参数的页面跳过）
//  3. 在 g_pages[] 数组中按 MenuPage 枚举顺序填写每页配置
//  4. 如需自定义绘制逻辑，实现对应的 show_xxx() 函数并传入
// ================================================================

// ---------- ① 声明你的参数变量 ----------
// 示例（请替换为你的实际变量）：
// int16 motor_speed  = 50;
// int16 servo_angle  = 90;
// int16 cam_threshold = 128;

// ---------- ② 为每个有参数的页面定义 MenuItem 数组 ----------
// 格式：{ "显示名", &变量, 最小值, 最大值, 步长 }
//
// 示例（PAGE_MAIN 无参数，PAGE_MY_PAGE_A 有两个参数）：
//
// static MenuItem items_my_page_a[] = {
//     { "Speed",     &motor_speed,   0,   100, 5 },
//     { "Threshold", &cam_threshold, 0,   255, 5 },
// };

// ---------- ③ 自定义绘制函数（可选）----------
// 若某页除参数列表外还需要显示额外内容（如摄像头画面），在此实现
//
// static void show_main_page(void)
// {
//     // 绘制摄像头画面等
//     draw_line();
// }

// ---------- ④ 页面总表（顺序必须与 MenuPage 枚举一致）----------
/*static MenuPageDef g_pages[PAGE_MAX] = {
    // PAGE_MAIN
    {
        .title = "MAIN",
        .items = NULL, // 主页无可调参数
        .item_count = 0,
        .draw = NULL, // 替换为 show_main_page 即可
    },
    // ↓↓↓ 按枚举顺序继续添加 ↓↓↓
    // PAGE_MY_PAGE_A
    // {
    //     .title      = "MY PAGE A",
    //     .items      = items_my_page_a,
    //     .item_count = sizeof(items_my_page_a) / sizeof(items_my_page_a[0]),
    //     .draw       = NULL,   // 框架会自动绘制参数列表
    // },
};*/

// ================================================================
//  第二步：声明你要调节的变量
//  （其他 .c 文件要用的话，在对应 .h 里 extern 声明一下）
// ================================================================
int16 motor_speed = 50; // 电机速度   范围 0~100
int16 motor_dir = 1;    // 电机方向   0=反转 1=正转

int16 cam_threshold = 128; // 二值化阈值 范围 0~255
int16 cam_exposure = 200;  // 曝光时间   范围 100~500

int16 pid_kp = 15; // Kp×10，实际用时除以10，即 1.5
int16 pid_ki = 2;  // Ki×10
int16 pid_kd = 8;  // Kd×10

// ================================================================
//  第三步：为有参数的页面写 MenuItem 数组
//  格式： { "显示名称", &变量名, 最小值, 最大值, 每次步长 }
// ================================================================

// 电机页的参数列表（2个参数）
static MenuItem items_motor[] = {
    {"Speed", &motor_speed, 0, 100, 5}, // 每按一次±5
    {"Dir", &motor_dir, 0, 1, 1},       // 0或1切换
};

// 摄像头页的参数列表（2个参数）
static MenuItem items_cam[] = {
    {"Thresh", &cam_threshold, 0, 255, 5},   // 每按一次±5
    {"Expose", &cam_exposure, 100, 500, 10}, // 每按一次±10
};

// PID页的参数列表（3个参数）
static MenuItem items_pid[] = {
    {"Kp x10", &pid_kp, 0, 100, 1}, // 实际Kp = pid_kp/10.0
    {"Ki x10", &pid_ki, 0, 50, 1},
    {"Kd x10", &pid_kd, 0, 50, 1},
};

// ================================================================
//  第四步（可选）：写自定义绘制函数
//  只有当这页除了参数列表之外还要显示别的东西时才需要
//  普通参数页直接填 NULL，框架会自动画
// ================================================================

// 主页：画摄像头线条，不需要参数列表
static void show_main_page(void)
{
    draw_line(); // 你原来的画线函数，直接放进来
}

// ================================================================
//  第五步：填写页面总表 g_pages[]
//  顺序必须和 Menu.h 里的枚举顺序完全一致！
// ================================================================
static MenuPageDef g_pages[PAGE_MAX] = {

    // PAGE_MAIN —— 主页，无参数，用自定义绘制
    {
        .title = "MAIN",
        .items = NULL, // 没有参数
        .item_count = 0,
        .draw = show_main_page, // 用自定义函数
    },

    // PAGE_MOTOR —— 电机设置页，2个参数，框架自动画列表
    {
        .title = "MOTOR SET",
        .items = items_motor, // 上面定义的数组
        .item_count = 2,      // 数组里有几个就填几
        .draw = NULL,         // NULL = 框架自动画
    },

    // PAGE_CAM —— 摄像头设置页，2个参数
    {
        .title = "CAM SET",
        .items = items_cam,
        .item_count = 2,
        .draw = NULL,
    },

    // PAGE_PID —— PID设置页，3个参数
    {
        .title = "PID SET",
        .items = items_pid,
        .item_count = 3,
        .draw = NULL,
    },

};

// ================================================================
//  全局状态
// ================================================================
MenuPage now_page = PAGE_MAIN;
uint8 menu_cursor = 0; // 当前选中的 MenuItem 索引

// ================================================================
//  内部：拨码开关读取
//  返回 0 = 选择模式（KEY3/KEY4 移动光标）
//  返回 非0 = 调节模式（KEY3/KEY4 修改选中项的值）
// ================================================================
static uint8 dip_is_adjust_mode(void)
{
    // 任意一位拨码拨到低电平 → 进入调节模式
    // 如果你只想用单个开关控制，只保留其中一个判断即可
    uint8 sw1 = (gpio_get_level(SWITCH1) == 0) ? 1 : 0;
    uint8 sw2 = (gpio_get_level(SWITCH2) == 0) ? 1 : 0;
    return sw1 | sw2;
}

// ================================================================
//  内部：按键扫描（消抖）
//  返回 1~4 表示对应按键触发，0 = 无触发
// ================================================================
static uint8 key_scan(void)
{
    // 状态机
    typedef enum
    {
        KEY_STATE_IDLE = 0, // 空闲，等待按键按下
        KEY_STATE_DEBOUNCE, // 按下后消抖中
        KEY_STATE_HOLD,     // 已确认按下，等待松手
        KEY_STATE_RELEASE,  // 松手消抖中
    } KeyState;

    static KeyState state = KEY_STATE_IDLE;
    static uint16 cnt = 0;
    static uint8 last_key = 0; // 消抖期间记录的键值

    // 读取当前有没有按键按下
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
    // ---- 空闲：等待有按键按下 ----
    case KEY_STATE_IDLE:
        if (cur_key != 0)
        {
            last_key = cur_key; // 记录是哪个键
            cnt = 0;
            state = KEY_STATE_DEBOUNCE;
        }
        break;

    // ---- 消抖：按键必须连续稳定 DEBOUNCE_CNT 次 ----
    case KEY_STATE_DEBOUNCE:
        if (cur_key == last_key) // 还是同一个键
        {
            cnt++;
            if (cnt >= DEBOUNCE_CNT)
            {
                cnt = 0;
                state = KEY_STATE_HOLD;
                return last_key; // ← 唯一一次返回键值
            }
        }
        else // 中途变了（是抖动），重新开始
        {
            state = KEY_STATE_IDLE;
            cnt = 0;
        }
        break;

    // ---- 按住：等待松手 ----
    case KEY_STATE_HOLD:
        if (cur_key == 0) // 检测到松手迹象
        {
            cnt = 0;
            state = KEY_STATE_RELEASE;
        }
        // 如果一直按住，什么也不做（不重复触发）
        break;

    // ---- 松手消抖：稳定 RELEASE_CNT 次才回到空闲 ----
    case KEY_STATE_RELEASE:
        if (cur_key == 0)
        {
            cnt++;
            if (cnt >= RELEASE_CNT)
            {
                cnt = 0;
                state = KEY_STATE_IDLE; // 真正松手，准备下次按键
            }
        }
        else // 还没松干净，重新等
        {
            state = KEY_STATE_HOLD;
            cnt = 0;
        }
        break;

    default:
        state = KEY_STATE_IDLE;
        break;
    }

    return 0; // 其余状态均返回 0，不触发
}

// ================================================================
//  内部：光标边界限制（切页时自动归零）
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
//  公共接口：初始化按键 & 拨码引脚
// ================================================================
void key_init_all(void)
{
    gpio_init(KEY1, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY2, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY3, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY4, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(SWITCH1, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(SWITCH2, GPI, GPIO_HIGH, GPI_PULL_UP);
}

// ================================================================
//  公共接口：按键处理（放入主循环或 10ms 定时器）
// ================================================================
void key_process(void)
{
    uint8 key = key_scan();
    if (key == 0)
        return;

    // ---- KEY1 / KEY2：翻页（任何模式均生效）----
    if (key == 1) // 下一页
    {
        now_page = (now_page + 1) % PAGE_MAX;
        menu_cursor = 0;
        tft180_clear();
        return;
    }
    if (key == 2) // 上一页
    {
        now_page = (now_page == 0) ? (PAGE_MAX - 1) : (now_page - 1);
        menu_cursor = 0;
        tft180_clear();
        return;
    }

    // ---- KEY3 / KEY4：由拨码决定行为 ----
    uint8 adjust_mode = dip_is_adjust_mode();
    MenuPageDef *page = &g_pages[now_page];
    uint8 item_cnt = page->item_count;

    if (!adjust_mode)
    {
        // ---- 选择模式：上下移动光标 ----
        if (item_cnt == 0)
            return; // 本页无可调项，忽略

        if (key == 3) // 光标上移
            menu_cursor = (menu_cursor == 0) ? (item_cnt - 1) : (menu_cursor - 1);

        if (key == 4) // 光标下移
            menu_cursor = (menu_cursor + 1) % item_cnt;
    }
    else
    {
        // ---- 调节模式：修改当前选中项的值 ----
        if (item_cnt == 0)
            return; // 本页无可调项，忽略

        cursor_clamp();
        MenuItem *cur = &page->items[menu_cursor];

        if (key == 3) // 数值增加
        {
            if (*cur->value + cur->step <= cur->max)
                *cur->value += cur->step;
            else
                *cur->value = cur->max;
        }
        if (key == 4) // 数值减少
        {
            if (*cur->value - cur->step >= cur->min)
                *cur->value -= cur->step;
            else
                *cur->value = cur->min;
        }
    }
}

// ================================================================
//  内部：框架默认绘制（标题 + 参数列表 + 光标 + 拨码状态提示）
// ================================================================
static void default_draw(MenuPageDef *page)
{
    char buf[32];
    uint8 adjust_mode = dip_is_adjust_mode();

    // 第0行：标题
    tft180_show_string(0, 0, (char *)page->title);

    // 第1行：当前模式提示
    tft180_show_string(0, 8, adjust_mode ? "[SW:ADJUST]" : "[SW:SELECT]");

    // 第2行起：参数列表
    for (uint8 i = 0; i < page->item_count; i++)
    {
        MenuItem *item = &page->items[i];
        // 光标标记
        const char *cursor_mark = (i == menu_cursor) ? ">" : " ";
        sprintf(buf, "%s%-10s%d", cursor_mark, item->label, *item->value);
        tft180_show_string(0, 16 + i * 8, buf);
    }

    // 底部操作提示
    tft180_show_string(0, 120, "K1/K2:Page K3/K4:+-");
}

// ================================================================
//  公共接口：菜单刷新显示（放入主循环）
// ================================================================
void menu_show(void)
{
    // 等待摄像头帧（如不使用摄像头可删除这两行）
    while (!mt9v03x_finish_flag)
        ;
    mt9v03x_finish_flag = 0;

    MenuPageDef *page = &g_pages[now_page];

    // 优先使用自定义绘制函数，没有则走框架默认绘制
    if (page->draw != NULL)
        page->draw();
    else
        default_draw(page);
}
