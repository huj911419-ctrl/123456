#include "Menu.h"
#include "Track_funsion.h"
#include "string.h"
#include "stdio.h"

// == == == == == == == == == == 全局变量定义 == == == == == == == == == ==
MenuPage now_page = PAGE_MAIN;
uint8 cam_threshold = 128;
int16 motor_speed = 50;
int16 servo_angle = 90;

uint8 dip_mode = 0; // 拨码原始值 (0-3)
WorkMode current_mode = MODE_NORMAL;

// ==================== 硬件引脚定义 ====================
// 按键引脚 (保持不变)
#define KEY1 P20_6
#define KEY2 P20_7
#define KEY3 P11_3
#define KEY4 P11_2

// 拨码开关引脚 (你的实际连接)
#define DIP_PIN_HIGH P33_11 // 高位 (第1位)
#define DIP_PIN_LOW P33_12  // 低位 (第0位)

// ==================== 初始化函数 ====================
/**
 * @brief 初始化按键和拨码开关引脚
 * @note  全部配置为上拉输入 (GPI, 1, 2)
 */
void key_init_all(void)
{
    // 初始化按键
    gpio_init(KEY1, GPI, 1, 2);
    gpio_init(KEY2, GPI, 1, 2);
    gpio_init(KEY3, GPI, 1, 2);
    gpio_init(KEY4, GPI, 1, 2);

    // 初始化拨码开关 (上拉输入)
    gpio_init(DIP_PIN_HIGH, GPI, 1, 2);
    gpio_init(DIP_PIN_LOW, GPI, 1, 2);
}
/**
 * @brief  扫描2位拨码开关状态
 * @return 组合后的模式值 (0-3)
 * @note   拨到ON时为低电平(0)，OFF为高电平(1)
 */
uint8 key_scan(void)
{
    static uint16 key_cnt = 0;
    uint8 key_val = 0;
    // 检测按键电平（低电平有效）
    if (gpio_get_level(P20_6) == 0)
        key_val = 1;
    else if (gpio_get_level(P20_7) == 0)
        key_val = 2;
    else if (gpio_get_level(P11_2) == 0)
        key_val = 3;
    else if (gpio_get_level(P11_3) == 0)
        key_val = 4;
    else
    {
        key_cnt = 0;
        return 0;
    }
    // 消抖（延时20ms）
    key_cnt++;
    if (key_cnt >= 2)
    {
        key_cnt = 0;
        return key_val;
    }
    return 0;
}
uint8 dip_scan_mode(void)
{
    uint8 mode = 0;

    // 读取低位 (P33.12) → bit0
    if (gpio_get_level(DIP_PIN_LOW) == 0)
        mode |= 0x01;
    // 读取高位 (P33.11) → bit1
    if (gpio_get_level(DIP_PIN_HIGH) == 0)
        mode |= 0x02;

    return mode;
}
/**
 * @brief  按键处理主逻辑
 * @note   KEY1/KEY2 永远负责翻页；KEY3/KEY4 功能随拨码模式变化
 */
void key_process(void)
{
    uint8 key = key_scan();
    if (key == 0)
        return;

    // 1. 页面切换 (优先级最高，任何模式都生效)
    if (key == 1) // KEY1: 下一页
    {
        now_page = (now_page + 1) % PAGE_MAX;
        tft180_clear();
        system_delay_ms(10);
        return;
    }
    if (key == 2) // KEY2: 上一页
    {
        now_page = (now_page == 0) ? (PAGE_MAX - 1) : (now_page - 1);
        tft180_clear();
        system_delay_ms(10);
        return;
    }

    // 2. 读取当前拨码模式
    dip_mode = dip_scan_mode();
    current_mode = (WorkMode)dip_mode;

    // 3. 根据模式处理 KEY3/KEY4
    switch (current_mode)
    {
    case MODE_NORMAL: // 00: 正常手动调节
        if (key == 3) // +5
        {
            if (now_page == PAGE_CAM_SET && cam_threshold < 250)
                cam_threshold += 5;
            if (now_page == PAGE_MOTOR_SET && motor_speed < 100)
                motor_speed += 5;
            if (now_page == PAGE_SERVO_SET && servo_angle < 180)
                servo_angle += 5;
        }
        if (key == 4) // -5
        {
            if (now_page == PAGE_CAM_SET && cam_threshold > 5)
                cam_threshold -= 5;
            if (now_page == PAGE_MOTOR_SET && motor_speed > 0)
                motor_speed -= 5;
            if (now_page == PAGE_SERVO_SET && servo_angle > 0)
                servo_angle -= 5;
        }
        break;

    case MODE_CAM_AUTO: // 01: 摄像头自动模式
        // 示例：锁定手动调节，只显示提示
        if (key == 3 || key == 4)
        {
            tft180_show_string(0, 100, "Auto Mode:Locked");
        }
        break;

    case MODE_SERVO_FOLLOW: // 10: 舵机云台跟随
        if (key == 3)       // 舵机+2°
        {
            if (servo_angle < 180)
                servo_angle += 2;
        }
        if (key == 4) // 舵机-2°
        {
            if (servo_angle > 0)
                servo_angle -= 2;
        }
        break;

    case MODE_EMERGENCY: // 11: 紧急停机
        // 强制停电机，屏蔽所有调节
        motor_speed = 0;
        tft180_show_string(0, 100, "EMERGENCY STOP!");
        break;

    default:
        current_mode = MODE_NORMAL;
        break;
    }
}

void show_motor_set_page(void)
{
    char str_buf[32] = {0};
    tft180_show_string(0, 0, "MOTOR SET");

    sprintf(str_buf, "Speed:%d%%", motor_speed);
    tft180_show_string(0, 8, str_buf);
    tft180_show_string(0, 16, "Range:0-100%");

    tft180_show_string(0, 24, "KEY3:+5% KEY4:-5%");
    tft180_show_string(0, 32, "KEY1:Next KEY2:Back");
}

void show_servo_set_page(void)
{
    char str_buf[32] = {0};
    tft180_show_string(0, 0, "SERVO SET");

    sprintf(str_buf, "Angle:%ddeg", servo_angle);
    tft180_show_string(0, 8, str_buf);
    tft180_show_string(0, 16, "Range:0-180°");

    tft180_show_string(0, 24, "KEY3:+5deg KEY4:-5deg");
    tft180_show_string(0, 32, "KEY1:Next KEY2:Back");
}

// 主界面显示：摄像头灰度/二值化画面 + 实时电压
void show_main_page(void)
{
    //  float volt = get_adc_voltage();
    // char str_buf[32] = {0};
    // 1. 显示摄像头画面（0,0位置，160*80，阈值0=灰度，>0=二值化）
    draw_line();

    // 3. 显示实时电压（居中，醒目）
    // sprintf(str_buf, "Vol:%.2fV", volt);
    //  tft180_show_string(0, 96, str_buf);
}

// 系统信息页面：电压+硬件参数
void show_sys_info_page(void)
{
    float volt = get_adc_voltage();
    char str_buf[32] = {0};
    // 标题
    tft180_show_string(0, 0, "SYS INFO");
    // 实时电压
    sprintf(str_buf, "Volt:%.2fV", volt);
    tft180_show_string(0, 8, str_buf);
    // 硬件参数
    // tft180_show_string(0, 16, "MCU:TC264D");
    // tft180_show_string(0, 24, "LCD:TFT180(160*128)");
    // tft180_show_string(0, 32, "CAM:MT9V03X(GRAY)");
}

// 摄像头设置页面：二值化阈值调节+实时显示
void show_cam_set_page(void)
{
    char str_buf[32] = {0};
    // 标题
    tft180_show_string(0, 0, "CAM SET");
    // 阈值显示
    sprintf(str_buf, "Threshold: %d", cam_threshold);
    tft180_show_string(0, 8, str_buf);
    // 阈值说明
    tft180_show_string(0, 16, "0=GRAY >0=BINARY");
    // 按键操作提示
    tft180_show_string(0, 24, "KEY3:+5 KEY4:-5");
    tft180_show_string(0, 32, "KEY1:Next KEY2:Back");
    // 小窗显示摄像头效果（实时预览阈值）
    tft180_show_gray_image(0, 40, mt9v03x_image[0], MT9V03X_W, MT9V03X_H, 128, 40, cam_threshold);
}

// 菜单总调度函数
void menu_show(void)
{
    while (!mt9v03x_finish_flag)
        ;
    mt9v03x_finish_flag = 0;

    switch (now_page)
    {
    case PAGE_MAIN:
        show_main_page();
        break;
    case PAGE_SYS_INFO:
        show_sys_info_page();
        break;
    case PAGE_CAM_SET:
        show_cam_set_page();
        break;
    case PAGE_MOTOR_SET:
        show_motor_set_page();
        break;
    case PAGE_SERVO_SET:
        show_servo_set_page();
        break;
    default:
        now_page = PAGE_MAIN;
        break;
    }
}
