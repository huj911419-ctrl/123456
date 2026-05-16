#ifndef MENU_H_
#define MENU_H_

#include "zf_common_headfile.h"

typedef enum
{
    PAGE_MAIN = 0,
    PAGE_PID,
    PAGE_SPEED,
    PAGE_RA,
    PAGE_ROUTE,
    PAGE_MAX
} MenuPage;

typedef struct
{
    const char *label;
    int16 *value;
    int16 min;
    int16 max;
    int16 step;
    const char *desc;  // 参数说明
} MenuItem;

typedef struct
{
    const char *title;
    MenuItem *items;
    uint8 item_count;
    void (*draw)(void);
} MenuPageDef;

extern MenuPage now_page;
extern uint8 menu_cursor;
extern int16 motor_speed;
extern int16 motor_enable;
extern int16 route_seq[20];
extern int16 route_len;
extern int16 route_step;
extern int16 detect_count;
extern int16 cam_exposure;
extern int16 threshold_bias;

// 速度自适应参数
extern int16 sp_err_t1;
extern int16 sp_err_t2;
extern int16 sp_ratio_1;
extern int16 sp_ratio_2;
extern int16 steer_speed_k;

// 直角弯参数（在 Pid.h 中声明）

void key_init_all(void);
void key_process(void);
void menu_show(void);
void turn_right_led_on(void);
void turn_right_led_off(void);

#endif
