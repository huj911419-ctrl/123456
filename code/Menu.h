#ifndef MENU_H_
#define MENU_H_

#include "zf_common_headfile.h"

typedef enum
{
    PAGE_MAIN = 0,
    PAGE_MOTOR,
    PAGE_CAM,
    PAGE_PID,
    PAGE_IMU,
    PAGE_MAX
} MenuPage;

typedef struct
{
    const char *label;
    int16 *value;
    int16 min;
    int16 max;
    int16 step;
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
extern int16 motor_dir;
extern int16 motor_enable;
extern int16 motor_run_time;
extern int16 yaw_kp;

void key_init_all(void);
void key_process(void);
void menu_show(void);
void turn_right_led_on(void);
void turn_right_led_off(void);

#endif