#ifndef MENU_H_
#define MENU_H_

#include "zf_common_headfile.h"



// ==================== 枚举类型定义 ====================
typedef enum
{
    PAGE_MAIN = 0,  // 0: 主界面
    PAGE_SYS_INFO,  // 1: 系统信息
    PAGE_CAM_SET,   // 2: 摄像头设置
    PAGE_MOTOR_SET, // 3: 电机设置
    PAGE_SERVO_SET, // 4: 舵机设置
    PAGE_IMU_INFO,  // 5: IMU信息
    PAGE_MAX        // 页面总数
} MenuPage;

// 2位拨码开关 → 4种工作模式
typedef enum
{
    MODE_NORMAL = 0,   // 00: 正常手动模式
    MODE_CAM_AUTO,     // 01: 摄像头自动模式
    MODE_SERVO_FOLLOW, // 10: 舵机云台跟随
    MODE_EMERGENCY,    // 11: 紧急停机模式
    MODE_MAX
} WorkMode;

// ==================== 全局变量声明 ====================
extern MenuPage now_page;
extern uint8 cam_threshold;
extern int16 motor_speed;
extern int16 servo_angle;
extern uint8 dip_mode;        // 拨码原始值 (0-3)
extern WorkMode current_mode; // 当前工作模式

// ==================== 函数声明 ====================
void key_init_all(void);   // 按键+拨码初始化
uint8 key_scan(void);      // 按键扫描
void key_process(void);    // 按键逻辑处理
void menu_show(void);      // 菜单显示
uint8 dip_scan_mode(void); // 拨码开关扫描

#endif /* MENU_H_ */