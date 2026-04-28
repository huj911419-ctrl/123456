#ifndef CODE_PID_H_
#define CODE_PID_H_

#include "zf_common_headfile.h"

extern int16 base_speed;

#define STEER_KP 50.0f
#define STEER_KD 25.0f
#define STEER_MAX 4000.0f // 转锟斤拷锟斤拷锟斤拷薹锟�

// 转锟斤拷锟斤拷锟斤拷锟斤拷位锟斤拷锟斤拷锟斤拷锟斤拷值小锟节达拷值时锟斤拷锟斤拷为锟窖讹拷锟叫ｏ拷转锟斤拷锟斤拷锟斤拷锟斤拷锟�
#define STEER_DEADZONE 2 // 锟斤拷位锟斤拷锟斤拷锟斤拷

// ==================== 锟劫讹拷 PI 锟斤拷锟斤拷 ====================

#define SPEED_KP 0.45f
#define SPEED_KI 0.6f
#define SPEED_I_MAX 3000.0f // 锟斤拷锟斤拷锟睫凤拷

// 锟斤拷锟街凤拷锟斤拷锟斤拷值锟斤拷位锟斤拷锟斤拷锟斤拷锟斤拷值锟斤拷锟斤拷锟斤拷值时锟斤拷锟截憋拷锟劫度伙拷锟街ｏ拷锟斤拷止锟斤拷锟斤拷
#define SPEED_I_SEPARATION 20 // 锟斤拷位锟斤拷锟斤拷锟斤拷

// ==================== 锟劫讹拷目锟斤拷锟斤拷锟� ====================

#define BASE_SPEED 500.0f  // 直锟斤拷目锟斤拷锟劫度ｏ拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷/锟斤拷锟节ｏ拷  2000
#define MIN_SPEED 300.0f    // 锟斤拷锟斤拷锟斤拷目锟斤拷锟劫讹拷   500
#define MAX_DUTY 1300.0f   // 锟斤拷锟秸硷拷毡锟斤拷锟斤拷锟睫凤拷    10000
#define SPEED_REDUCE_K 5.0f // 锟斤拷锟斤拷锟斤拷锟接︼拷锟斤拷锟较碉拷锟�

// ==================== 锟斤拷锟斤拷涌锟� ====================

void line_pid_init(void);
void line_pid_control(void);
void line_pid_reset_derivative(void); // 微锟斤拷锟街讹拷锟斤拷位

#endif /* CODE_PID_H_ */    
