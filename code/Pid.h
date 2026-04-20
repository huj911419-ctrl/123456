#ifndef CODE_PID_H_
#define CODE_PID_H_

#include "zf_common_headfile.h"
// ===================== ���������� =====================

#define IMG_WIDTH 188              // ����ͷͼ�����
#define IMG_CENTER (IMG_WIDTH / 2) // ͼ�������� = 94

#define ROI_ROW_START 40 // ����Ȥ������ʼ�У�������β��
#define ROI_ROW_END 55   // ����Ȥ��������У�������ͷ��

#define BASE_DUTY 3000 // ֱ������ռ�ձȣ���Χ 0~10000��
#define MIN_DUTY 1000  // ������ռ�ձ�
#define MAX_DUTY 10000 // ���ռ�ձ��޷�

#define STEER_MAX 4000 // ת��������ֵ


//PD���ƣ�Ѳ��ת����Ʋ�������Ҫ����ʵ��������ԣ�
#define KP 55.0f // ת�����ϵ��
#define KD 20.0f // ת��΢��ϵ��
//PI���ƣ��ٶȿ��Ʋ�������Ҫ����ʵ��������ԣ�
#define SPEED_KP 1.5f              // �ٶȻ�����
#define SPEED_KI 0.5f              // �ٶȻ�����
#define SPEED_INTEGRAL_MAX 2000.0f // �����޷�
// ����Զ����ٲ���
#define SPEED_REDUCE_K 8.0f // �������ϵ�������Խ���ٶ�Խ�ͣ�

// ===================== ����ӿ� =====================

void line_follow_init(void);
void line_follow_control(int16 *center_line, uint8 img_height);

// 纯速度PI控制（不依赖循迹）
void speed_control(int16 target_l, int16 target_r);

// 获取当前误差（调试/复位用）
float line_follow_get_error(void);
#endif /* CODE_PID_H_ */    
