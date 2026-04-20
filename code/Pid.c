#include "Pid.h"
// ===================== �ڲ����� =====================

static float last_error = 0.0f;    // ��һ���������ڼ���΢����
static float current_error = 0.0f; // ��ǰ���������ڼ���ת����ٶȿ���

static float integral_l = 0.0f; // �����ٶ� PI ���ƻ�����
static float integral_r = 0.0f; // �����ٶ� PI ���ƻ�����

// ===================== �ڲ����� =====================

//-------------------------------------------------------------------------------------------------------------------
// �������     �޷�����  val: ���޷���ֵ  min_val: ��Сֵ  max_val: ���ֵ
// ���ز���     float: �޷����ֵ
//-------------------------------------------------------------------------------------------------------------------
static float clamp(float val, float min_val, float max_val)
{
    if (val > max_val)
        return max_val;
    if (val < min_val)
        return min_val;
    return val;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��������������Ȩ���
// ����˵��     center_line     ÿ������x�������飨������ͷѭ���㷨�����
// ����˵��     img_height      ͼ��������
// ���ز���     float           ����������ƫ�ң�����ƫ��
//-------------------------------------------------------------------------------------------------------------------
static float calc_error(int16 *center_line, uint8 img_height)
{
    float error = 0.0f;
    float weight_sum = 0.0f;

    uint8 row_start = ROI_ROW_START;
    uint8 row_end = ROI_ROW_END;

    if (row_end >= img_height)
        row_end = img_height - 1;

    for (uint8 row = row_start; row <= row_end; row++)
    {
        // Խ������ͷ���к�Խ��Ȩ��Խ��
        float w = (float)(row - row_start + 1);
        error += ((float)center_line[row] - (float)IMG_CENTER) * w;
        weight_sum += w;
    }

    if (weight_sum == 0.0f)
        return 0.0f;

    return error / weight_sum;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     �ٶ� PI ������
// ����˵��     target          Ŀ���ٶȣ�����������/���ڣ�
// ����˵��     actual          ʵ���ٶȣ����� motor_value��
// ����˵��     integral        ������ָ��
// ���ز���     float           PI ���ռ�ձ�����
//-------------------------------------------------------------------------------------------------------------------
static float speed_pi(float target, float actual, float *integral)
{
    float err = target - actual;
    *integral += err;
    *integral = clamp(*integral, -SPEED_INTEGRAL_MAX, SPEED_INTEGRAL_MAX);

    return SPEED_KP * err + SPEED_KI * (*integral);
}

// ===================== ����ӿ�ʵ�� =====================

//-------------------------------------------------------------------------------------------------------------------
// �������     ѭ��ģ���ʼ��
// ʹ��ʾ��     line_follow_init();
// ��ע��Ϣ     �� main �е���һ�Σ����� small_driver_uart_init() ֮�����
//-------------------------------------------------------------------------------------------------------------------
void line_follow_init(void)
{
    last_error = 0.0f;
    current_error = 0.0f;
    integral_l = 0.0f;
    integral_r = 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ѭ�������ƺ��������붨ʱ���жϻ���ѭ�������� 10ms ���ڵ��ã�
// ����˵��     center_line     ����ͷ�����ÿ������x��������
// ����˵��     img_height      ͼ��������
// ʹ��ʾ��     line_follow_control(center_line, IMG_H);
//-------------------------------------------------------------------------------------------------------------------
void line_follow_control(int16 *center_line, uint8 img_height)
{
    // ---------- 1. ������� ----------
    current_error = calc_error(center_line, img_height);

    float d_error = current_error - last_error;
    last_error = current_error;

    // ---------- 2. PD ת���� ----------
    float steer = KP * current_error + KD * d_error;
    steer = clamp(steer, -(float)STEER_MAX, (float)STEER_MAX);

    // ---------- 3. ����Զ����� ----------
    float abs_error = current_error < 0 ? -current_error : current_error;
    float target_speed = (float)BASE_DUTY - abs_error * SPEED_REDUCE_K;
    if (target_speed < (float)MIN_DUTY)
        target_speed = (float)MIN_DUTY;

    // ---------- 4. �ٶ� PI�����ڴű�����ʵ���ٶȣ� ----------
    float actual_l = (float)motor_value.receive_left_speed_data;
    float actual_r = (float)motor_value.receive_right_speed_data;

    float base_l = speed_pi(target_speed, actual_l, &integral_l);
    float base_r = speed_pi(target_speed, actual_r, &integral_r);

    // ---------- 5. ���ٺϳ� ----------
    // ƫ�ң�error > 0���� ���ּ��٣����ּ���
    float out_l = base_l + steer;
    float out_r = base_r - steer;

    // ---------- 6. ����޷������� ----------
    int16 duty_l = (int16)clamp(out_l, -(float)MAX_DUTY, (float)MAX_DUTY);
    int16 duty_r = (int16)clamp(out_r, -(float)MAX_DUTY, (float)MAX_DUTY);

    small_driver_set_duty(duty_l, duty_r);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     获取当前误差（调试/复位用）
//-------------------------------------------------------------------------------------------------------------------
float line_follow_get_error(void)
{
    return current_error;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     纯速度PI控制（不依赖循迹）
// 参数说明     target_l     左电机目标速度
// 参数说明     target_r     右电机目标速度
// 使用说明     speed_control(3000, 3000);
//-------------------------------------------------------------------------------------------------------------------
void speed_control(int16 target_l, int16 target_r)
{
    float actual_l = (float)motor_value.receive_left_speed_data;
    float actual_r = (float)motor_value.receive_right_speed_data;

    float out_l = speed_pi((float)target_l, actual_l, &integral_l);
    float out_r = speed_pi((float)target_r, actual_r, &integral_r);

    int16 duty_l = (int16)clamp(out_l, -(float)MAX_DUTY, (float)MAX_DUTY);
    int16 duty_r = (int16)clamp(out_r, -(float)MAX_DUTY, (float)MAX_DUTY);

    small_driver_set_duty(duty_l, duty_r);
}
