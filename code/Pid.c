#include "Pid.h"
#include "Menu.h"

int16 base_speed = 900;

static float s_steer_last_pos_err = 0.0f; // �ϴ�λ�������أ�
static uint8 s_steer_d_reset_flag = 0u;   // ΢�ָ�λ��־����1ʱ�´�D�����㣩

// �ٶ� PI
static float s_speed_integral_l = 0.0f;// �����ٶȻ�����
static float s_speed_integral_r = 0.0f;// �����ٶȻ�����

// ==================== �ڲ����� ====================

//-------------------------------------------------------------------------------------------------------------------
// �������     ת�� PD ����
// ����˵��     pos_err     λ�������أ�����ƫ�ң�����ƫ��
// ���ز���     float       ת�����������������������
// ��ע��Ϣ     ���������� + ΢���ֶ���λ
//-------------------------------------------------------------------------------------------------------------------
static float steer_pd_calc(int16 pos_err)
{
    // --- ����������������ֵ��Сʱֱ�ӹ��㣬�������� ---
    if (pos_err > -STEER_DEADZONE && pos_err < STEER_DEADZONE)
    {
        s_steer_last_pos_err = 0.0f; // ��������ͬ������ϴ����
        return 0.0f;
    }

    float err = (float)pos_err;

    // --- P �� ---
    float p_out = STEER_KP * err;

    // --- D �΢���ֶ���λ�����߻ָ����ʼ����D���������㣩---
    float d_out = 0.0f;
    if (s_steer_d_reset_flag == 0u)
    {
        d_out = STEER_KD * (err - s_steer_last_pos_err);
    }
    else
    {
        s_steer_d_reset_flag = 0u; // ���ĸ�λ��־������D��Ϊ0
    }

    s_steer_last_pos_err = err;

    float output = p_out + d_out;

    if (output > STEER_MAX)
        output = STEER_MAX;
    else if (output < -STEER_MAX)
        output = -STEER_MAX;

    return output;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     �ٶ� PI ���ƣ������ַ��룩
// ����˵��     target      Ŀ���ٶȣ�����������/���ڣ�
// ����˵��     actual      ʵ���ٶ�
// ����˵��     integral    ������ָ��
// ����˵��     pos_err_abs λ��������ֵ�����ڻ��ַ����жϣ�
// ���ز���     float       �ٶ� PI ���
// ��ע��Ϣ     ���ַ��룺λ�������󣨼��䣩ʱ��ͣ�����ۼӣ���ֹ����
//-------------------------------------------------------------------------------------------------------------------
static float speed_pi_calc(float target, float actual, float *integral, int16 pos_err_abs)
{
    float speed_err = target - actual;

    // --- ���ַ��룺λ���������ֵ�ڲ����������ۼ� ---
    if (pos_err_abs < SPEED_I_SEPARATION)
    {
        *integral += speed_err;

        if (*integral > SPEED_I_MAX)
            *integral = SPEED_I_MAX;
        else if (*integral < -SPEED_I_MAX)
            *integral = -SPEED_I_MAX;
    }
    // ������ֵʱ���ֱ����ϴ�ֵ��������Ҳ���ۼӣ���������ȫ����ķ�����

    float p_out = SPEED_KP * speed_err;
    float i_out = SPEED_KI * (*integral);

    return p_out + i_out;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ���ռ�ձ��޷�
//-------------------------------------------------------------------------------------------------------------------
static int16 clamp_duty(float val)
{
    if (val > MAX_DUTY)
        val = MAX_DUTY;
    else if (val < -MAX_DUTY)
        val = -MAX_DUTY;
    return (int16)val;
}

// ==================== ����ӿ� ====================

//-------------------------------------------------------------------------------------------------------------------
// �������     PID ģ���ʼ��
// ʹ��ʾ��     line_pid_init();
//-------------------------------------------------------------------------------------------------------------------
void line_pid_init(void)
{
    s_steer_last_pos_err = 0.0f;// ת��PD�ϴ��������
    s_steer_d_reset_flag = 1u; // ��ʼ��ʱ����һ�θ�λ����������D������
    s_speed_integral_l = 0.0f;// �����ٶȻ�������
    s_speed_integral_r = 0.0f;// �����ٶȻ�������
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ΢���ֶ���λ���ⲿ�������ã�
// ʹ��ʾ��     line_pid_reset_derivative();
// ��ע��Ϣ     ������״̬�л�ʱ���綪�߻ָ�����ͣ�����������ã���ֹD��ͻ��
//-------------------------------------------------------------------------------------------------------------------
void line_pid_reset_derivative(void)
{
    s_steer_d_reset_flag = 1u;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数名称     电机自动停止计时器
//-------------------------------------------------------------------------------------------------------------------
static uint32 s_motor_run_counter = 0;

void line_pid_control(void)
{
    // -------- 电机使能判断 --------
    if (motor_enable == 0)
    {
        small_driver_set_duty(0, 0);
        s_speed_integral_l = 0.0f;
        s_speed_integral_r = 0.0f;
        s_motor_run_counter = 0;
        return;
    }

    // -------- 自动停止计时 --------
    s_motor_run_counter++;
    if (s_motor_run_counter >= (uint32)motor_run_time * 1000 / 30)
    {
        motor_enable = 0;
        small_driver_set_duty(0, 0);
        s_speed_integral_l = 0.0f;
        s_speed_integral_r = 0.0f;
        s_motor_run_counter = 0;
        return;
    }

    // -------- 赛道检测 --------
  /* if (g_tf.line_lost)
    {
        small_driver_set_duty(0, 0);
        s_speed_integral_l = 0.0f; // 停车时清除积分，防止恢复时超调
        s_speed_integral_r = 0.0f;
        line_pid_reset_derivative(); // 赛道丢失时D项不参与计算
        return;
    }*/

    int16 pos_err = g_tf.error; // 位置误差（偏差）
    int16 pos_err_abs = pos_err >= 0 ? pos_err : -pos_err;

    // -------- 1. 转向 PD（位置偏差 → 转向输出）--------
    float steer = 0;//steer_pd_calc(pos_err);

    // -------- 2. 速度根据偏差目标速度 --------
    float target_speed = (float)base_speed - (float)pos_err_abs * SPEED_REDUCE_K;
    if (target_speed < MIN_SPEED)
        target_speed = MIN_SPEED;

    // -------- 3. �ٶ� PI���������ٶ���� �� ����ռ�ձȣ������ַ��룩--------
    float actual_l = (float)motor_value.receive_left_speed_data;
    float actual_r = (float)motor_value.receive_right_speed_data;

    float speed_out_l = speed_pi_calc(target_speed, actual_l, &s_speed_integral_l, pos_err_abs);
    float speed_out_r = speed_pi_calc(target_speed, actual_r, &s_speed_integral_r, pos_err_abs);

    // -------- 4. ���е��� --------
    // pos_err > 0��ƫ�ң��� ���ּ��٣����ּ���
    float out_l = speed_out_l + steer;
    float out_r = speed_out_r - steer;

    // -------- 5. �޷���� --------
    small_driver_set_duty(clamp_duty(out_l), clamp_duty(out_r));
}
