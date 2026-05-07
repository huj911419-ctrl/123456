#include "Pid.h"
#include "Menu.h"
#include "IMU.h"

int16 base_speed = 0; // �����ٶ�

// ================= ȫ�־�̬���� =================
static float s_steer_last_pos_err = 0.0f; // ��һ��λ��������D����㣩
static uint8 s_steer_d_reset_flag = 0u;   // ΢�ָ�λ��־��1ʱ�´�D����룩

static float s_speed_integral = 0.0f;

// �Զ�ֹͣ + ֱ����״̬
static uint32 s_motor_run_counter = 0;
static uint8 s_prev_ra_flag = 0; // ��һ��ֱ�����־

// ================================================================
// �ڲ�������ת�� PD ����
// ================================================================
static float steer_pd_calc(int16 pos_err)
{
    // 一阶低通滤波，抑制摄像头帧间噪声引起的高频抖动
    static float s_filtered_err = 0.0f;
    s_filtered_err = s_filtered_err * ERROR_FILTER_ALPHA + (float)pos_err * (1.0f - ERROR_FILTER_ALPHA);
    float err = s_filtered_err;

    // 死区：偏差小不输出，但保持滤波状态和 last_err（修复退出死区时 D 项突跳）
    if (err > -STEER_DEADZONE && err < STEER_DEADZONE)
    {
        s_steer_last_pos_err = err;
        return 0.0f;
    }

    float p_out = STEER_KP * err;

    float d_out = 0.0f;
    if (s_steer_d_reset_flag == 0u)
    {
        d_out = STEER_KD * (err - s_steer_last_pos_err);
    }
    else
    {
        s_steer_d_reset_flag = 0u;
    }

    s_steer_last_pos_err = err;

    float output = p_out + d_out;

    if (output > STEER_MAX)
        output = STEER_MAX;
    else if (output < -STEER_MAX)
        output = -STEER_MAX;

    return output;
}

// ================================================================
// �ڲ��������ٶ� PI ������
// ================================================================
static float speed_pi_calc(float target, float actual, float *integral, int16 pos_err_abs)
{
    float speed_err = target - actual;

    if (pos_err_abs < SPEED_I_SEPARATION)
    {
        *integral += speed_err;

        if (*integral > SPEED_I_MAX)
            *integral = SPEED_I_MAX;
        else if (*integral < -SPEED_I_MAX)
            *integral = -SPEED_I_MAX;
    }

    float p_out = SPEED_KP * speed_err;
    float i_out = SPEED_KI * (*integral);

    return p_out + i_out;
}

// ================================================================
// �ڲ�����������޷�
// ================================================================
static int16 clamp_duty(float val)
{
    if (val > MAX_DUTY)
        val = MAX_DUTY;
    else if (val < -MAX_DUTY)
        val = -MAX_DUTY;
    return (int16)val;
}

// ================================================================
// PID��ʼ��
// ================================================================
void line_pid_init(void)
{
    s_steer_last_pos_err = 0.0f;
    s_steer_d_reset_flag = 1u;
    s_speed_integral = 0.0f;
    s_prev_ra_flag = 0;
    imu_reset_yaw();
}

// ================================================================
// ��λD��
// ================================================================
void line_pid_reset_derivative(void)
{
    s_steer_d_reset_flag = 1u;
}

// ================================================================
// PID�����ƺ�����30ms���ڣ�
// ================================================================
void line_pid_control(void)
{
    if (motor_enable == 0)
    {
        small_driver_set_duty(0, 0);
        s_speed_integral = 0.0f;
        s_motor_run_counter = 0;
        return;
    }

    s_motor_run_counter++;
    if (s_motor_run_counter >= (uint32)motor_run_time * 1000 / 30)
    {
        motor_enable = 0;
        small_driver_set_duty(0, 0);
        s_speed_integral = 0.0f;
        s_motor_run_counter = 0;
        return;
    }

    int16 pos_err = g_tf.error;
    int16 pos_err_abs = pos_err >= 0 ? pos_err : -pos_err;
    base_speed = (int16)motor_speed * 8;

    // ֱ���䴦��
    if (g_ra_flag == 1)
    {
        s_prev_ra_flag = 1;
        small_driver_set_duty(0, base_speed);
        turn_right_led_on();
        return;
    }
    else if (g_ra_flag == 2)
    {
        s_prev_ra_flag = 2;
        small_driver_set_duty(base_speed, 0);
        turn_right_led_off();
        return;
    }

    if (s_prev_ra_flag != 0)
    {
        imu_reset_yaw();
        s_prev_ra_flag = 0;
        turn_right_led_off();
    }

    float steer = steer_pd_calc(pos_err);

    // Yaw����
    {
        float yaw_kp_val = (float)yaw_kp / 10.0f;
        float yaw_comp = 0.0f;
        float yaw_abs = yaw_angle >= 0 ? yaw_angle : -yaw_angle;
        if (yaw_abs > YAW_DEADZONE)
            yaw_comp = yaw_kp_val * yaw_angle;
        steer += yaw_comp;
    }

    float target_speed = base_speed;

    float actual_l = (float)motor_value.receive_left_speed_data;
    float actual_r = (float)motor_value.receive_right_speed_data;
    float avg_actual = (actual_l + actual_r) * 0.5f;

    // 单速度环：两轮平均速度闭环，避免左右PI互相对抗
    float speed_out = speed_pi_calc(target_speed, avg_actual, &s_speed_integral, pos_err_abs);

    // 速度越高需要越大转向力度
    float speed_factor = 1.0f + (float)base_speed * 0.002f;
    steer *= speed_factor;

    float out_l = speed_out + steer;
    float out_r = speed_out - steer;

    small_driver_set_duty(clamp_duty(out_l), clamp_duty(out_r));
}
