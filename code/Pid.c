#include "Pid.h"
#include "Menu.h"
#include "IMU.h"

int16 base_speed = 900;  // 基础速度

static float s_steer_last_pos_err = 0.0f;  // 上一次位置误差（用于D项计算）
static uint8 s_steer_d_reset_flag = 0u;    // 微分复位标志（1时下次D项不参与）

// 速度PI
static float s_speed_integral_l = 0.0f;// 左轮速度积分
static float s_speed_integral_r = 0.0f;// 右轮速度积分

// ================================================================
// 内部函数：转向 PD 计算
// ================================================================
static float steer_pd_calc(int16 pos_err)
{
    // --- 死区处理：误差值太小时直接返回0 ---
    if (pos_err > -STEER_DEADZONE && pos_err < STEER_DEADZONE)
    {
        s_steer_last_pos_err = 0.0f; // 清除上一次误差
        return 0.0f;
    }

    float err = (float)pos_err;

    // --- P项 ---
    float p_out = STEER_KP * err;

    // --- D项（微分项：抑制振荡）---
    float d_out = 0.0f;
    if (s_steer_d_reset_flag == 0u)
    {
        d_out = STEER_KD * (err - s_steer_last_pos_err);
    }
    else
    {
        s_steer_d_reset_flag = 0u; // 复位标志置0
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
// 内部函数：速度 PI 控制器
// ================================================================
static float speed_pi_calc(float target, float actual, float *integral, int16 pos_err_abs)
{
    float speed_err = target - actual;

    // --- 积分项：偏差较小时才积分（防止积分饱和）---
    if (pos_err_abs < SPEED_I_SEPARATION)
    {
        *integral += speed_err;

        if (*integral > SPEED_I_MAX)
            *integral = SPEED_I_MAX;
        else if (*integral < -SPEED_I_MAX)
            *integral = -SPEED_I_MAX;
    }
    // 偏差大时直接保持上次的积分值，防止积分过度

    float p_out = SPEED_KP * speed_err;
    float i_out = SPEED_KI * (*integral);

    return p_out + i_out;
}

// ================================================================
// 内部函数：输出限幅
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
// 函数接口：PID初始化
// ================================================================
void line_pid_init(void)
{
    s_steer_last_pos_err = 0.0f;
    s_steer_d_reset_flag = 1u; // 初始化时先复位一次，让D项不参与
    s_speed_integral_l = 0.0f;
    s_speed_integral_r = 0.0f;
    s_prev_ra_flag = 0;
    imu_reset_yaw();
}

// ================================================================
// 函数接口：复位D项（外部调用）
// ================================================================
void line_pid_reset_derivative(void)
{
    s_steer_d_reset_flag = 1u;
}

// ================================================================
// 内部变量：自动停止计时 + 直角弯状态跟踪
// ================================================================
static uint32 s_motor_run_counter = 0;
static uint8 s_prev_ra_flag = 0;  // 上一次直角弯标志（用于检测退出）

// ================================================================
// 函数接口：PID控制主函数（在定时器中调用，30ms周期）
// ================================================================
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

    int16 pos_err = g_tf.error; // 位置误差（偏差）
    int16 pos_err_abs = pos_err >= 0 ? pos_err : -pos_err;

    // -------- 0. 直角弯处理 --------
    if (g_ra_flag == 1)
    {
        s_prev_ra_flag = 1;
        small_driver_set_duty(0, 500);
        turn_right_led_on();
        return;
    }
    else if (g_ra_flag == 2)
    {
        s_prev_ra_flag = 2;
        small_driver_set_duty(500, 0);
        turn_right_led_off();
        return;
    }

    if (s_prev_ra_flag != 0)
    {
        imu_reset_yaw();
        s_prev_ra_flag = 0;
        turn_right_led_off();
    }

    // -------- 1. 转向 PD（位置偏差 → 转向输出）+ Yaw 补偿 --------
    float steer = steer_pd_calc(pos_err);

    // Yaw 补偿：当车辆偏航时施加反向修正
    {
        float yaw_kp_val = (float)yaw_kp / 10.0f;  // 菜单值转实际值
        float yaw_comp = 0.0f;
        float yaw_abs = yaw_angle >= 0 ? yaw_angle : -yaw_angle;
        if (yaw_abs > YAW_DEADZONE)
            yaw_comp = yaw_kp_val * yaw_angle;
        steer += yaw_comp;
    }

    // -------- 2. 速度固定 --------
    float target_speed = base_speed;

    // -------- 3. 速度 PI 控制器（目标速度 vs 实际速度）--------
    float actual_l = (float)motor_value.receive_left_speed_data;
    float actual_r = (float)motor_value.receive_right_speed_data;

    float speed_out_l = speed_pi_calc(target_speed, actual_l, &s_speed_integral_l, pos_err_abs);
    float speed_out_r = speed_pi_calc(target_speed, actual_r, &s_speed_integral_r, pos_err_abs);

    // -------- 4. 合成输出 --------
    float out_l = speed_out_l + steer;
    float out_r = speed_out_r - steer;

    // -------- 5. 输出限幅 --------
    small_driver_set_duty(clamp_duty(out_l), clamp_duty(out_r));
}