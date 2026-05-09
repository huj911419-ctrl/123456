#include "Pid.h"
#include "Menu.h"
#include "IMU.h"

int16 base_speed = 0;

// ================= 静态变量 =================
static float s_steer_last_pos_err = 0.0f;
static float s_prev_steer_output = 0.0f;  // 转向输出变化率限制的记忆值
static uint8 s_steer_d_reset_flag = 0u;

static float s_speed_integral = 0.0f;

// 运行停止 + 直角弯状态机
static uint8 s_prev_ra_flag = 0;
static uint8 s_is_cross_turn = 0;      // 当前转弯由十字路口路线触发
static uint8 s_intersection_done = 0;  // 已完成路口数
static uint8 s_finish_flag = 0;        // 收尾阶段标志
static uint16 s_finish_cnt = 0;        // 收尾帧计数

// 直角弯平滑过渡状态机
typedef enum {
    RA_STATE_NONE = 0,
    RA_STATE_ENTERING,
    RA_STATE_TURNING,
    RA_STATE_EXITING,
} RaState_e;

static RaState_e s_ra_state = RA_STATE_NONE;
static uint8 s_ra_dir = 0;             // 当前直角方向 (1=右转 2=左转)
static uint16 s_ra_enter_cnt = 0;
static uint16 s_ra_turn_cnt = 0;       // TURNING 阶段帧计数器
static uint16 s_ra_exit_cnt = 0;

// 十字路口直行导航帧计数
static uint16 s_cross_nav_cnt = 0;

// 丢线恢复
static uint8 s_line_lost_cnt = 0;

// ================================================================
// 转向 PD 控制器（带软死区 + 变化率限制 + 输出滤波）
// ================================================================
static float steer_pd_calc(int16 pos_err)
{
    static float s_filtered_err = 0.0f;

    // 1. 误差低通滤波（抑制摄像头帧间噪声）
    s_filtered_err = s_filtered_err * ERROR_FILTER_ALPHA + (float)pos_err * (1.0f - ERROR_FILTER_ALPHA);
    float err = s_filtered_err;
    float err_abs = (err >= 0) ? err : -err;

    // 2. 完全死区：偏差极小，输出归零
    if (err_abs <= (float)STEER_DEADZONE)
    {
        s_steer_last_pos_err = err;
        s_prev_steer_output *= 0.5f;
        return 0.0f;
    }

    // 3. P 项 + 软过渡：死区～软结束之间用二次曲线，消除硬死区的"突然转向"感
    float p_scale = 1.0f;
    if (err_abs < (float)STEER_SOFT_END)
    {
        float t = (err_abs - (float)STEER_DEADZONE)
                / ((float)STEER_SOFT_END - (float)STEER_DEADZONE);
        p_scale = t * t;
    }
    float p_out = STEER_KP * err * p_scale;

    // 4. D 项（微分）
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

    // 5. 硬限幅
    if (output > STEER_MAX)
        output = STEER_MAX;
    else if (output < -STEER_MAX)
        output = -STEER_MAX;

    // 6. 变化率限制：每帧最多变化 STEER_SLEW_MAX，防止电机指令突变
    float delta = output - s_prev_steer_output;
    if (delta > STEER_SLEW_MAX)
        output = s_prev_steer_output + STEER_SLEW_MAX;
    else if (delta < -STEER_SLEW_MAX)
        output = s_prev_steer_output - STEER_SLEW_MAX;

    // 7. 输出低通滤波：让转向更平滑
    output = s_prev_steer_output * STEER_OUTPUT_FILTER + output * (1.0f - STEER_OUTPUT_FILTER);

    s_prev_steer_output = output;
    return output;
}

// ================================================================
// 速度 PI 控制器
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
// 输出限幅
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
// 速度自适应：根据偏差大小在直道/弯道速度之间线性插值
// ================================================================
static int16 calc_adapted_speed(int16 base, int16 pos_err_abs)
{
    int16 t1 = sp_err_t1;
    int16 t2 = sp_err_t2;
    int16 r1 = sp_ratio_1;
    int16 r2 = sp_ratio_2;

    if (t2 <= t1)
        t2 = t1 + 1;

    if (pos_err_abs <= t1)
        return (int16)((int32)base * r1 / 100);
    else if (pos_err_abs >= t2)
        return (int16)((int32)base * r2 / 100);
    else
    {
        int32 ratio = (int32)r1 + ((int32)(r2 - r1) * (pos_err_abs - t1)) / (t2 - t1);
        return (int16)((int32)base * ratio / 100);
    }
}

// ================================================================
// 轨道恢复检查：赛道重新可见且有效行数足够
// ================================================================
static uint8 track_ok(void)
{
    return (g_tf.line_lost == 0u && g_tf.valid_row_count >= 30u) ? 1u : 0u;
}

// ================================================================
// 路口完成回调：计数+1、更新路线步进、清零 g_ra_flag、检查收尾
// ================================================================
static void action_done(void)
{
    s_intersection_done++;

    if (s_is_cross_turn)
    {
        route_step++;
        s_cross_nav_cnt = 0;
        s_is_cross_turn = 0;
    }

    g_ra_flag = 0u;

    if (detect_count > 0 && s_intersection_done >= (uint8)detect_count)
    {
        s_finish_flag = 1u;
        s_finish_cnt = 0;
    }
}

// ================================================================
// PID初始化
// ================================================================
void line_pid_init(void)
{
    s_steer_last_pos_err = 0.0f;
    s_prev_steer_output = 0.0f;
    s_steer_d_reset_flag = 1u;
    s_speed_integral = 0.0f;
    s_prev_ra_flag = 0;
    s_is_cross_turn = 0;
    s_intersection_done = 0;
    s_finish_flag = 0;
    s_finish_cnt = 0;
    s_ra_state = RA_STATE_NONE;
    s_ra_dir = 0;
    s_cross_nav_cnt = 0;
    s_line_lost_cnt = 0;
    imu_reset_yaw();
}

// ================================================================
// 复位D项
// ================================================================
void line_pid_reset_derivative(void)
{
    s_steer_d_reset_flag = 1u;
    s_prev_steer_output = 0.0f;
}

// ================================================================
// PID控制主函数（在11ms定时器中断中调用）
// ================================================================
void line_pid_control(void)
{
    if (motor_enable == 0)
    {
        small_driver_set_duty(0, 0);
        s_speed_integral = 0.0f;
        s_is_cross_turn = 0;
        s_intersection_done = 0;
        s_finish_flag = 0;
        s_finish_cnt = 0;
        s_ra_state = RA_STATE_NONE;
        s_ra_dir = 0;
        s_cross_nav_cnt = 0;
        s_line_lost_cnt = 0;
        route_step = 0;
        return;
    }

    /* ---- 收尾阶段：正常巡线3秒后停车 ---- */
    if (s_finish_flag)
    {
        s_finish_cnt++;
        if (s_finish_cnt > 273u)
        {
            motor_enable = 0;
            small_driver_set_duty(0, 0);
            return;
        }
        /* 收尾期间继续正常巡线 PID（fall through） */
    }

    int16 pos_err = g_tf.error;
    int16 pos_err_abs = pos_err >= 0 ? pos_err : -pos_err;
    base_speed = (int16)motor_speed * 8;

    /* ---- 丢线恢复 ---- */
    if (g_tf.line_lost)
    {
        s_line_lost_cnt++;
        if (s_line_lost_cnt > 10)
        {
            float lost_speed = (float)base_speed * 0.3f;
            small_driver_set_duty(clamp_duty(lost_speed), clamp_duty(lost_speed));
            s_speed_integral *= 0.9f;
            s_ra_state = RA_STATE_NONE;
            return;
        }
    }
    else
    {
        s_line_lost_cnt = 0;
    }

    /* ---- 直角预判减速 ---- */
    if (!s_finish_flag)
    {
        static uint8 s_pre_lock = 0;
        static uint8 s_pre_timeout = 0;

        if (g_ra_pre_flag && g_ra_flag == 0)
        {
            s_pre_lock = 1;
            s_pre_timeout = 0;
        }

        if (g_ra_flag != 0)
        {
            s_pre_lock = 0;
        }

        if (s_pre_lock)
        {
            base_speed = base_speed / 7;
            if (!g_ra_pre_flag)
            {
                s_pre_timeout++;
                if (s_pre_timeout > 30)
                {
                    s_pre_lock = 0;
                    s_pre_timeout = 0;
                }
            }
        }
    }

    /* ---- 十字路口 ---- */
    if (g_ra_flag == 3)
    {
        s_prev_ra_flag = 3;
        s_is_cross_turn = 1;
        s_ra_state = RA_STATE_NONE;

        uint8 dir = (route_step < route_len) ? (uint8)route_seq[route_step] : 0u;

        if (dir == 0)
        {
            /* 直行通过十字 */
            s_cross_nav_cnt++;
            float cross_speed = (float)base_speed * 0.6f;
            small_driver_set_duty(clamp_duty(cross_speed), clamp_duty(cross_speed));

            if (s_cross_nav_cnt > 40u && track_ok())
                action_done();
        }
        else
        {
            /* 十字路口按路线转弯：复用 RA 状态机，方向由 route_seq 决定 */
            s_ra_dir = dir;
            g_ra_flag = dir;  /* 借 g_ra_flag 触发 RA 状态机入口 */
        }
        if (dir == 0)
            return;
    }

    /* ---- 直角弯：g_ra_flag == 1(右转) 或 2(左转) ---- */
    if (g_ra_flag == 1 || g_ra_flag == 2)
    {
        if (s_ra_state == RA_STATE_NONE)
        {
            s_ra_state = RA_STATE_ENTERING;
            s_ra_enter_cnt = 0;
            s_ra_turn_cnt = 0;
            if (s_ra_dir == 0)
                s_ra_dir = (uint8)g_ra_flag;
        }

        if (s_ra_state == RA_STATE_ENTERING)
        {
            s_ra_enter_cnt++;
            float blend = (float)s_ra_enter_cnt / (float)ra_enter_frames;
            if (blend > 1.0f) blend = 1.0f;

            float steer = steer_pd_calc(pos_err);
            float hard_steer = (float)base_speed;
            float blended_steer = steer * (1.0f - blend) + hard_steer * blend;
            float speed_cmd = (float)base_speed * (1.0f - blend * 0.5f);

            if (s_ra_dir == 1)
            {
                small_driver_set_duty(clamp_duty(speed_cmd - blended_steer),
                                      clamp_duty(speed_cmd + blended_steer));
                turn_right_led_on();
            }
            else
            {
                small_driver_set_duty(clamp_duty(speed_cmd + blended_steer),
                                      clamp_duty(speed_cmd - blended_steer));
                turn_right_led_off();
            }

            if (s_ra_enter_cnt >= (uint16)ra_enter_frames)
            {
                s_ra_state = RA_STATE_TURNING;
                s_ra_turn_cnt = 0;
                s_prev_ra_flag = s_ra_dir;
            }
            return;
        }

        if (s_ra_state == RA_STATE_TURNING)
        {
            s_ra_turn_cnt++;
            if (s_ra_dir == 1)
            {
                small_driver_set_duty(0, base_speed);
                turn_right_led_on();
            }
            else
            {
                small_driver_set_duty(base_speed, 0);
                turn_right_led_off();
            }

            if (s_ra_turn_cnt >= (uint16)ra_turn_frames)
            {
                s_ra_state = RA_STATE_EXITING;
                s_ra_exit_cnt = 0;
                line_pid_reset_derivative();
            }
            return;
        }
    }

    /* ---- 退出直角弯：EXITING 阶段 ---- */
    if (s_ra_state == RA_STATE_EXITING)
    {
        s_ra_exit_cnt++;
        float blend = 1.0f - (float)s_ra_exit_cnt / (float)ra_exit_frames;
        if (blend < 0.0f) blend = 0.0f;

        float steer = steer_pd_calc(pos_err);
        float hard_steer = (float)base_speed;
        float blended_steer = hard_steer * blend + steer * (1.0f - blend);

        int16 la_trend = g_tf.error_trend >= 0 ? g_tf.error_trend : -g_tf.error_trend;
        int16 la_eff = pos_err_abs > la_trend ? pos_err_abs : la_trend;
        float target_speed = (float)calc_adapted_speed(base_speed, la_eff);
        float actual_l = (float)motor_value.receive_left_speed_data;
        float actual_r = (float)motor_value.receive_right_speed_data;
        float avg_actual = (actual_l + actual_r) * 0.5f;
        float speed_out = speed_pi_calc(target_speed, avg_actual, &s_speed_integral, pos_err_abs);

        float out_l = speed_out + blended_steer;
        float out_r = speed_out - blended_steer;
        small_driver_set_duty(clamp_duty(out_l), clamp_duty(out_r));

        if (s_ra_exit_cnt >= (uint16)ra_exit_frames)
        {
            s_ra_state = RA_STATE_NONE;
            if (s_prev_ra_flag != 0)
            {
                imu_reset_yaw();
                turn_right_led_off();
            }
            action_done();
            s_ra_dir = 0;
        }
        return;
    }

    /* ---- 正常巡线 PID（含收尾阶段） ---- */
    if (s_prev_ra_flag != 0)
    {
        imu_reset_yaw();
        s_prev_ra_flag = 0;
        turn_right_led_off();
    }

    float steer = steer_pd_calc(pos_err);

    /* Yaw 补偿 */
    {
        float yaw_kp_val = (float)yaw_kp / 10.0f;
        float yaw_comp = 0.0f;
        float yaw_abs = yaw_angle >= 0 ? yaw_angle : -yaw_angle;
        if (yaw_abs > YAW_DEADZONE)
            yaw_comp = yaw_kp_val * yaw_angle;
        steer += yaw_comp;
    }

    int16 trend_abs = g_tf.error_trend >= 0 ? g_tf.error_trend : -g_tf.error_trend;
    int16 effective_err = pos_err_abs > trend_abs ? pos_err_abs : trend_abs;
    float target_speed = (float)calc_adapted_speed(base_speed, effective_err);

    float actual_l = (float)motor_value.receive_left_speed_data;
    float actual_r = (float)motor_value.receive_right_speed_data;
    float avg_actual = (actual_l + actual_r) * 0.5f;

    float speed_out = speed_pi_calc(target_speed, avg_actual, &s_speed_integral, pos_err_abs);

    float speed_factor = 1.0f + (float)base_speed * (float)steer_speed_k * 0.001f;
    steer *= speed_factor;

    float out_l = speed_out + steer;
    float out_r = speed_out - steer;

    small_driver_set_duty(clamp_duty(out_l), clamp_duty(out_r));
}
