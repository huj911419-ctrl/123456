#include "Pid.h"
#include "Menu.h"

int16 base_speed = 0;

// ================= 静态变量 =================
static float s_steer_last_pos_err = 0.0f;
static float s_steer_last_raw_err = 0.0f;   // D项用：原始误差（未滤波）
static float s_prev_steer_output = 0.0f;
static uint8 s_steer_d_reset_flag = 0u;

static float s_speed_integral = 0.0f;

// 运行停止 + 直角弯状态机
static uint8 s_is_cross_turn = 0;
static uint8 s_intersection_done = 0;
static uint8 s_finish_flag = 0;
static uint16 s_finish_cnt = 0;

// 直角弯状态机
typedef enum { RA_ST_NONE, RA_ST_ACTIVE } RaState;
typedef enum { RA_PH_WAIT, RA_PH_SLOW, RA_PH_APPROACH, RA_PH_HARD } RaPhase;
static RaState s_ra_state = RA_ST_NONE;
static RaPhase s_ra_phase = RA_PH_WAIT;
static uint8 s_ra_dir = 0;
static uint16 s_ra_approach_cnt = 0;
static uint16 s_ra_timer = 0;
static uint16 s_ra_hard_cnt = 0;
static uint8 s_ra_ip_row = 0u;      /* RA期间锁定的拐点行号 */
static uint16 s_ra_phase_cnt = 0u;  /* 当前阶段持续帧数 */

// 十字路口直行导航帧计数
static uint16 s_cross_nav_cnt = 0;

// 丢线恢复
static uint8 s_line_lost_cnt = 0;

// ================================================================
// 转向 PD 控制器（P项用滤波误差，D项用原始误差差分）
// ================================================================
static float steer_pd_calc(int16 pos_err)
{
    static float s_filtered_err = 0.0f;

    // 1. 误差低通滤波（仅给 P 项用，抑制帧间噪声）
    s_filtered_err = s_filtered_err * ERROR_FILTER_ALPHA + (float)pos_err * (1.0f - ERROR_FILTER_ALPHA);
    float err = s_filtered_err;
    float err_abs = (err >= 0) ? err : -err;
    float raw_err = (float)pos_err;

    // 2. 完全死区：偏差极小，输出归零
    if (err_abs <= (float)STEER_DEADZONE)
    {
        s_steer_last_pos_err = err;
        s_steer_last_raw_err = raw_err;
        s_prev_steer_output *= 0.5f;
        return 0.0f;
    }

    // 3. P 项 + 软过渡
    float p_scale = 1.0f;
    if (err_abs < (float)STEER_SOFT_END)
    {
        float t = (err_abs - (float)STEER_DEADZONE)
                / ((float)STEER_SOFT_END - (float)STEER_DEADZONE);
        p_scale = t * t;
    }
    float p_out = STEER_KP * err * p_scale;

    // 4. D 项：用原始误差差分（保留微分的提前性）
    float d_out = 0.0f;
    if (s_steer_d_reset_flag == 0u)
    {
        d_out = STEER_KD * (raw_err - s_steer_last_raw_err);
    }
    else
    {
        s_steer_d_reset_flag = 0u;
    }
    s_steer_last_pos_err = err;
    s_steer_last_raw_err = raw_err;

    float output = p_out + d_out;

    // 5. 硬限幅
    if (output > STEER_MAX)
        output = STEER_MAX;
    else if (output < -STEER_MAX)
        output = -STEER_MAX;

    // 6. 变化率限制
    float delta = output - s_prev_steer_output;
    if (delta > STEER_SLEW_MAX)
        output = s_prev_steer_output + STEER_SLEW_MAX;
    else if (delta < -STEER_SLEW_MAX)
        output = s_prev_steer_output - STEER_SLEW_MAX;

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
// 输出限幅（含 NaN 保护）
// ================================================================
static int16 clamp_duty(float val)
{
    if (val != val)  /* NaN 检测：NaN != NaN */
        return 0;
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
// 轨道恢复检查
// ================================================================
static uint8 track_ok(void)
{
    return (g_tf.line_lost == 0u && g_tf.valid_row_count >= 15u) ? 1u : 0u;
}

// ================================================================
// 路口完成回调
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
// 直角弯状态机复位
// ================================================================
static void ra_reset(void)
{
    s_ra_state = RA_ST_NONE;
    s_ra_phase = RA_PH_WAIT;
    s_ra_dir = 0;
    s_ra_approach_cnt = 0;
    s_ra_timer = 0;
    s_ra_hard_cnt = 0;
    s_ra_ip_row = 0u;
    s_ra_phase_cnt = 0u;
}

// ================================================================
// PID初始化
// ================================================================
void line_pid_init(void)
{
    s_steer_last_pos_err = 0.0f;
    s_steer_last_raw_err = 0.0f;
    s_prev_steer_output = 0.0f;
    s_steer_d_reset_flag = 1u;
    s_speed_integral = 0.0f;
    s_is_cross_turn = 0;
    s_intersection_done = 0;
    s_finish_flag = 0;
    s_finish_cnt = 0;

    ra_reset();
    s_cross_nav_cnt = 0;
    s_line_lost_cnt = 0;
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
// RA 状态机执行结果
// ================================================================
typedef struct {
    uint8 need_pid;       /* WAIT/SLOW/APPROACH 阶段需要走正常 PID */
    uint8 should_return;  /* HARD 阶段直接输出，本帧跳过后续逻辑 */
    float speed_scale;    /* RA阶段速度缩放因子 */
} RaResult;

// ================================================================
// RA 状态机单步执行
// ================================================================
static RaResult ra_state_machine_step(void)
{
    RaResult r = { 0, 0, 1.0f };

    /* 检测到拐点 → 激活 RA */
    if ((g_ra_flag == 1 || g_ra_flag == 2) && s_ra_state == RA_ST_NONE)
    {
        s_ra_dir = (uint8)g_ra_flag;
        s_ra_state = RA_ST_ACTIVE;
        s_ra_phase = RA_PH_WAIT;
        s_ra_ip_row = g_ip_max_row;
        s_ra_phase_cnt = 0u;
        turn_right_led_on();
    }

    if (s_ra_state != RA_ST_ACTIVE)
        return r;

    s_ra_timer++;
    s_ra_phase_cnt++;

    /* 更新锁定值：只增不减 */
    if (g_ip_max_row > s_ra_ip_row)
        s_ra_ip_row = g_ip_max_row;

    /* 退出条件 */
    uint8 camera_done = (s_ra_phase == RA_PH_HARD
                         && g_tf.line_lost == 0u
                         && g_tf.valid_row_count >= 15u) ? 1u : 0u;
    uint8 hard_timeout = (s_ra_phase == RA_PH_HARD && s_ra_hard_cnt > RA_HARD_TIMEOUT) ? 1u : 0u;
    uint8 timeout = (s_ra_timer > RA_TIMEOUT_FRAMES) ? 1u : 0u;

    if (camera_done || hard_timeout || timeout)
    {
        ra_reset();
        turn_right_led_off();
        action_done();
        line_pid_reset_derivative();
        return r;
    }

    /* 阶段升级：锁定值 OR 超时，满足任一即推进 */
    if (s_ra_phase >= RA_PH_APPROACH)
    {
        if (s_ra_approach_cnt >= (uint16)ra_approach_frames)
            s_ra_phase = RA_PH_HARD;
    }
    else if ((s_ra_ip_row >= (uint8)ra_turn_row || s_ra_phase_cnt >= RA_SLOW_TIMEOUT)
             && s_ra_phase < RA_PH_APPROACH)
    {
        s_ra_phase = RA_PH_APPROACH;
        s_ra_approach_cnt = 0;
        s_ra_phase_cnt = 0u;
    }
    else if ((s_ra_ip_row >= (uint8)ra_slow_row || s_ra_phase_cnt >= RA_WAIT_TIMEOUT)
             && s_ra_phase < RA_PH_SLOW)
    {
        s_ra_phase = RA_PH_SLOW;
        s_ra_phase_cnt = 0u;
    }

    /* 执行当前阶段 */
    if (s_ra_phase == RA_PH_HARD)
    {
        s_ra_hard_cnt++;
        float outer = (float)ra_hard_outer;
        float inner = (float)ra_hard_inner;
        float out_l, out_r;
        if (s_ra_dir == 1)  { out_l = outer; out_r = inner; }
        else                { out_l = inner; out_r = outer; }
        small_driver_set_duty(clamp_duty(out_l), clamp_duty(out_r));
        r.should_return = 1u;
    }
    else if (s_ra_phase == RA_PH_APPROACH)
    {
        s_ra_approach_cnt++;
        r.speed_scale = (float)ra_slow_pct * 0.01f;
        r.need_pid = 1u;
    }
    else if (s_ra_phase == RA_PH_SLOW)
    {
        r.speed_scale = (float)ra_slow_pct * 0.01f;
        r.need_pid = 1u;
    }
    else
    {
        r.need_pid = 1u;  /* WAIT 阶段 */
    }

    return r;
}

// ================================================================
// 十字路口处理（返回 1=已处理，0=未处理）
// ================================================================
static uint8 cross_intersection_step(void)
{
    if (g_ra_flag != 3)
        return 0u;

    s_is_cross_turn = 1;
    uint8 dir = (route_step < route_len) ? (uint8)route_seq[route_step] : 0u;

    if (dir == 0)
    {
        s_cross_nav_cnt++;
        float cross_speed = (float)base_speed * 0.6f;
        small_driver_set_duty(clamp_duty(cross_speed), clamp_duty(cross_speed));
        if (s_cross_nav_cnt > 40u && track_ok())
            action_done();
    }
    else
    {
        s_ra_dir = dir;
        s_ra_state = RA_ST_ACTIVE;
        s_ra_phase = RA_PH_WAIT;
        turn_right_led_on();
    }
    return 1u;
}

// ================================================================
// 丢线恢复（返回 1=已处理，0=未处理）
// ================================================================
static uint8 line_lost_step(void)
{
    if (!g_tf.line_lost)
    {
        s_line_lost_cnt = 0;
        return 0u;
    }

    s_line_lost_cnt++;
    uint8 lost_thresh = 12u - (uint8)((uint16)base_speed / 100u);
    if (lost_thresh < 5u) lost_thresh = 5u;
    if (s_line_lost_cnt > lost_thresh)
    {
        float lost_speed = (float)base_speed * 0.5f;
        float lost_steer = s_prev_steer_output * 0.7f;
        small_driver_set_duty(clamp_duty(lost_speed + lost_steer),
                              clamp_duty(lost_speed - lost_steer));
        s_speed_integral *= 0.95f;
        return 1u;
    }
    return 0u;
}

// ================================================================
// 正常巡线 PID
// ================================================================
static void normal_pid_step(int16 pos_err, int16 pos_err_abs)
{
    /* 高速前瞻混合 */
    float mixed_err = (float)pos_err;
    if (base_speed > 300)
    {
        float la_ratio = (float)(base_speed - 300) / 500.0f;
        if (la_ratio > 0.6f) la_ratio = 0.6f;
        mixed_err = (float)pos_err * (1.0f - la_ratio)
                  + (float)g_tf.lookahead_error * la_ratio;
    }
    float steer = steer_pd_calc((int16)mixed_err);

    /* 速度自适应：error 和 trend 独立处理 */
    int16 trend_abs = g_tf.error_trend >= 0 ? g_tf.error_trend : -g_tf.error_trend;
    int16 speed_err = pos_err_abs;
    if (base_speed > 200)
    {
        float trend_factor = (float)(base_speed - 200) / 800.0f;
        if (trend_factor > 0.5f) trend_factor = 0.5f;
        speed_err = pos_err_abs + (int16)((float)trend_abs * trend_factor);
    }
    float target_speed = (float)calc_adapted_speed(base_speed, speed_err);

    float actual_l = (float)motor_value.receive_left_speed_data;
    float actual_r = (float)motor_value.receive_right_speed_data;
    float avg_actual = (actual_l + actual_r) * 0.5f;

    /* 速度前馈 + PI 反馈 */
    float ff = target_speed * SPEED_FF_RATIO;
    float speed_out = ff + speed_pi_calc(target_speed - ff, avg_actual, &s_speed_integral, pos_err_abs);

    /* 转向速度耦合（加上限） */
    float speed_factor = 1.0f + (float)base_speed * (float)steer_speed_k * 0.001f;
    if (speed_factor > SPEED_FACTOR_MAX)
        speed_factor = SPEED_FACTOR_MAX;
    steer *= speed_factor;

    small_driver_set_duty(clamp_duty(speed_out + steer), clamp_duty(speed_out - steer));
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
        ra_reset();
        s_cross_nav_cnt = 0;
        s_line_lost_cnt = 0;
        route_step = 0;
        return;
    }

    /* 收尾阶段：渐停 */
    if (s_finish_flag)
    {
        s_finish_cnt++;
        if (s_finish_cnt > FINISH_TOTAL_FRAMES)
        {
            motor_enable = 0;
            small_driver_set_duty(0, 0);
            return;
        }
        if (s_finish_cnt > FINISH_FADE_START)
        {
            float fade = 1.0f - (float)(s_finish_cnt - FINISH_FADE_START)
                       / (float)FINISH_FADE_FRAMES;
            if (fade < 0.0f) fade = 0.0f;
            float fade_speed = (float)base_speed * fade;
            small_driver_set_duty(clamp_duty(fade_speed), clamp_duty(fade_speed));
            return;
        }
    }

    int16 pos_err = g_tf.error;
    int16 pos_err_abs = pos_err >= 0 ? pos_err : -pos_err;

    /* RA 状态机 */
    RaResult ra = ra_state_machine_step();
    if (ra.should_return)
        return;

    base_speed = (int16)((int32)motor_speed * 8 * ra.speed_scale);

    /* 丢线恢复（仅非RA状态） */
    if (!ra.need_pid && line_lost_step())
        return;

    /* 十字路口（仅非RA状态） */
    if (!ra.need_pid && cross_intersection_step())
        return;

    /* 正常巡线 PID */
    normal_pid_step(pos_err, pos_err_abs);
}
