#include "Pid.h"
#include "Menu.h"
#include "Track_funsion.h"
#include "IMU.h"

int16 base_speed = 0;

uint8 ra_dbg_state = 0u;
uint8 ra_dbg_phase = 0u;
uint8 ra_dbg_dir = 0u;
uint8 ra_dbg_ip_row = 0u;
uint16 ra_dbg_timer = 0u;
uint16 ra_dbg_hard_cnt = 0u;
uint8 ra_dbg_exit_good_cnt = 0u;
int16 ra_dbg_yaw10 = 0;

static float s_steer_last_pos_err = 0.0f;
static float s_steer_last_raw_err = 0.0f;
static float s_prev_steer_output = 0.0f;
static uint8 s_steer_d_reset_flag = 0u;
static float s_speed_integral = 0.0f;
static uint32 s_motor_run_counter = 0;
static int16 s_base_speed_ramped = 0;
static uint8 s_straight_cnt = 0u;
static uint8 s_straight_active = 0u;

typedef enum { RA_ST_NONE, RA_ST_ACTIVE } RaState;
typedef enum { RA_PH_WAIT, RA_PH_SLOW, RA_PH_APPROACH, RA_PH_HARD, RA_PH_RECOVER } RaPhase;

static RaState s_ra_state = RA_ST_NONE;
static RaPhase s_ra_phase = RA_PH_WAIT;
static uint8 s_ra_dir = 0u;
static uint8 s_ra_orig_flag = 0u;
static uint8 s_ra_ip_row = 0u;
static uint8 s_ra_straight = 0u;
static uint8 s_ra_exit_good_cnt = 0u;
static uint8 s_ra_recover_good_cnt = 0u;
static uint16 s_ra_approach_cnt = 0u;
static uint16 s_ra_timer = 0u;
static uint16 s_ra_hard_cnt = 0u;
static uint16 s_ra_recover_cnt = 0u;
static uint16 s_ra_phase_cnt = 0u;

#define ACT_STRAIGHT 0u
#define ACT_RIGHT    1u
#define ACT_LEFT     2u

typedef struct
{
    uint8 count;
    uint8 flag;
    uint8 action;
} IntersectionRule;

/* User route table: {nth time, detected type, action}. */
static const IntersectionRule user_rules[] = {
    { 1u, 4u, ACT_RIGHT },
    { 1u, 5u,  ACT_LEFT },
    {2u, 5u,  ACT_RIGHT },
    { 2u, 4u, ACT_RIGHT },
    { 3u, 4u, ACT_RIGHT },
    { 1u, 3u, ACT_LEFT },
    { 3u, 5u, ACT_LEFT },
    { 2u, 3u, ACT_LEFT },
    { 3u, 3u, ACT_STRAIGHT },
    { 4u, 5u, ACT_RIGHT },
    { 4u, 4u, ACT_STRAIGHT },
    { 5u, 4u, ACT_STRAIGHT },
};
#define USER_RULE_COUNT (sizeof(user_rules) / sizeof(user_rules[0]))

static uint8 s_inter_count[7] = {0u};
static uint8 s_rules_done = 0u;
static uint16 s_rules_done_timer = 0u;

static void ra_debug_update(void);

typedef struct
{
    uint8 need_pid;
    uint8 should_return;
    float speed_scale;
} RaResult;

static float abs_f(float v)
{
    return (v >= 0.0f) ? v : -v;
}

static int16 abs_i16(int16 v)
{
    return (v >= 0) ? v : (int16)(-v);
}

static float normalize_angle(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

static float ra_yaw_progress(void)
{
    float delta = normalize_angle(yaw_angle);

    if (s_ra_dir == 2u)
        delta = -delta;

    return (delta > 0.0f) ? delta : 0.0f;
}

static void ra_debug_update(void)
{
    float yaw_progress = 0.0f;

    if (s_ra_state == RA_ST_ACTIVE && s_ra_dir != 0u)
        yaw_progress = ra_yaw_progress();

    ra_dbg_state = (uint8)s_ra_state;
    ra_dbg_phase = (uint8)s_ra_phase;
    ra_dbg_dir = s_ra_dir;
    ra_dbg_ip_row = s_ra_ip_row;
    ra_dbg_timer = s_ra_timer;
    ra_dbg_hard_cnt = s_ra_hard_cnt;
    ra_dbg_exit_good_cnt = (s_ra_phase == RA_PH_RECOVER) ?
                           s_ra_recover_good_cnt : s_ra_exit_good_cnt;
    ra_dbg_yaw10 = (int16)(yaw_progress * 10.0f);
}

static int16 clamp_duty(float val)
{
    if (val != val) return 0;
    if (val > MAX_DUTY) val = MAX_DUTY;
    else if (val < -MAX_DUTY) val = -MAX_DUTY;
    return (int16)val;
}

static void ra_reset(void)
{
    s_ra_state = RA_ST_NONE;
    s_ra_phase = RA_PH_WAIT;
    s_ra_dir = 0u;
    s_ra_orig_flag = 0u;
    s_ra_ip_row = 0u;
    s_ra_straight = 0u;
    s_ra_exit_good_cnt = 0u;
    s_ra_recover_good_cnt = 0u;
    s_ra_approach_cnt = 0u;
    s_ra_timer = 0u;
    s_ra_hard_cnt = 0u;
    s_ra_recover_cnt = 0u;
    s_ra_phase_cnt = 0u;
    ra_debug_update();
}

static void update_rules_done(void)
{
    uint8 all_done = 1u;

    for (uint8 i = 0u; i < USER_RULE_COUNT; i++)
    {
        uint8 flag = user_rules[i].flag;
        if (flag >= 7u || s_inter_count[flag] < user_rules[i].count)
        {
            all_done = 0u;
            break;
        }
    }

    if (all_done)
        s_rules_done = 1u;
}

void line_pid_reset_derivative(void)
{
    s_steer_d_reset_flag = 1u;
    s_prev_steer_output = 0.0f;
}

static void reset_speed_planner(void)
{
    s_base_speed_ramped = 0;
    s_straight_cnt = 0u;
    s_straight_active = 0u;
}

static void ra_finish(void)
{
    turn_right_led_off();
    g_ra_flag = 0u;
    s_speed_integral = 0.0f;
    line_pid_reset_derivative();
    update_rules_done();
    ra_reset();
}

static void ra_enter_recover(void)
{
    turn_right_led_off();
    g_ra_flag = 0u;
    s_ra_phase = RA_PH_RECOVER;
    s_ra_phase_cnt = 0u;
    s_ra_recover_cnt = 0u;
    s_ra_recover_good_cnt = 0u;
    s_speed_integral = 0.0f;
    line_pid_reset_derivative();
    reset_speed_planner();
    ra_debug_update();
}

static void ra_start(uint8 dir, uint8 orig_flag, uint8 straight)
{
    s_ra_dir = dir;
    s_ra_orig_flag = orig_flag;
    s_ra_state = RA_ST_ACTIVE;
    s_ra_phase = straight ? RA_PH_SLOW : RA_PH_WAIT;
    s_ra_ip_row = g_ip_max_row;
    s_ra_straight = straight;
    s_ra_exit_good_cnt = 0u;
    s_ra_recover_good_cnt = 0u;
    s_ra_approach_cnt = 0u;
    s_ra_timer = 0u;
    s_ra_hard_cnt = 0u;
    s_ra_recover_cnt = 0u;
    s_ra_phase_cnt = 0u;
    s_speed_integral = 0.0f;
    reset_speed_planner();

    if (!straight)
        turn_right_led_on();

    ra_debug_update();
}

static void ra_enter_hard(void)
{
    if (s_ra_phase == RA_PH_HARD)
        return;

    s_ra_phase = RA_PH_HARD;
    s_ra_phase_cnt = 0u;
    s_ra_hard_cnt = 0u;
    s_ra_exit_good_cnt = 0u;
    s_ra_recover_good_cnt = 0u;
    s_ra_recover_cnt = 0u;
    s_speed_integral = 0.0f;
    line_pid_reset_derivative();

    if (imu_ready && !imu_error)
        imu_reset_yaw();

    ra_debug_update();
}

void line_pid_init(void)
{
    s_steer_last_pos_err = 0.0f;
    s_steer_last_raw_err = 0.0f;
    s_prev_steer_output = 0.0f;
    s_steer_d_reset_flag = 1u;
    s_speed_integral = 0.0f;
    s_motor_run_counter = 0u;
    s_rules_done = 0u;
    s_rules_done_timer = 0u;
    reset_speed_planner();

    ra_reset();

    for (uint8 i = 0u; i < 7u; i++)
        s_inter_count[i] = 0u;
}

static float steer_pd_calc(int16 pos_err)
{
    static float s_filtered_err = 0.0f;

    s_filtered_err = s_filtered_err * ERROR_FILTER_ALPHA +
                     (float)pos_err * (1.0f - ERROR_FILTER_ALPHA);

    float err = s_filtered_err;
    float err_abs = abs_f(err);
    float raw_err = (float)pos_err;

    if (err_abs <= (float)STEER_DEADZONE)
    {
        s_steer_last_pos_err = err;
        s_steer_last_raw_err = raw_err;
        s_prev_steer_output *= 0.5f;
        return 0.0f;
    }

    float p_scale = 1.0f;
    if (err_abs < (float)STEER_SOFT_END)
    {
        float t = (err_abs - (float)STEER_DEADZONE) /
                  ((float)STEER_SOFT_END - (float)STEER_DEADZONE);
        p_scale = t * t;
    }

    float p_out = STEER_KP * err * p_scale;
    float d_out = 0.0f;

    if (s_steer_d_reset_flag == 0u)
        d_out = STEER_KD * (raw_err - s_steer_last_raw_err);
    else
        s_steer_d_reset_flag = 0u;

    s_steer_last_pos_err = err;
    s_steer_last_raw_err = raw_err;

    float output = p_out + d_out;

    if (output > STEER_MAX) output = STEER_MAX;
    else if (output < -STEER_MAX) output = -STEER_MAX;

    float delta = output - s_prev_steer_output;
    if (delta > STEER_SLEW_MAX)
        output = s_prev_steer_output + STEER_SLEW_MAX;
    else if (delta < -STEER_SLEW_MAX)
        output = s_prev_steer_output - STEER_SLEW_MAX;

    s_prev_steer_output = output;
    return output;
}

static float speed_pi_calc(float target, float actual, float *integral, int16 pos_err_abs)
{
    float speed_err = target - actual;

    if (pos_err_abs < SPEED_I_SEPARATION)
    {
        *integral += speed_err;

        if (*integral > SPEED_I_MAX) *integral = SPEED_I_MAX;
        else if (*integral < -SPEED_I_MAX) *integral = -SPEED_I_MAX;
    }

    return SPEED_KP * speed_err + SPEED_KI * (*integral);
}

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

    if (pos_err_abs >= t2)
        return (int16)((int32)base * r2 / 100);

    int32 ratio = (int32)r1 + ((int32)(r2 - r1) * (pos_err_abs - t1)) / (t2 - t1);
    return (int16)((int32)base * ratio / 100);
}

static uint8 straight_speed_candidate(int16 pos_err_abs)
{
    if (g_tf.line_lost != 0u)
        return 0u;
    if (g_tf.valid_row_count < SPEED_STRAIGHT_VALID_ROWS)
        return 0u;
    if (pos_err_abs > SPEED_STRAIGHT_ERR_MAX)
        return 0u;
    if (abs_i16(g_tf.lookahead_error) > SPEED_STRAIGHT_LOOKAHEAD_MAX)
        return 0u;
    if (abs_i16(g_tf.error_trend) > SPEED_STRAIGHT_TREND_MAX)
        return 0u;
    if (g_ra_pre_flag != 0u || g_ra_flag != 0u)
        return 0u;

    return 1u;
}

static int16 speed_ramp_apply(int16 target)
{
    if (target < 0)
        target = 0;

    if (s_base_speed_ramped <= 0)
    {
        s_base_speed_ramped = target;
        return s_base_speed_ramped;
    }

    if (target > s_base_speed_ramped + SPEED_RAMP_UP_STEP)
        s_base_speed_ramped += SPEED_RAMP_UP_STEP;
    else if (target < s_base_speed_ramped - SPEED_RAMP_DOWN_STEP)
        s_base_speed_ramped -= SPEED_RAMP_DOWN_STEP;
    else
        s_base_speed_ramped = target;

    return s_base_speed_ramped;
}

static int16 apply_speed_pct(int16 target, int16 pct)
{
    if (pct < 0) pct = 0;
    if (pct > 120) pct = 120;

    return (int16)((int32)target * pct / 100);
}

static int16 calc_signal_speed_pct(int16 signal, int16 t1, int16 t2, int16 min_pct)
{
    if (t2 <= t1)
        t2 = t1 + 1;

    if (signal <= t1)
        return 100;

    if (signal >= t2)
        return min_pct;

    return (int16)(100 - ((int32)(100 - min_pct) * (signal - t1)) / (t2 - t1));
}

static int16 plan_lookahead_speed(int16 target, int16 pos_err_abs)
{
    int16 la_abs = abs_i16(g_tf.lookahead_error);
    int16 trend_abs = abs_i16(g_tf.error_trend);
    int16 la_pct = calc_signal_speed_pct(la_abs,
                                         SPEED_LOOKAHEAD_SLOW_T1,
                                         SPEED_LOOKAHEAD_SLOW_T2,
                                         SPEED_LOOKAHEAD_MIN_PCT);
    int16 trend_pct = calc_signal_speed_pct(trend_abs,
                                            SPEED_TREND_SLOW_T1,
                                            SPEED_TREND_SLOW_T2,
                                            SPEED_TREND_MIN_PCT);
    int16 pct = (la_pct < trend_pct) ? la_pct : trend_pct;

    if (pos_err_abs > sp_err_t2)
        pct = (pct < sp_ratio_2) ? pct : sp_ratio_2;

    return apply_speed_pct(target, pct);
}

static int16 plan_base_speed(int16 target, int16 pos_err_abs, uint8 pre_slow_active)
{
    s_straight_active = 0u;

    if (s_ra_state != RA_ST_NONE || pre_slow_active)
    {
        s_straight_cnt = 0u;
        return speed_ramp_apply(target);
    }

    if (g_sym_component_flag && g_tf.line_lost == 0u)
    {
        s_straight_cnt = 0u;
        s_straight_active = 1u;
        return speed_ramp_apply(target);
    }

    if (g_tf.line_lost != 0u)
    {
        s_straight_cnt = 0u;
        target = (int16)((int32)target * SPEED_LINE_LOST_PCT / 100);
        return speed_ramp_apply(target);
    }

    if (g_tf.valid_row_count < SPEED_VISION_BAD_VALID_ROWS)
    {
        s_straight_cnt = 0u;
        target = (int16)((int32)target * SPEED_VISION_BAD_PCT / 100);
        return speed_ramp_apply(target);
    }

    target = plan_lookahead_speed(target, pos_err_abs);

    if (straight_speed_candidate(pos_err_abs))
    {
        if (s_straight_cnt < 255u)
            s_straight_cnt++;

        if (s_straight_cnt >= SPEED_STRAIGHT_CONFIRM_FRAMES)
        {
            s_straight_active = 1u;
            target = (int16)((int32)target * SPEED_STRAIGHT_BOOST_PCT / 100);
        }
    }
    else
    {
        s_straight_cnt = 0u;
    }

    return speed_ramp_apply(target);
}

static uint8 fallback_intersection_action(uint8 flag)
{
    if (flag == 3u)
        return ACT_LEFT;
    if (flag == 4u)
        return ACT_RIGHT;
    if (flag == 1u)
        return ACT_RIGHT;
    if (flag == 2u)
        return ACT_LEFT;

    return ACT_STRAIGHT;
}

static uint8 select_intersection_action(uint8 flag)
{
    uint8 action = fallback_intersection_action(flag);
    uint8 next_count;

    if (flag >= 7u)
        return action;

    next_count = s_inter_count[flag] + 1u;

    for (uint8 i = 0u; i < USER_RULE_COUNT; i++)
    {
        if (user_rules[i].flag == flag &&
            user_rules[i].count == next_count)
        {
            s_inter_count[flag] = next_count;
            action = user_rules[i].action;
            return action;
        }
    }

    /* Unknown extra detection: do not consume route count. */
    return action;
}

static RaResult ra_state_machine_step(int16 pos_err_abs)
{
    RaResult r = { 0u, 0u, 1.0f };

    if ((g_ra_flag == 1u || g_ra_flag == 2u) && s_ra_state == RA_ST_NONE)
    {
        ra_start((uint8)g_ra_flag, (uint8)g_ra_flag, 0u);
    }

    if ((g_ra_flag >= 3u && g_ra_flag <= 5u) && s_ra_state == RA_ST_NONE)
    {
        uint8 action = select_intersection_action((uint8)g_ra_flag);

        if (action == ACT_RIGHT || action == ACT_LEFT)
            ra_start((action == ACT_RIGHT) ? 1u : 2u, (uint8)g_ra_flag, 0u);
        else
            ra_start(0u, (uint8)g_ra_flag, 1u);
    }

    if (s_ra_state != RA_ST_ACTIVE)
    {
        ra_debug_update();
        return r;
    }

    s_ra_timer++;
    s_ra_phase_cnt++;

    if (g_ip_max_row > s_ra_ip_row)
        s_ra_ip_row = g_ip_max_row;

    if (s_ra_timer > RA_TIMEOUT_FRAMES)
    {
        ra_finish();
        return r;
    }

    if (s_ra_straight)
    {
        if (s_ra_timer >= RA_STRAIGHT_FRAMES)
        {
            ra_finish();
            return r;
        }

        r.speed_scale = (float)ra_slow_pct * 0.01f;
        r.need_pid = 1u;
        ra_debug_update();
        return r;
    }

    if (s_ra_phase == RA_PH_RECOVER)
    {
        uint8 recover_ok = (g_tf.line_lost == 0u &&
                            g_tf.valid_row_count >= RA_RECOVER_VALID_ROWS &&
                            pos_err_abs <= RA_RECOVER_ERROR_MAX) ? 1u : 0u;

        s_ra_recover_cnt++;

        if (recover_ok) s_ra_recover_good_cnt++;
        else s_ra_recover_good_cnt = 0u;

        if ((s_ra_recover_cnt >= RA_RECOVER_MIN_FRAMES &&
             s_ra_recover_good_cnt >= RA_RECOVER_CONFIRM_FRAMES) ||
            s_ra_recover_cnt >= RA_RECOVER_MAX_FRAMES)
        {
            ra_finish();
            return r;
        }

        r.speed_scale = (float)RA_RECOVER_SPEED_PCT * 0.01f;
        r.need_pid = 1u;
        ra_debug_update();
        return r;
    }

    if (s_ra_phase == RA_PH_WAIT)
    {
        if (s_ra_ip_row >= (uint8)ra_slow_row || s_ra_phase_cnt >= RA_WAIT_TIMEOUT)
        {
            s_ra_phase = RA_PH_SLOW;
            s_ra_phase_cnt = 0u;
            s_speed_integral = 0.0f;
        }
    }
    else if (s_ra_phase == RA_PH_SLOW)
    {
        if (s_ra_ip_row >= (uint8)ra_turn_row || s_ra_phase_cnt >= RA_SLOW_TIMEOUT)
        {
            s_ra_phase = RA_PH_APPROACH;
            s_ra_approach_cnt = 0u;
            s_ra_phase_cnt = 0u;
            s_speed_integral = 0.0f;
        }
    }
    else if (s_ra_phase == RA_PH_APPROACH)
    {
        s_ra_approach_cnt++;
        if (s_ra_approach_cnt >= (uint16)ra_approach_frames)
            ra_enter_hard();
    }

    if (s_ra_phase == RA_PH_HARD)
    {
        uint8 min_hard = (s_ra_orig_flag >= 3u) ?
                         RA_CROSS_HARD_MIN_FRAMES : RA_HARD_MIN_FRAMES;
        uint8 line_ok = (g_tf.line_lost == 0u &&
                         g_tf.valid_row_count >= RA_EXIT_VALID_ROWS &&
                         pos_err_abs <= RA_EXIT_ERROR_MAX) ? 1u : 0u;

        s_ra_hard_cnt++;

        if (line_ok) s_ra_exit_good_cnt++;
        else s_ra_exit_good_cnt = 0u;

        uint8 camera_done = (s_ra_hard_cnt >= min_hard &&
                             s_ra_exit_good_cnt >= RA_EXIT_CONFIRM_FRAMES) ? 1u : 0u;

        uint8 yaw_done = 0u;
        if (imu_ready && !imu_error && ra_hard_yaw > 0 &&
            s_ra_hard_cnt >= min_hard &&
            ra_yaw_progress() >= (float)ra_hard_yaw)
        {
            yaw_done = 1u;
        }

        uint8 hard_timeout = (s_ra_hard_cnt > RA_HARD_TIMEOUT) ? 1u : 0u;

        if (camera_done || yaw_done || hard_timeout)
        {
            ra_enter_recover();
            return r;
        }

        float outer = (float)ra_hard_outer;
        float inner = (float)ra_hard_inner;
        float out_l;
        float out_r;

        if (s_ra_dir == 1u)
        {
            out_l = inner;
            out_r = outer;
        }
        else
        {
            out_l = outer;
            out_r = inner;
        }

        small_driver_set_duty(clamp_duty(out_l), clamp_duty(out_r));
        r.should_return = 1u;
        ra_debug_update();
        return r;
    }

    if (s_ra_phase == RA_PH_SLOW || s_ra_phase == RA_PH_APPROACH)
        r.speed_scale = (float)ra_slow_pct * 0.01f;

    r.need_pid = 1u;
    ra_debug_update();
    return r;
}

static void normal_pid_step(int16 pos_err, int16 pos_err_abs)
{
    float mixed_err = (float)pos_err;

    if (base_speed > 300)
    {
        float la_ratio = (float)(base_speed - 300) / 500.0f;
        if (la_ratio > 0.6f) la_ratio = 0.6f;
        mixed_err = (float)pos_err * (1.0f - la_ratio) +
                    (float)g_tf.lookahead_error * la_ratio;
    }

    float steer = steer_pd_calc((int16)mixed_err);

#if YAW_COMP_ENABLE
    {
        float yaw_kp_val = (float)yaw_kp / 10.0f;
        float yaw_abs = abs_f(yaw_angle);
        if (yaw_abs > YAW_DEADZONE)
            steer += yaw_kp_val * yaw_angle;
    }
#endif

    int16 trend_abs = abs_i16(g_tf.error_trend);
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
    float ff = target_speed * SPEED_FF_RATIO;
    float speed_out = ff + speed_pi_calc(target_speed - ff,
                                         avg_actual,
                                         &s_speed_integral,
                                         pos_err_abs);

    float speed_factor = 1.0f + (float)base_speed * (float)steer_speed_k * 0.001f;
    if (speed_factor > SPEED_FACTOR_MAX)
        speed_factor = SPEED_FACTOR_MAX;

    steer *= speed_factor;

    if (s_straight_active)
        steer *= (float)SPEED_STRAIGHT_STEER_PCT * 0.01f;

    if (s_ra_state == RA_ST_ACTIVE && s_ra_phase == RA_PH_RECOVER)
        steer *= (float)RA_RECOVER_STEER_PCT * 0.01f;

    small_driver_set_duty(clamp_duty(speed_out + steer),
                          clamp_duty(speed_out - steer));
}

void line_pid_control(void)
{
    if (motor_enable == 0)
    {
        small_driver_set_duty(0, 0);
        base_speed = 0;
        s_speed_integral = 0.0f;
        s_motor_run_counter = 0u;
        reset_speed_planner();
        ra_reset();
        return;
    }

    s_motor_run_counter++;
    if (s_motor_run_counter >= (uint32)motor_run_time * 1000u / 11u)
    {
        motor_enable = 0;
        small_driver_set_duty(0, 0);
        base_speed = 0;
        s_speed_integral = 0.0f;
        s_motor_run_counter = 0u;
        reset_speed_planner();
        ra_reset();
        return;
    }

    if (s_rules_done)
    {
        s_rules_done_timer++;
        if (s_rules_done_timer >= RULES_DONE_DELAY)
        {
            motor_enable = 0;
            small_driver_set_duty(0, 0);
            base_speed = 0;
            s_speed_integral = 0.0f;
            s_motor_run_counter = 0u;
            s_rules_done = 0u;
            s_rules_done_timer = 0u;
            reset_speed_planner();
            ra_reset();
            return;
        }
    }

    int16 pos_err = g_tf.error;
    int16 pos_err_abs = abs_i16(pos_err);

    RaResult ra = ra_state_machine_step(pos_err_abs);
    if (ra.should_return)
        return;

    int16 target_base_speed = (int16)((float)motor_speed * 8.0f * ra.speed_scale);
    uint8 pre_slow_active = 0u;

    if (s_ra_state == RA_ST_NONE)
    {
        static uint8 s_pre_lock = 0u;
        static uint8 s_pre_timeout = 0u;

        if (g_ra_pre_flag && g_ra_flag == 0u)
        {
            s_pre_lock = 1u;
            s_pre_timeout = 0u;
        }

        if (g_ra_flag != 0u)
            s_pre_lock = 0u;

        if (g_sym_component_flag)
        {
            s_pre_lock = 0u;
            s_pre_timeout = 0u;
        }

        if (s_pre_lock)
        {
            pre_slow_active = 1u;
            target_base_speed = (int16)((int32)target_base_speed * ra_slow_pct / 100);

            if (!g_ra_pre_flag)
            {
                s_pre_timeout++;
                if (s_pre_timeout > 30u)
                {
                    s_pre_lock = 0u;
                    s_pre_timeout = 0u;
                }
            }
        }
    }

    base_speed = plan_base_speed(target_base_speed, pos_err_abs, pre_slow_active);

    normal_pid_step(pos_err, pos_err_abs);
}
