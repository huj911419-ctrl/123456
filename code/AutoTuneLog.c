#define AUTO_TUNE_LOG_IMPLEMENTATION
#include "AutoTuneLog.h"
#include "Menu.h"
#include "Pid.h"
#include "Track_funsion.h"
#include "IMU.h"
#include "Battery.h"
#include "zf_device_small_driver_uart_control.h"

#define AT_HEADER_1 0xAAu
#define AT_HEADER_2 0x55u
#define AT_SUMMARY_SIZE 22u
#define AT_DUMP_PAYLOAD_SIZE (AUTO_TUNE_RECORD_SIZE * AUTO_TUNE_DUMP_PER_PKT)
#define AT_TURN_MARK_HARD_ENTER      0x01u
#define AT_TURN_MARK_YAW_START       0x02u
#define AT_TURN_MARK_DUTY_START      0x04u
#define AT_TURN_MARK_RECOVER_BLOCK   0x08u
#define AT_YAW_START_RATE_DPS        140
#define AT_DUTY_START_DIFF           650

extern volatile uint32 prof_tf_us;
extern volatile uint32 prof_inter_us;

typedef struct
{
    uint16 seq;
    uint16 pid_tick;
    int16 base_speed_value;
    int16 speed_plan;
    int16 speed_out;
    int16 steer_out;
    int16 err;
    int16 lookahead;
    int16 trend;
    int16 yaw10;
    int16 ra_yaw10;
    int16 ra_hard_target10;
    int16 ra_outer_cmd;
    int16 yaw_rate_dps;
    int16 duty_left;
    int16 duty_right;
    int16 left_speed;
    int16 right_speed;
    uint16 ra_timer;
    uint16 ra_hard_cnt;
    uint16 tf_us;
    uint16 inter_us;
    uint16 battery_x10;
    uint8 valid_rows;
    uint8 ra_flag;
    uint8 ra_pre;
    uint8 ra_phase;
    uint8 ra_dir;
    uint8 ra_ip_row;
    uint8 ra_slow_row;
    uint8 ra_turn_row;
    uint8 ra_exit_reason;
    uint8 route_step;
    uint8 route_flag;
    uint8 route_action;
    uint8 post_edge_side;
    uint8 speed_reason;
    uint8 line_lost;
    uint8 inter_type;
    uint8 sym_component;
    uint8 ip_max_row;
    uint8 ip_visual_row;
    uint8 ip_ctrl_row;
    uint8 ip_ctrl_dir;
    uint8 ip_ctrl_score;
    uint8 ip_ctrl_reason;
    uint8 fast_ra_type;
    uint8 fast_front;
    uint8 fast_left;
    uint8 fast_right;
    uint8 line_takeover_ready;
    uint8 line_takeover_cnt;
    uint8 exit_boost_active;
    uint8 exit_boost_cnt;
    int16 hard_outer_dynamic;
    int16 yaw_pred;
    int16 yaw_remain;
    uint8 outer_scale;
    uint8 takeover_valid_rows;
    int16 takeover_error;
    int16 takeover_lookahead;
    int16 takeover_trend;
    uint8 box_top;
    uint8 box_bottom;
    uint8 box_left;
    uint8 box_right;
    uint8 pre_detail;
    uint8 turn_mark;
    uint8 hard_enter_ip;
    uint16 hard_enter_tick;
    uint8 pre_seen_frames;
    int16 yaw_total_progress10;        /* 总yaw progress * 10 (from ra_dbg_yaw_total_progress10) */
    int16 yaw_hard_progress10;         /* HARD yaw progress * 10 (from ra_dbg_yaw_hard_progress10) */
    uint8 visual_exit_ready;           /* visual_exit_ready flag (from ra_dbg_visual_exit_ready) */
    uint8 yaw_guard_active;            /* yaw_guard_active flag (from ra_dbg_yaw_guard_active) */
    uint8 over_turn_guard;             /* over_turn_guard flag (from ra_dbg_over_turn_guard) */
    uint8 line_takeover_speed_cap;     /* line_takeover_speed_cap counter (from ra_dbg_line_takeover_speed_cap) */
    uint8 turn_assist_active;          /* turn_assist_active flag (from ra_dbg_turn_assist_active) */
    uint8 turn_assist_weight;          /* turn_assist weight pct (from ra_dbg_turn_assist_weight) */
    int16 turn_assist_found_col;       /* turn_assist found column (from ra_dbg_turn_assist_found_col) */
    uint8 continuous_turn_mode;        /* continuous_turn_mode flag (from ra_dbg_continuous_turn_mode) */
    uint8 exit_reason_verbose;         /* verbose exit reason (from ra_dbg_exit_reason_verbose) */
    uint8 inner_min_pct;               /* inner min duty pct (from ra_dbg_inner_min_pct) */
    uint8 outer_boost_pct;             /* outer boost pct (from ra_dbg_outer_boost_pct) */
    /* V7 new fields */
    int16 turn_assist_found_row;       /* turn_assist found row (from ra_dbg_turn_assist_found_row) */
    uint8 front_short_flag;            /* front_short active flag (from ra_dbg_front_short_flag) */
    uint8 front_short_score;           /* front_short severity score (from ra_dbg_front_short_score) */
    uint8 front_short_dir;             /* front_short direction (from ra_dbg_front_short_dir) */
    uint8 front_short_confirm;         /* front_short confirm count (from ra_dbg_front_short_confirm) */
    uint8 pre_turn_active;             /* front_short pre-turn active (from ra_dbg_pre_turn_active) */
    int16 pre_turn_steer;              /* front_short pre-turn steer (from ra_dbg_pre_turn_steer) */
    uint8 recover_lost_extend;         /* recover_lost_extend counter (from ra_dbg_recover_lost_extend) */
} AutoTuneRecord;

#if AUTO_TUNE_LOG_ENABLE

static AutoTuneRecord s_at_buf[AUTO_TUNE_LOG_CAPACITY];
static volatile uint16 s_at_write = 0u;
static volatile uint16 s_at_count = 0u;
static volatile uint16 s_at_seq = 0u;
static volatile uint16 s_at_pid_tick = 0u;
static volatile uint8 s_at_div = 0u;
static volatile uint8 s_at_overflow = 0u;
static volatile uint8 s_at_capture_active = 0u;
static uint8 s_at_prev_phase = 0u;
static uint8 s_at_yaw_start_seen = 0u;
static uint8 s_at_duty_start_seen = 0u;
static uint8 s_at_hard_enter_ip = 0u;
static uint8 s_at_hard_pre_seen_frames = 0u;
static uint8 s_at_pre_seen_frames = 0u;
static uint16 s_at_hard_enter_tick = 0u;

static uint8 s_at_dump_pending = 0u;
static uint8 s_at_summary_sent = 0u;
static uint16 s_at_dump_start = 0u;
static uint16 s_at_dump_sent = 0u;
static uint16 s_at_dump_total = 0u;
static uint16 s_at_live_last_seq = 0xFFFFu;

static int16 at_clip_i16(float v)
{
    if (v > 32767.0f) return 32767;
    if (v < -32768.0f) return -32768;
    return (int16)v;
}

static int16 at_abs_i16(int16 v)
{
    if (v == (int16)-32768) return 32767;
    return (v < 0) ? (int16)(-v) : v;
}

static uint8 at_clip_u8_u16(uint16 v)
{
    return (v > 255u) ? 255u : (uint8)v;
}

static void at_put_u16(uint8 *p, uint16 *idx, uint16 v)
{
    p[(*idx)++] = (uint8)((v >> 8) & 0xFFu);
    p[(*idx)++] = (uint8)(v & 0xFFu);
}

static void at_put_i16(uint8 *p, uint16 *idx, int16 v)
{
    at_put_u16(p, idx, (uint16)v);
}

static uint16 at_clip_u16_u32(uint32 v)
{
    return (v > 65535u) ? 65535u : (uint16)v;
}

static void at_send_packet(uint8 type, const uint8 *payload, uint16 len)
{
    uint8 hdr[5];

    hdr[0] = AT_HEADER_1;
    hdr[1] = AT_HEADER_2;
    hdr[2] = type;
    hdr[3] = (uint8)((len >> 8) & 0xFFu);
    hdr[4] = (uint8)(len & 0xFFu);

    uart_write_buffer(UART_0, hdr, 5);
    if (len != 0u)
        uart_write_buffer(UART_0, (uint8 *)payload, len);
}

static void at_pack_record(uint8 *p, const AutoTuneRecord *r)
{
    uint16 idx = 0u;

    at_put_u16(p, &idx, r->seq);
    at_put_u16(p, &idx, r->pid_tick);
    at_put_i16(p, &idx, r->base_speed_value);
    at_put_i16(p, &idx, r->speed_plan);
    at_put_i16(p, &idx, r->speed_out);
    at_put_i16(p, &idx, r->steer_out);
    at_put_i16(p, &idx, r->err);
    at_put_i16(p, &idx, r->lookahead);
    at_put_i16(p, &idx, r->trend);
    at_put_i16(p, &idx, r->yaw10);
    at_put_i16(p, &idx, r->ra_yaw10);
    at_put_i16(p, &idx, r->ra_hard_target10);
    at_put_i16(p, &idx, r->ra_outer_cmd);
    at_put_i16(p, &idx, r->yaw_rate_dps);
    at_put_i16(p, &idx, r->duty_left);
    at_put_i16(p, &idx, r->duty_right);
    at_put_i16(p, &idx, r->left_speed);
    at_put_i16(p, &idx, r->right_speed);
    at_put_u16(p, &idx, r->ra_timer);
    at_put_u16(p, &idx, r->ra_hard_cnt);
    at_put_u16(p, &idx, r->tf_us);
    at_put_u16(p, &idx, r->inter_us);
    at_put_u16(p, &idx, r->battery_x10);
    p[idx++] = r->valid_rows;
    p[idx++] = r->ra_flag;
    p[idx++] = r->ra_pre;
    p[idx++] = r->ra_phase;
    p[idx++] = r->ra_dir;
    p[idx++] = r->ra_ip_row;
    p[idx++] = r->ra_slow_row;
    p[idx++] = r->ra_turn_row;
    p[idx++] = r->ra_exit_reason;
    p[idx++] = r->route_step;
    p[idx++] = r->route_flag;
    p[idx++] = r->route_action;
    p[idx++] = r->post_edge_side;
    p[idx++] = r->speed_reason;
    p[idx++] = r->line_lost;
    p[idx++] = r->inter_type;
    p[idx++] = r->sym_component;
    p[idx++] = r->ip_max_row;
    p[idx++] = r->ip_visual_row;
    p[idx++] = r->ip_ctrl_row;
    p[idx++] = r->ip_ctrl_dir;
    p[idx++] = r->ip_ctrl_score;
    p[idx++] = r->ip_ctrl_reason;
    p[idx++] = r->fast_ra_type;
    p[idx++] = r->fast_front;
    p[idx++] = r->fast_left;
    p[idx++] = r->fast_right;
    p[idx++] = r->line_takeover_ready;
    p[idx++] = r->line_takeover_cnt;
    p[idx++] = r->exit_boost_active;
    p[idx++] = r->exit_boost_cnt;
    at_put_i16(p, &idx, r->hard_outer_dynamic);
    at_put_i16(p, &idx, r->yaw_pred);
    at_put_i16(p, &idx, r->yaw_remain);
    p[idx++] = r->outer_scale;
    p[idx++] = r->takeover_valid_rows;
    at_put_i16(p, &idx, r->takeover_error);
    at_put_i16(p, &idx, r->takeover_lookahead);
    at_put_i16(p, &idx, r->takeover_trend);
    p[idx++] = r->box_top;
    p[idx++] = r->box_bottom;
    p[idx++] = r->box_left;
    p[idx++] = r->box_right;
    p[idx++] = r->pre_detail;
    at_put_u16(p, &idx, r->hard_enter_tick);
    p[idx++] = r->turn_mark;
    p[idx++] = r->hard_enter_ip;
    p[idx++] = r->pre_seen_frames;
    at_put_i16(p, &idx, r->yaw_total_progress10);
    at_put_i16(p, &idx, r->yaw_hard_progress10);
    p[idx++] = r->visual_exit_ready;
    p[idx++] = r->yaw_guard_active;
    p[idx++] = r->over_turn_guard;
    p[idx++] = r->line_takeover_speed_cap;
    p[idx++] = r->turn_assist_active;
    p[idx++] = r->turn_assist_weight;
    at_put_i16(p, &idx, r->turn_assist_found_col);
    p[idx++] = r->continuous_turn_mode;
    p[idx++] = r->exit_reason_verbose;
    p[idx++] = r->inner_min_pct;
    p[idx++] = r->outer_boost_pct;
    /* V7 new fields */
    at_put_i16(p, &idx, r->turn_assist_found_row);
    p[idx++] = r->front_short_flag;
    p[idx++] = r->front_short_score;
    p[idx++] = r->front_short_dir;
    p[idx++] = r->front_short_confirm;
    p[idx++] = r->pre_turn_active;
    at_put_i16(p, &idx, r->pre_turn_steer);
    p[idx++] = r->recover_lost_extend;
}

static void at_pack_live(uint8 *p, const AutoTuneRecord *r)
{
    uint16 idx = 0u;

    at_put_u16(p, &idx, r->seq);
    at_put_i16(p, &idx, r->speed_plan);
    at_put_i16(p, &idx, r->err);
    at_put_i16(p, &idx, r->lookahead);
    at_put_i16(p, &idx, r->yaw10);
    at_put_i16(p, &idx, r->ra_yaw10);
    at_put_i16(p, &idx, r->yaw_rate_dps);
    at_put_u16(p, &idx, r->battery_x10);
    at_put_i16(p, &idx, r->duty_left);
    at_put_i16(p, &idx, r->duty_right);
    p[idx++] = r->valid_rows;
    p[idx++] = r->ra_flag;
    p[idx++] = r->ra_phase;
    p[idx++] = r->ra_dir;
    p[idx++] = r->ra_ip_row;
    p[idx++] = r->route_step;
    p[idx++] = r->post_edge_side;
    p[idx++] = r->speed_reason;
    p[idx++] = r->line_lost;
    p[idx++] = r->inter_type;
    p[idx++] = r->ip_max_row;
}

static void at_pack_summary(uint8 *p)
{
    uint16 idx = 0u;

    p[idx++] = AUTO_TUNE_VERSION;
    p[idx++] = AUTO_TUNE_RECORD_SIZE;
    at_put_u16(p, &idx, s_at_dump_total);
    at_put_u16(p, &idx, (uint16)s_at_overflow);
    at_put_u16(p, &idx, (uint16)PID_PERIOD_MS);
    at_put_u16(p, &idx, (uint16)(PID_PERIOD_MS * AUTO_TUNE_LOG_PID_DIV));
    at_put_i16(p, &idx, motor_speed);
    at_put_i16(p, &idx, pid_kp);
    at_put_i16(p, &idx, pid_kd);
    at_put_i16(p, &idx, ra_turn_row);
    at_put_i16(p, &idx, ra_hard_outer);
    at_put_i16(p, &idx, ra_hard_yaw);
}

static uint16 at_next_index(uint16 idx)
{
    idx++;
    if (idx >= (uint16)AUTO_TUNE_LOG_CAPACITY)
        idx = 0u;
    return idx;
}

static void at_reset_capture(void)
{
    s_at_write = 0u;
    s_at_count = 0u;
    s_at_seq = 0u;
    s_at_pid_tick = 0u;
    s_at_div = 0u;
    s_at_overflow = 0u;
    s_at_dump_pending = 0u;
    s_at_summary_sent = 0u;
    s_at_dump_start = 0u;
    s_at_dump_sent = 0u;
    s_at_dump_total = 0u;
    s_at_live_last_seq = 0xFFFFu;
    s_at_prev_phase = 0u;
    s_at_yaw_start_seen = 0u;
    s_at_duty_start_seen = 0u;
    s_at_hard_enter_ip = 0u;
    s_at_hard_pre_seen_frames = 0u;
    s_at_pre_seen_frames = 0u;
    s_at_hard_enter_tick = 0u;
}

uint8 auto_tune_log_busy(void)
{
    return (s_at_capture_active != 0u || s_at_dump_pending != 0u) ? 1u : 0u;
}

static uint8 at_force_pid_sample(void)
{
    return (ra_dbg_state != 0u ||
            g_ra_flag != 0u ||
            g_ra_pre_flag != 0u ||
            g_ra_pre_slow_flag != 0u ||
            g_tf.line_lost != 0u) ? 1u : 0u;
}

static uint8 at_record_div(void)
{
    uint8 div = (uint8)AUTO_TUNE_LOG_PID_DIV;

    if (at_force_pid_sample())
        return 1u;

    return (div < 1u) ? 1u : div;
}

void auto_tune_log_pid_tick(void)
{
    AutoTuneRecord *r;
    uint8 div;
    uint8 turn_mark;
    uint8 pre_seen;
    int16 progress_rate;
    int16 duty_diff;

    if (motor_enable == 0)
        return;

    if (s_at_capture_active == 0u)
    {
        at_reset_capture();
        s_at_capture_active = 1u;
    }

    s_at_pid_tick++;
    div = at_record_div();

    if (div > 1u)
    {
        s_at_div++;
        if (s_at_div < div)
            return;
        s_at_div = 0u;
    }
    else
    {
        s_at_div = 0u;
    }

    r = &s_at_buf[s_at_write];
    r->seq = s_at_seq++;
    r->pid_tick = s_at_pid_tick;
    r->base_speed_value = base_speed;
    r->speed_plan = speed_dbg_plan;
    r->speed_out = speed_dbg_out;
    r->steer_out = steer_dbg_out;
    r->err = g_tf.error;
    r->lookahead = g_tf.lookahead_error;
    r->trend = g_tf.error_trend;
    r->yaw10 = at_clip_i16(yaw_angle * 10.0f);
    r->ra_yaw10 = ra_dbg_yaw10;
    r->ra_hard_target10 = ra_dbg_hard_target10;
    r->ra_outer_cmd = ra_dbg_outer_cmd;
    r->yaw_rate_dps = at_clip_i16(yaw_rate);
    r->duty_left = duty_dbg_left;
    r->duty_right = duty_dbg_right;
    r->left_speed = motor_value.receive_left_speed_data;
    r->right_speed = motor_value.receive_right_speed_data;
    r->ra_timer = ra_dbg_timer;
    r->ra_hard_cnt = ra_dbg_hard_cnt;
    r->tf_us = at_clip_u16_u32(prof_tf_us);
    r->inter_us = at_clip_u16_u32(prof_inter_us);
    r->battery_x10 = battery_get_voltage_x10();
    r->valid_rows = at_clip_u8_u16(g_tf.valid_row_count);
    r->ra_flag = g_ra_flag;
    r->ra_pre = (uint8)((g_ra_pre_flag || g_ra_pre_slow_flag) ? 1u : 0u);
    r->ra_phase = ra_dbg_phase;
    r->ra_dir = ra_dbg_dir;
    r->ra_ip_row = ra_dbg_ip_row;
    r->ra_slow_row = ra_dbg_slow_row;
    r->ra_turn_row = ra_dbg_turn_row;
    r->ra_exit_reason = ra_dbg_exit_reason;
    r->route_step = route_dbg_step;
    r->route_flag = route_dbg_flag;
    r->route_action = route_dbg_action;
    r->post_edge_side = g_post_edge_side;
    r->speed_reason = speed_dbg_reason;
    r->line_lost = g_tf.line_lost;
    r->inter_type = g_inter_result.detected_type;
    r->sym_component = g_sym_component_flag;
    r->ip_max_row = g_ip_max_row;
    r->ip_visual_row = g_ip_visual_row;
    r->ip_ctrl_row = g_ip_ctrl_row;
    r->ip_ctrl_dir = g_ip_ctrl_dir;
    r->ip_ctrl_score = g_ip_ctrl_score;
    r->ip_ctrl_reason = g_ip_ctrl_reason;
    r->fast_ra_type = g_fast_ra_type;
    r->fast_front = g_fast_ra_front;
    r->fast_left = g_fast_ra_left;
    r->fast_right = g_fast_ra_right;
    r->line_takeover_ready = ra_dbg_line_takeover_ready;
    r->line_takeover_cnt = ra_dbg_line_takeover_cnt;
    r->exit_boost_active = ra_dbg_exit_boost_active;
    r->exit_boost_cnt = ra_dbg_exit_boost_cnt;
    r->hard_outer_dynamic = ra_dbg_hard_outer_dynamic;
    r->yaw_pred = ra_dbg_yaw_pred10;
    r->yaw_remain = ra_dbg_yaw_remain10;
    r->outer_scale = ra_dbg_outer_scale;
    r->takeover_valid_rows = ra_dbg_takeover_valid_rows;
    r->takeover_error = ra_dbg_takeover_error;
    r->takeover_lookahead = ra_dbg_takeover_lookahead;
    r->takeover_trend = ra_dbg_takeover_trend;
    r->box_top = g_inter_result.box_top;
    r->box_bottom = g_inter_result.box_bottom;
    r->box_left = g_inter_result.box_left;
    r->box_right = g_inter_result.box_right;
    r->pre_detail = (uint8)((g_ra_pre_flag ? 0x01u : 0u) |
                            (g_ra_pre_slow_flag ? 0x02u : 0u) |
                            ((g_ra_pre_dir & 0x03u) << 2));

    pre_seen = (uint8)((g_ra_pre_flag || g_ra_pre_slow_flag) ? 1u : 0u);
    if (r->ra_phase != 3u)
    {
        if (pre_seen != 0u)
        {
            if (s_at_pre_seen_frames < 255u)
                s_at_pre_seen_frames++;
        }
        else if (r->ra_phase == 0u && r->ra_flag == 0u)
        {
            s_at_pre_seen_frames = 0u;
        }
    }

    turn_mark = 0u;
    if (r->ra_phase == 3u && s_at_prev_phase != 3u)
    {
        turn_mark |= AT_TURN_MARK_HARD_ENTER;
        s_at_yaw_start_seen = 0u;
        s_at_duty_start_seen = 0u;
        s_at_hard_enter_ip = r->ra_ip_row;
        s_at_hard_enter_tick = r->pid_tick;
        s_at_hard_pre_seen_frames = s_at_pre_seen_frames;
        s_at_pre_seen_frames = 0u;
    }

    if (r->ra_phase == 3u)
    {
        progress_rate = r->yaw_rate_dps;
        duty_diff = 0;
        if (r->ra_dir == 1u)
        {
            progress_rate = (int16)(-progress_rate);
            duty_diff = (int16)(r->duty_right - r->duty_left);
        }
        else if (r->ra_dir == 2u)
        {
            duty_diff = (int16)(r->duty_left - r->duty_right);
        }

        if (s_at_yaw_start_seen == 0u &&
            progress_rate >= (int16)AT_YAW_START_RATE_DPS)
        {
            turn_mark |= AT_TURN_MARK_YAW_START;
            s_at_yaw_start_seen = 1u;
        }
        if (s_at_duty_start_seen == 0u &&
            duty_diff >= (int16)AT_DUTY_START_DIFF)
        {
            turn_mark |= AT_TURN_MARK_DUTY_START;
            s_at_duty_start_seen = 1u;
        }
    }
    else if (r->ra_phase == 5u &&
             (r->ra_flag != 0u ||
              pre_seen != 0u ||
              r->ip_max_row >= RA_FIXED_HARD_ROW))
    {
        turn_mark |= AT_TURN_MARK_RECOVER_BLOCK;
    }

    r->turn_mark = turn_mark;
    r->hard_enter_ip = s_at_hard_enter_ip;
    r->hard_enter_tick = s_at_hard_enter_tick;
    r->pre_seen_frames = (r->ra_phase == 3u) ?
                         s_at_hard_pre_seen_frames :
                         s_at_pre_seen_frames;

    if (r->ra_phase == 3u &&
        r->ra_hard_cnt >= 4u &&
        at_abs_i16(r->ra_outer_cmd) >= 1800 &&
        at_abs_i16(r->yaw_rate_dps) <= 80)
    {
        r->pre_detail |= 0x40u;
        if (((int32)at_abs_i16(r->left_speed) +
             (int32)at_abs_i16(r->right_speed)) <= 400)
        {
            r->pre_detail |= 0x80u;
        }
    }
    /* New improved direct turn fields */
    r->yaw_total_progress10 = ra_dbg_yaw_total_progress10;
    r->yaw_hard_progress10 = ra_dbg_yaw_hard_progress10;
    r->visual_exit_ready = ra_dbg_visual_exit_ready;
    r->yaw_guard_active = ra_dbg_yaw_guard_active;
    r->over_turn_guard = ra_dbg_over_turn_guard;
    r->line_takeover_speed_cap = ra_dbg_line_takeover_speed_cap;
    r->turn_assist_active = ra_dbg_turn_assist_active;
    r->turn_assist_weight = ra_dbg_turn_assist_weight;
    r->turn_assist_found_col = ra_dbg_turn_assist_found_col;
    r->continuous_turn_mode = ra_dbg_continuous_turn_mode;
    r->exit_reason_verbose = ra_dbg_exit_reason_verbose;
    r->inner_min_pct = ra_dbg_inner_min_pct;
    r->outer_boost_pct = ra_dbg_outer_boost_pct;
    /* V7 new fields */
    r->turn_assist_found_row = ra_dbg_turn_assist_found_row;
    r->front_short_flag = ra_dbg_front_short_flag;
    r->front_short_score = ra_dbg_front_short_score;
    r->front_short_dir = ra_dbg_front_short_dir;
    r->front_short_confirm = ra_dbg_front_short_confirm;
    r->pre_turn_active = ra_dbg_pre_turn_active;
    r->pre_turn_steer = ra_dbg_pre_turn_steer;
    r->recover_lost_extend = ra_dbg_recover_lost_extend;

    s_at_prev_phase = r->ra_phase;

    s_at_write = at_next_index(s_at_write);
    if (s_at_count < (uint16)AUTO_TUNE_LOG_CAPACITY)
        s_at_count++;
    else
        s_at_overflow = 1u;
}

static void at_live_task(void)
{
#if AUTO_TUNE_LIVE_ENABLE
    uint16 last_idx;
    uint16 last_seq;
    uint16 diff;
    uint8 payload[AUTO_TUNE_LIVE_SIZE];

    if (s_at_count == 0u)
        return;

    last_idx = (s_at_write == 0u) ?
               (uint16)(AUTO_TUNE_LOG_CAPACITY - 1u) :
               (uint16)(s_at_write - 1u);
    last_seq = s_at_buf[last_idx].seq;

    if (s_at_live_last_seq != 0xFFFFu)
    {
        diff = (uint16)(last_seq - s_at_live_last_seq);
        if (diff < (uint16)AUTO_TUNE_LIVE_DIV)
            return;
    }

    at_pack_live(payload, &s_at_buf[last_idx]);
    at_send_packet(AUTO_TUNE_TYPE_LIVE, payload, AUTO_TUNE_LIVE_SIZE);
    s_at_live_last_seq = last_seq;
#endif
}

void auto_tune_log_task(void)
{
    uint8 payload[AT_DUMP_PAYLOAD_SIZE];
    uint16 records_this_packet;
    uint16 i;
    uint16 idx;

    if (motor_enable != 0)
    {
        at_live_task();
        return;
    }

    if (s_at_capture_active != 0u)
    {
        s_at_capture_active = 0u;
        s_at_dump_total = s_at_count;
        s_at_dump_sent = 0u;
        s_at_summary_sent = 0u;
        if (s_at_count == (uint16)AUTO_TUNE_LOG_CAPACITY)
            s_at_dump_start = s_at_write;
        else
            s_at_dump_start = 0u;
        s_at_dump_pending = (s_at_dump_total != 0u) ? 1u : 0u;
    }

    if (s_at_dump_pending == 0u)
        return;

    if (s_at_summary_sent == 0u)
    {
        uint8 summary[AT_SUMMARY_SIZE];
        at_pack_summary(summary);
        at_send_packet(AUTO_TUNE_TYPE_SUMMARY, summary, AT_SUMMARY_SIZE);
        s_at_summary_sent = 1u;
        return;
    }

    if (s_at_dump_sent >= s_at_dump_total)
    {
        at_send_packet(AUTO_TUNE_TYPE_END, 0, 0u);
        s_at_dump_pending = 0u;
        s_at_count = 0u;
        s_at_write = 0u;
        return;
    }

    records_this_packet = (uint16)(s_at_dump_total - s_at_dump_sent);
    if (records_this_packet > (uint16)AUTO_TUNE_DUMP_PER_PKT)
        records_this_packet = (uint16)AUTO_TUNE_DUMP_PER_PKT;

    idx = 0u;
    for (i = 0u; i < records_this_packet; i++)
    {
        uint16 buf_idx = (uint16)(s_at_dump_start + s_at_dump_sent + i);
        while (buf_idx >= (uint16)AUTO_TUNE_LOG_CAPACITY)
            buf_idx = (uint16)(buf_idx - (uint16)AUTO_TUNE_LOG_CAPACITY);
        at_pack_record(&payload[idx], &s_at_buf[buf_idx]);
        idx = (uint16)(idx + AUTO_TUNE_RECORD_SIZE);
    }

    at_send_packet(AUTO_TUNE_TYPE_RECORD, payload, idx);
    s_at_dump_sent = (uint16)(s_at_dump_sent + records_this_packet);
}

#else

void auto_tune_log_pid_tick(void)
{
}

void auto_tune_log_task(void)
{
}

uint8 auto_tune_log_busy(void)
{
    return 0u;
}

#endif
