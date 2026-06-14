#define AUTO_TUNE_LOG_IMPLEMENTATION
#include "AutoTuneLog.h"
#include "Menu.h"
#include "Pid.h"
#include "Track_funsion.h"
#include "IMU.h"
#include "zf_device_small_driver_uart_control.h"

#define AT_HEADER_1 0xAAu
#define AT_HEADER_2 0x55u
#define AT_SUMMARY_SIZE 22u
#define AT_DUMP_PAYLOAD_SIZE (AUTO_TUNE_RECORD_SIZE * AUTO_TUNE_DUMP_PER_PKT)

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
    uint8 box_top;
    uint8 box_bottom;
    uint8 box_left;
    uint8 box_right;
    uint8 pre_detail;
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
    p[idx++] = r->box_top;
    p[idx++] = r->box_bottom;
    p[idx++] = r->box_left;
    p[idx++] = r->box_right;
    p[idx++] = r->pre_detail;
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
            g_ra_pre_slow_flag != 0u) ? 1u : 0u;
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
    r->box_top = g_inter_result.box_top;
    r->box_bottom = g_inter_result.box_bottom;
    r->box_left = g_inter_result.box_left;
    r->box_right = g_inter_result.box_right;
    r->pre_detail = (uint8)((g_ra_pre_flag ? 0x01u : 0u) |
                            (g_ra_pre_slow_flag ? 0x02u : 0u) |
                            ((g_ra_pre_dir & 0x03u) << 2));
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
