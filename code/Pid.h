#ifndef CODE_PID_H_
#define CODE_PID_H_

#include "zf_common_headfile.h"
#include "App_Config.h"

/* PID control period. Keep time-based gains and ramps normalized to 11 ms tuning. */
#define PID_PERIOD_MS      8u
#define PID_PERIOD_REF_MS  11u
#define PID_DT_SCALE       ((float)PID_PERIOD_MS / (float)PID_PERIOD_REF_MS)
#define PID_D_SCALE        ((float)PID_PERIOD_REF_MS / (float)PID_PERIOD_MS)
#define PID_MS_TO_TICKS(ms) \
    ((uint16)((((uint32)(ms)) + (uint32)PID_PERIOD_MS - 1u) / (uint32)PID_PERIOD_MS))
#define PID_SECONDS_TO_TICKS(sec) \
    ((uint32)((((uint32)(sec) * 1000u) + (uint32)PID_PERIOD_MS - 1u) / (uint32)PID_PERIOD_MS))

extern int16 base_speed;
extern int16 speed_dbg_out;
extern int16 steer_dbg_out;
extern int16 duty_dbg_left;
extern int16 duty_dbg_right;
extern uint8 speed_dbg_vq_pct;
extern uint8 speed_dbg_pre_lock;
extern int16 speed_dbg_raw;
extern int16 speed_dbg_plan;
extern uint8 speed_dbg_reason;

/* Runtime menu variables, defined in Menu.c. */
extern int16 motor_speed;
extern int16 pid_kp;
extern int16 pid_ki;
extern int16 pid_kd;
extern int16 yaw_kp;

extern int16 cascade_en;
extern int16 yaw_kd;
extern int16 yaw_damp_gain;

extern int16 ra_hard_inner;
extern int16 ra_hard_outer;
extern int16 ra_hard_yaw;
extern int16 ra_slow_row;
extern int16 ra_slow_pct;
extern int16 ra_turn_row;
extern int16 ra_approach_frames;

extern int16 sp_err_t1;
extern int16 sp_err_t2;
extern int16 sp_ratio_1;
extern int16 sp_ratio_2;
extern int16 steer_speed_k;
extern int16 steer_ff_k;

extern uint8 ra_dbg_state;
extern uint8 ra_dbg_phase;
extern uint8 ra_dbg_dir;
extern uint8 ra_dbg_ip_row;
extern uint16 ra_dbg_timer;
extern uint16 ra_dbg_hard_cnt;
extern uint8 ra_dbg_exit_good_cnt;
extern int16 ra_dbg_yaw10;
extern uint8 route_dbg_step;
extern uint8 route_dbg_total;
extern uint8 route_dbg_flag;
extern uint8 route_dbg_count;
extern uint8 route_dbg_action;
extern volatile uint8 vacuum_enable;

/* Steering PD. */
#define STEER_KP  ((float)pid_kp * 0.8f)
#define STEER_KD  ((float)pid_kd * 0.6f)
#define STEER_MAX 4000.0f
#define STEER_DEADZONE 2
#define STEER_SOFT_END 8
#define STEER_SLEW_MAX 600.0f
#define STEER_GAIN_SPEED_START 180
#define STEER_GAIN_SPEED_END 800
#define STEER_GAIN_CURVE_T1 10
#define STEER_GAIN_CURVE_T2 38
#define STEER_FAST_KP_PCT 105
#define STEER_FAST_KD_PCT 105
#define STEER_CURVE_KP_PCT 155
#define STEER_CURVE_KD_PCT 145
#define STEER_FF_SPEED_START 180
#define STEER_FF_SPEED_END 800
#define STEER_FF_MAX 600.0f
#define STEER_FF_FILTER_ALPHA 0.35f
#define STEER_DIFF_MIN_LIMIT 250.0f
#define STEER_DIFF_MAX_LIMIT 1900.0f
#define STEER_DIFF_NORMAL_PCT 95
#define STEER_DIFF_STRAIGHT_PCT 75
#define STEER_DIFF_RECOVER_PCT 120
#define STEER_STRAIGHT_KP_PCT 78
#define STEER_STRAIGHT_KD_PCT 75
#define STEER_STRAIGHT_FF_PCT 35
#define STEER_STRAIGHT_SLEW_PCT 70

/* Speed PI. */
#define SPEED_KP 0.5f
#define SPEED_KI ((float)pid_ki * 0.25f)
#define SPEED_I_MAX 500.0f
#define SPEED_I_SEPARATION 20
#define SPEED_FF_RATIO 0.55f
#define SPEED_ACCEL_FF_GAIN 0.55f
#define SPEED_ACCEL_FF_LIMIT 400.0f
#define SPEED_FACTOR_MAX 1.9f
#define SPEED_STRAIGHT_VALID_ROWS 35u
#define SPEED_STRAIGHT_ERR_MAX 8
#define SPEED_STRAIGHT_LOOKAHEAD_MAX 10
#define SPEED_STRAIGHT_TREND_MAX 10
#define SPEED_SYM_VALID_ROWS 30u
#define SPEED_SYM_ERR_MAX 12
#define SPEED_STRAIGHT_CONFIRM_FRAMES 2u
#define SPEED_STRAIGHT_BOOST_PCT 120
#define SPEED_STRAIGHT_STEER_PCT 100
#define SPEED_LOOKAHEAD_SLOW_T1 4
#define SPEED_LOOKAHEAD_SLOW_T2 28
#define SPEED_LOOKAHEAD_MIN_PCT 72
#define SPEED_TREND_SLOW_T1 4
#define SPEED_TREND_SLOW_T2 24
#define SPEED_TREND_MIN_PCT 72
#define SPEED_VISION_BAD_VALID_ROWS 18u
#define SPEED_VISION_BAD_PCT 40
#define SPEED_QUALITY_GOOD_ROWS 35u
#define SPEED_QUALITY_ROW_MIN_PCT 78
#define SPEED_COMPONENT_VALID_ROWS 30u
#define SPEED_COMPONENT_ERR_MAX 38
#define SPEED_COMPONENT_LA_MAX 50
#define SPEED_COMPONENT_TREND_MAX 50
#define SPEED_SINGLE_EDGE_VALID_ROWS 28u
#define SPEED_SINGLE_EDGE_ERR_MAX 20
#define SPEED_SINGLE_EDGE_LOOKAHEAD_MAX 32
#define SPEED_SINGLE_EDGE_TREND_MAX 32
#define SPEED_SINGLE_EDGE_BOOST_PCT 120
#define SPEED_SINGLE_EDGE_HOLD_FRAMES 10u
#define SPEED_ERR_JUMP_T1 10
#define SPEED_ERR_JUMP_T2 28
#define SPEED_ERR_JUMP_MIN_PCT 78
#define SPEED_LINE_LOST_PCT 25
#define SPEED_RAMP_UP_STEP 1150
#define SPEED_RAMP_STRAIGHT_UP_STEP 2300
#define SPEED_RAMP_DOWN_STEP 500

#define MAX_DUTY 5000.0f
#define VAC_PWM_CH ATOM0_CH0_P21_2
#define VAC_PWM_FREQ 10000u
#define VAC_PWM_DUTY 3800u
#define ERROR_FILTER_ALPHA 0.55f
#define ERROR_FILTER_STRAIGHT_ALPHA 0.78f

/* Keep normal-line yaw compensation disabled by default.
 * IMU is used below only as a hard-turn exit reference. */
#define YAW_COMP_ENABLE 0
#define YAW_DEADZONE 1.0f
#define YAW_RATE_LIMIT 200.0f  /* yaw_rate 限幅阈值（�?秒），超过此值转向输出被压住 */

/* RA state timing, one frame is PID_PERIOD_MS. */
#define RA_HARD_TIMEOUT          14u
#define RA_FAST_SPEED_START      520
#define RA_FAST_HARD_TIMEOUT     24u
#define RA_CROSS_HARD_TIMEOUT    24u
#define RA_HARD_FORCE_TIMEOUT_EXTRA 30u
#define RA_HARD_EMERGENCY_TIMEOUT_EXTRA 50u
#define RA_TIMEOUT_FRAMES        150u
#define RA_WAIT_TIMEOUT          3u
#define RA_SLOW_TIMEOUT          10u
#define RA_FAST_SLOW_TIMEOUT     10u
#define RA_LOW_SPEED_START       360
#define RA_LOW_SLOW_TIMEOUT      10u
#define RA_PRE_SLOW_PCT          70
#define RA_CROSS_WAIT_TIMEOUT    2u
#define RA_CROSS_SLOW_TIMEOUT    10u
#define RA_HARD_MIN_FRAMES       5u
#define RA_FAST_HARD_MIN_FRAMES  7u
#define RA_HARD_RAMP_FRAMES      1u
#define RA_FAST_HARD_RAMP_FRAMES 1u
#define RA_CROSS_HARD_MIN_FRAMES 7u
#define RA_EXIT_VALID_ROWS       12u
#define RA_EXIT_ERROR_MAX        18
#define RA_EXIT_CONFIRM_FRAMES   3u
#define RA_RECOVER_FIXED_FRAMES  4u
#define RA_RECOVER_SPEED_PCT     72
#define RA_RECOVER_STEER_PCT     45
#define RA_RECOVER_VALID_ROWS    20u
#define RA_RECOVER_ERROR_MAX     18
#define RA_RECOVER_LOOKAHEAD_MAX 22
#define RA_RECOVER_TREND_MAX     24
#define RA_RECOVER_CONFIRM_FRAMES 3u
#define RA_RECOVER_NEAR_DETECT_MIN_FRAMES RA_RECOVER_FIXED_FRAMES
#define RA_RECOVER_YAW_TARGET_DEG 84.0f
#define RA_RECOVER_YAW_KP         8.0f
#define RA_RECOVER_YAW_MAX        280.0f
#define RA_RECOVER_YAW_DEADZONE   3.0f
#define RA_RECOVER_SEED_STEER_PCT 5
#define RA_FAST_DIRECT_YAW_OFFSET 0.0f
#define RA_HARD_TIMEOUT_EXIT_MARGIN 0.5f
#define RA_HARD_TAPER_START_RATIO 0.78f
#define RA_HARD_TAPER_END_RATIO   0.96f
#define RA_HARD_TAPER_MIN_PCT     72
#define RA_FAST_HARD_TAPER_START_RATIO 0.82f
#define RA_FAST_HARD_TAPER_END_RATIO   0.98f
#define RA_FAST_HARD_TAPER_MIN_PCT     75
#define RA_HARD_YAW_RATE_SOFT_LIMIT 340.0f
#define RA_HARD_YAW_RATE_MIN_PCT    75
#define RA_DIRECT_TURN_ROW_OFFSET 0u
#define RA_COMPLEX_TURN_ROW_OFFSET 0u
#define RA_FAST_TURN_ROW_ADVANCE 0u
#define RA_FAST_TURN_ROW_ADVANCE_MAX 0u
#define RA_FAST_TURN_ROW_ADVANCE_SPEED_END 1400
#define RA_LOW_TURN_ROW_DELAY    0u
#define RA_LATE_APPROACH_SKIP_ROW_MARGIN 6u
#define RA_FAST_APPROACH_FRAMES  2u
#define RA_LOW_APPROACH_FRAMES   1u
#define RA_PRE_TURN_VALID_ROWS   18u
#define RA_PRE_TURN_ROW_ADVANCE  12u
#define RA_PRE_TURN_ROW_SPAN     16u
#define RA_PRE_TURN_SPEED_END    1400
#define RA_PRE_TURN_STEER_MAX    420.0f
#define RA_PRE_TURN_SLEW_MAX     90.0f
#define RA_PRE_TURN_ENABLE       1
#define RA_COMPLEX_DUTY_PCT       82
#define RA_STRAIGHT_FRAMES       55u
#define RULES_DONE_DELAY         136u

/* 直角和丢线保底�?*/
#define RA_FALLBACK_DIRECT_ENABLE 1u

#define LOST_SEARCH_ENTER_FRAMES 3u
#define LOST_SEARCH_SWITCH_FRAMES 45u
#define LOST_SEARCH_EXIT_VALID_ROWS 15u
#define LOST_SEARCH_DUTY 520.0f
#define LOST_SEARCH_ERR_DEADZONE 4

#define EDGE_BOTH  0u
#define EDGE_LEFT  1u
#define EDGE_RIGHT 2u
#define EDGE_AUTO  3u
#define SINGLE_EDGE_POST_TURN_MS 500u
#define SINGLE_EDGE_UNTIL_NEXT_TURN 0xFFFFu

extern uint8 g_post_edge_side;

/* Cascade PID: image outer + yaw_rate inner. */
#define CAS_POS_KP          0.48f
#define CAS_TREND_KD        0.72f
#define CAS_LA_K            0.60f
#define CAS_TARGET_MAX      120.0f
#define CAS_TARGET_FILTER   0.55f
#define CAS_YAW_KP_SCALE    0.10f
#define CAS_YAW_KD_SCALE    0.10f
#define CAS_DEADZONE_DPS    1.0f
#define YAW_DAMP_SCALE      0.18f

void line_pid_init(void);
void line_pid_control(void);
void line_pid_reset_derivative(void);
void start_single_edge(uint8 side, uint16 duration_ms);
uint8 route_next_flag_is(uint8 flag);

#endif /* CODE_PID_H_ */
