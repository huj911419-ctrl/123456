#ifndef CODE_PID_H_
#define CODE_PID_H_

#include "zf_common_headfile.h"

extern int16 base_speed;

/* Runtime menu variables, defined in Menu.c. */
extern int16 motor_speed;
extern int16 pid_kp;
extern int16 pid_ki;
extern int16 pid_kd;
extern int16 yaw_kp;

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

extern uint8 ra_dbg_state;
extern uint8 ra_dbg_phase;
extern uint8 ra_dbg_dir;
extern uint8 ra_dbg_ip_row;
extern uint16 ra_dbg_timer;
extern uint16 ra_dbg_hard_cnt;
extern uint8 ra_dbg_exit_good_cnt;
extern int16 ra_dbg_yaw10;

/* Steering PD. */
#define STEER_KP  ((float)pid_kp * 0.8f)
#define STEER_KD  ((float)pid_kd * 0.6f)
#define STEER_MAX 4000.0f
#define STEER_DEADZONE 1
#define STEER_SOFT_END 6
#define STEER_SLEW_MAX 250.0f

/* Speed PI. */
#define SPEED_KP 0.5f
#define SPEED_KI ((float)pid_ki * 0.25f)
#define SPEED_I_MAX 500.0f
#define SPEED_I_SEPARATION 20
#define SPEED_FF_RATIO 0.4f
#define SPEED_FACTOR_MAX 5.0f
#define SPEED_STRAIGHT_VALID_ROWS 35u
#define SPEED_STRAIGHT_ERR_MAX 6
#define SPEED_STRAIGHT_LOOKAHEAD_MAX 8
#define SPEED_STRAIGHT_TREND_MAX 8
#define SPEED_STRAIGHT_CONFIRM_FRAMES 3u
#define SPEED_STRAIGHT_BOOST_PCT 112
#define SPEED_STRAIGHT_STEER_PCT 65
#define SPEED_LOOKAHEAD_SLOW_T1 12
#define SPEED_LOOKAHEAD_SLOW_T2 38
#define SPEED_LOOKAHEAD_MIN_PCT 65
#define SPEED_TREND_SLOW_T1 10
#define SPEED_TREND_SLOW_T2 34
#define SPEED_TREND_MIN_PCT 70
#define SPEED_VISION_BAD_VALID_ROWS 18u
#define SPEED_VISION_BAD_PCT 40
#define SPEED_LINE_LOST_PCT 25
#define SPEED_RAMP_UP_STEP 35
#define SPEED_RAMP_DOWN_STEP 90

#define MAX_DUTY 5000.0f
#define ERROR_FILTER_ALPHA 0.60f

/* Keep normal-line yaw compensation disabled by default.
 * IMU is used below only as a hard-turn exit reference. */
#define YAW_COMP_ENABLE 0
#define YAW_DEADZONE 1.0f

/* RA state timing, one frame is 11 ms. */
#define RA_HARD_TIMEOUT          15u
#define RA_TIMEOUT_FRAMES        150u
#define RA_WAIT_TIMEOUT          12u
#define RA_SLOW_TIMEOUT          15u
#define RA_HARD_MIN_FRAMES       10u
#define RA_CROSS_HARD_MIN_FRAMES 24u
#define RA_EXIT_VALID_ROWS       12u
#define RA_EXIT_ERROR_MAX        18
#define RA_EXIT_CONFIRM_FRAMES   3u
#define RA_RECOVER_MIN_FRAMES    15u
#define RA_RECOVER_MAX_FRAMES    28u
#define RA_RECOVER_SPEED_PCT     65
#define RA_RECOVER_STEER_PCT     70
#define RA_RECOVER_VALID_ROWS    25u
#define RA_RECOVER_ERROR_MAX     24
#define RA_RECOVER_CONFIRM_FRAMES 4u
#define RA_STRAIGHT_FRAMES       55u
#define RULES_DONE_DELAY         136u

void line_pid_init(void);
void line_pid_control(void);
void line_pid_reset_derivative(void);

#endif /* CODE_PID_H_ */
