#ifndef TRACK_FUSION_H
#define TRACK_FUSION_H

#include "zf_common_headfile.h"
#include "App_Config.h"

extern int16 threshold_bias;

/* Image compression: 188x120 -> 94x60. */
#define COMP_W          (MT9V03X_W / 2)
#define COMP_H          (MT9V03X_H / 2)
#define COMP_CENTER     (COMP_W / 2)

#define TF_IMG_H        COMP_H
#define TF_IMG_W        COMP_W
#define TF_IMG_CENTER   COMP_CENTER

/* Track extraction parameters. */
#ifndef TF_OTSU_INTERVAL
#if RACE_MODE
#define TF_OTSU_INTERVAL    TF_OTSU_INTERVAL_RACE
#else
#define TF_OTSU_INTERVAL    TF_OTSU_INTERVAL_DEBUG
#endif
#endif
#define TF_JIDIAN_ROW       (TF_IMG_H - 4)
#define TF_JIDIAN_MIN_ROW   (TF_IMG_H - 15)
#define TF_SINGLE_EDGE_JIDIAN_MIN_ROW (TF_IMG_H - 32)
#define TF_SEARCH_END_ROW   4
#define TF_LOCAL_RANGE      10
#define TF_MAX_MISS_ROWS    5
/* 1 = 扫到分支行时跳过该行继续向上扫（保住前瞻，修T�?斜入弯冷启动�?
 * 0 = 旧行为：扫到分支行立即停扫（valid_rows会塌�?�?*/
#define TF_BRANCH_ROW_KEEP_SCAN 1
#define TF_MIN_TRACK_WIDTH  4
#define TF_MAX_TRACK_WIDTH  30
#define TRACK_HALF_WIDTH    8
#define TRACK_HALF_WIDTH_MIN 4
#define TRACK_HALF_WIDTH_MAX 14
#define TF_INVALID          (-1)
#define TF_OTSU_MIN_THRESHOLD 60
#define TF_DENOISE_WHITE_MIN 4
#define TF_DENOISE_BLACK_FILL 7
#define TF_ERROR_FILTER_MAX_STEP 8
#define TF_LOOKAHEAD_FILTER_MAX_STEP 10
#define TF_FAST_VISION_SPEED 1500
#define TF_FAST_DENOISE_DIV 3u
#define TF_FAST_AUX_ROW_STEP 3u
#define TF_GLARE_PRE_GUARD_ENABLE 1u
#define TF_GLARE_PRE_ROW_WHITE_MIN 42u
#define TF_GLARE_PRE_ROW_STREAK_MIN 30u

/* Lookahead rows. */
#define TF_LOOKAHEAD_START_ROW 35
#define TF_LOOKAHEAD_END_ROW   20

#define CORNER_FILL_ENABLE 1
#define CORNER_FILL_ENTRY_MARGIN 5
#define CORNER_FILL_EXIT_MARGIN 5
#define CORNER_FILL_MIN_SPAN 6
#define CORNER_FILL_BRANCH_GAP 4
#define CORNER_FILL_EDGE_MARGIN 6
#define CORNER_FILL_BRANCH_MIN_STREAK 5
#define CORNER_FILL_EXIT_SCAN_ROWS 8
#define CORNER_FILL_L1_PCT 14
#define CORNER_FILL_L2_PCT 22
#define CORNER_FILL_OUT_SLOPE 1.6f
#define CORNER_FILL_DIR_MAX 3
#define CORNER_FILL_EXIT_SHIFT_MIN 12
#define CORNER_FILL_EXIT_SHIFT_MAX 24
#define CORNER_FILL_INNER_SHIFT_MIN 8
#define CORNER_FILL_INNER_SHIFT_MAX 16
#define CORNER_FILL_TAKEOVER_LA_ROWS 6u

#define Image_WHITE 255u
#define Image_BLACK 0u

typedef struct
{
    int16 left_edge[TF_IMG_H];
    int16 right_edge[TF_IMG_H];
    int16 center_line[TF_IMG_H];
    uint8 row_valid[TF_IMG_H];

    int16 error;
    int16 lookahead_error;
    int16 error_trend;
    uint16 valid_row_count;
    uint8 line_lost;

    uint8 left_jidian;
    uint8 right_jidian;
} TrackFusion_t;

extern TrackFusion_t g_tf;
extern uint8 g_corner_fill_active;
extern int16 g_corner_fill_center[TF_IMG_H];
extern uint8 g_corner_fill_valid[TF_IMG_H];
extern uint8 Image_Binarize[TF_IMG_H][TF_IMG_W];
extern uint8 image_0[COMP_H][COMP_W];
extern uint16 Image_Threshold;
extern volatile uint8 g_tf_image_ready;
extern volatile uint32 g_tf_frame_count;
extern volatile uint16 g_tf_white_count;

/* Intersection detection parameters. */
#define INTER_COOLDOWN_FRAMES  2
#define INTER_POST_TURN_SUPPRESS_FRAMES 8u
#define INTER_POST_TURN_BLOCK_FRAMES 2u
#define INTER_POST_TURN_FAR_BLOCK_FRAMES 1u
#define INTER_MAX_LOCK_FRAMES  300
#define INTER_BOX_START_ROW    28
#define INTER_MIN_MISS_ROW     18
#define INTER_MIN_VALID_ROWS   25
#define BOX_HEIGHT             36
#define BOX_WIDTH              60
#define INTER_BOX_WIDTH_MIN    56
#define INTER_BOX_WIDTH_MAX    78
#define INTER_BOX_WIDTH_SCALE  4
#define INTER_BOX_HEIGHT_MIN   32
#define INTER_BOX_HEIGHT_MAX   46
#define INTER_BOX_HEIGHT_SCALE 3
#define INTER_BOX_FRONT_PCT    80
#define INTER_BAND_THICKNESS   3
#define INTER_TOP_MIN_WHITE    8
#define INTER_TOP_SCAN_ROWS    6
#define INTER_SIDE_MIN_WHITE   5
#define INTER_SIDE_SCAN_COLS   10
#define INTER_SIDE_HIT_ROWS    2
#define INTER_SIDE_HIT_WHITE   8
#define INTER_SIDE_CROSS_TOP_PCT 15
#define INTER_SIDE_CROSS_BOTTOM_PCT 75
#define INTER_SIDE_CROSS_MIN_STREAK 8
#define INTER_SIDE_CROSS_HIT_ROWS 2
#define INTER_BAND_MIN_STREAK  5
#define INTER_BRANCH_MIN_STREAK 12
#define INTER_BRANCH_AREA_HALF_ROWS 4
#define INTER_BRANCH_AREA_MIN_ROWS 3
#define INTER_BRANCH_AREA_MIN_WHITE 28
#define INTER_BRANCH_STRONG_MIN_STREAK 16
#define INTER_BRANCH_STRONG_AREA_MIN_ROWS 3
#define INTER_BRANCH_STRONG_AREA_MIN_WHITE 36
#define INTER_BRANCH_PAIR_ROW_DELTA 8
#define INTER_BRANCH_NEAR_GAP 6
#define INTER_BRANCH_MIN_REACH 22
#define INTER_BRANCH_REAL_ROWS 2
#define INTER_EDGE_TOUCH_MARGIN 5u
#define INTER_EDGE_CONNECT_MIN_STREAK 14u
#define INTER_EDGE_CONNECT_HIT_ROWS 2u
#define INTER_BOX_STABLE_DELTA 3
#define INTER_BOX_STABLE_FRAMES 1
#define INTER_TYPE_VOTE_FRAMES 2
#define INTER_TYPE_VOTE_MIN    2
#define INTER_FAST_CONFIRM_ENABLE 1
#define INTER_DIRECT_FAST_CONFIRM_ROW_MARGIN 0u
#define INTER_IP_SIDE_BIAS     8
#define INTER_IP_ROW_BIAS      2
#define INTER_SYM_ROW_DELTA    3
#define INTER_SYM_CENTER_DELTA 8
#define INTER_SYM_BRANCH_STREAK 18
#define INTER_SYM_PRE_ROWS     2
#define INTER_INLINE_AHEAD_ROWS 18
#define INTER_INLINE_CENTER_MIN_WHITE 4
#define INTER_INLINE_CENTER_MIN_STREAK 2
#define INTER_INLINE_CENTER_DELTA 16
#define INTER_STRAIGHT_IGNORE_ROWS 42
#define INTER_STRAIGHT_ERR_LIMIT 18
#define INTER_STRAIGHT_LA_ERR_LIMIT 24
#define INTER_FAST_PASS_VALID_ROWS 45u
#define INTER_FAST_PASS_TOP_START_ROW 4
#define INTER_FAST_PASS_TOP_END_ROW 16
#define INTER_FAST_PASS_TOP_MIN_VALID 8u
#define INTER_FAST_PASS_ERR_LIMIT 32
#define INTER_FAST_PASS_LA_ERR_LIMIT 36
#define INTER_FAST_PASS_TREND_LIMIT 36
#define INTER_FAST_PASS_EDGE_MARGIN 6
#define INTER_TRI_CROSS_VALID_ROWS_MAX 38u
#define INTER_TRI_CROSS_TOP_VALID_MAX 3u
#define INTER_INLINE_COMPONENT_VALID_ROWS_MIN 44u
#define INTER_INLINE_COMPONENT_TOP_VALID_MIN 8u
#define INTER_INLINE_COMPONENT_BOTTOM_ROW_MIN 51
#define INTER_INLINE_COMPONENT_BOTTOM_ROW_MAX 53
#define INTER_INLINE_COMPONENT_BOTTOM_VALID_MIN 47u
#define INTER_INLINE_COMPONENT_BOTTOM_TOP_MIN 10u
#define INTER_INLINE_COMPONENT_WHITE_MIN 280u
#define INTER_INLINE_STRAIGHT_VALID_ROWS 40u
#define INTER_INLINE_STRAIGHT_TOP_VALID_MIN 4u
#define INTER_INLINE_STRAIGHT_LOWER_VALID_MIN 20u
#define INTER_INLINE_STRAIGHT_IP_ROW_MAX 32
#define INTER_INLINE_STRAIGHT_ERR_LIMIT 6
#define INTER_INLINE_STRAIGHT_LA_LIMIT 6
#define INTER_INLINE_STRAIGHT_TREND_LIMIT 6
#define INTER_COMPONENT_GUARD_ENABLE 1u
#define INTER_COMPONENT_GUARD_VALID_ROWS 32u
#define INTER_COMPONENT_GUARD_LOWER_ROWS 16u
#define INTER_COMPONENT_GUARD_STRONG_VALID_ROWS 44u
#define INTER_COMPONENT_GUARD_TOP_ROWS 4u
#define INTER_COMPONENT_GUARD_ERR_LIMIT 76
#define INTER_COMPONENT_GUARD_LA_LIMIT 76
#define INTER_COMPONENT_GUARD_TREND_LIMIT 76
#define INTER_COMPONENT_PRE_GUARD_ENABLE 1u
#define INTER_COMPONENT_PRE_VALID_ROWS 40u
#define INTER_COMPONENT_PRE_LOWER_ROWS 24u
#define INTER_COMPONENT_PRE_TOP_ROWS 2u
#define INTER_COMPONENT_PRE_CENTER_SPAN_MAX 14
#define INTER_COMPONENT_PRE_ERR_LIMIT 32
#define INTER_COMPONENT_PRE_LA_LIMIT 36
#define INTER_COMPONENT_PRE_TREND_LIMIT 36
#define INTER_ROUTE_REMAP_IP_ROW 60u
#define INTER_ROUTE_REMAP_VALID_ROWS 30u
#define INTER_COMPLEX_ARM_IP_ROW 64u
#define INTER_SINGLE_EDGE_COMPONENT_ARM_IP_ROW 88u

typedef struct
{
    uint8 valid;
    int16 row;
    int16 col;
} InflectionPoint_t;

typedef struct
{
    InflectionPoint_t left_ip;
    InflectionPoint_t right_ip;
    uint8 box_top;
    uint8 box_bottom;
    uint8 box_left;
    uint8 box_right;
    uint8 detected_type;
} IntersectionResult_t;

/* Right-angle pre-detection parameters. */
#define RA_PRE_START_ROW    (TF_IMG_H - 5)
#define RA_PRE_END_ROW      8
#define RA_PRE_LOST_THRESH  1
#define RA_PRE_EDGE_MARGIN  20
#define RA_PRE_FAR_START_ROW 28
#define RA_PRE_FAR_END_ROW   0
#define RA_PRE_FAR_WIDTH_MIN 10
#define RA_PRE_FAR_LEFT_COL  36
#define RA_PRE_FAR_RIGHT_COL 64
#define RA_PRE_FAR_OPEN_ROWS 2
#define RA_PRE_VALID_ROWS_LOW 26
#define RA_PRE_CENTER_SPAN_MIN 14
#define RA_PRE_COMPONENT_OPEN_ROWS 5
#define RA_PRE_COMPONENT_CENTER_SPAN 30
#define RA_PRE_COMPONENT_VALID_ROWS 36
#define RA_PRE_ROUTE_VALID_ROWS 42u
#define RA_PRE_ROUTE_START_ROWS 36u
#define RA_PRE_ROUTE_VALID_ROWS_MIN 8u
#define RA_PRE_ROUTE_ERR_MAX 54
#define RA_PRE_ROUTE_LA_MAX 64
#define RA_PRE_ROUTE_STRAIGHT_ERR_MAX 16
#define RA_PRE_ROUTE_STRAIGHT_LA_MAX 20
#define RA_PRE_ROUTE_IP_ROW 30u
#define RA_PRE_CONFIRM_FRAMES 1
#define RA_PRE_SLOW_OFF_FRAMES 4

extern uint8 g_ra_flag;
extern uint8 g_ra_pre_flag;
extern uint8 g_ra_pre_dir;
extern uint8 g_ra_pre_slow_flag;
extern uint8 g_sym_component_flag;
extern IntersectionResult_t g_inter_result;
extern uint8 g_ip_max_row;
extern int16 ip_col_offset;
extern int16 ip_left_col;
extern int16 ip_right_col;

void track_fusion_init(void);
void track_fusion_update(void);
void track_fusion_publish(void);
void track_intersection_suppress_after_turn(void);
void right_angle_pre_detect(void);
void detect_intersection(void);

#endif /* TRACK_FUSION_H */
