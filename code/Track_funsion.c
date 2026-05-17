#include "Track_funsion.h"

/* ========================================================================
 * Global variables
 * ======================================================================== */

TrackFusion_t g_tf;
uint8 Image_Binarize[TF_IMG_H][TF_IMG_W];
uint8 image_0[COMP_H][COMP_W];
uint16 Image_Threshold = 60;
int16 threshold_bias = 0;

/* Otsu internals */
static uint8 s_otsu_cnt = 0;
static uint32 s_hist[256];

/* First miss row and last valid center for inflection detection */
static int16 s_first_miss_row = -1;
static int16 s_last_valid_center = TF_IMG_CENTER;

#define IP_COL_BUF_SIZE 8
static int16 s_center_buf[IP_COL_BUF_SIZE];
static uint8 s_center_buf_cnt = 0u;
int16 ip_col_offset = 5;  // 0=鍊掓暟绗�1涓紝1=鍊掓暟绗�2涓�...

/* ========================================================================
 * Image compression: 188x120 -> 94x60 (nearest-neighbor)
 * ======================================================================== */
static void compress_image(void)
{
    float div_h = (float)MT9V03X_H / COMP_H;
    float div_w = (float)MT9V03X_W / COMP_W;

    for (int i = 0; i < COMP_H; i++)
    {
        int row = (int)(i * div_h + 0.5f);
        if (row >= MT9V03X_H) row = MT9V03X_H - 1;

        for (int j = 0; j < COMP_W; j++)
        {
            int col = (int)(j * div_w + 0.5f);
            if (col >= MT9V03X_W) col = MT9V03X_W - 1;
            image_0[i][j] = mt9v03x_image[row][col];
        }
    }
}

/* ========================================================================
 * Otsu adaptive threshold (compressed image)
 * ======================================================================== */
static uint16 calc_otsu(void)
{
    uint32 total = (uint32)COMP_H * COMP_W;
    float sum = 0.0f;
    uint16 threshold = 60;

    for (uint16 i = 0; i < 256; i++) s_hist[i] = 0;
    for (uint8 i = 0; i < COMP_H; i++)
        for (uint8 j = 0; j < COMP_W; j++)
            s_hist[image_0[i][j]]++;

    for (uint16 t = 0; t < 256; t++)
        sum += (float)t * s_hist[t];

    float sumB = 0.0f;
    uint32 wB = 0;
    float maxVar = 0.0f;

    for (uint16 t = 0; t < 256; t++)
    {
        wB += s_hist[t];
        if (wB == 0) continue;

        uint32 wF = total - wB;
        if (wF == 0) break;

        sumB += (float)t * s_hist[t];

        float mB = sumB / wB;
        float mF = (sum - sumB) / wF;
        float varBetween = (float)wB * wF * (mB - mF) * (mB - mF);

        if (varBetween > maxVar)
        {
            maxVar = varBetween;
            threshold = t;
        }
    }

    if (threshold < 60) threshold = 60;

    /* Apply threshold bias from menu */
    int16 biased = (int16)threshold + threshold_bias;
    if (biased < 20) biased = 20;
    if (biased > 240) biased = 240;

    return (uint16)biased;
}

/* ========================================================================
 * Binarization (compressed image)
 * ======================================================================== */
static void binarize_image(void)
{
    for (uint16 i = 0; i < COMP_H; i++)
        for (uint16 j = 0; j < COMP_W; j++)
            Image_Binarize[i][j] = (image_0[i][j] > Image_Threshold) ? Image_WHITE : Image_BLACK;
}

/* ========================================================================
 * 4-neighbor denoise on binary image
 * ======================================================================== */
static void denoise_binary_image(void)
{
    for (uint8 i = 1; i < COMP_H - 1; i++)
    {
        for (uint8 j = 1; j < COMP_W - 1; j++)
        {
            uint16 neighbor_sum = Image_Binarize[i-1][j]
                                + Image_Binarize[i+1][j]
                                + Image_Binarize[i][j-1]
                                + Image_Binarize[i][j+1];

            if (Image_Binarize[i][j] == Image_WHITE && neighbor_sum < 2 * Image_WHITE)
                Image_Binarize[i][j] = Image_BLACK;
            else if (Image_Binarize[i][j] == Image_BLACK && neighbor_sum > 2 * Image_WHITE)
                Image_Binarize[i][j] = Image_WHITE;
        }
    }
}

/* ========================================================================
 * Binary image helpers
 * ======================================================================== */
static inline uint8 is_white(int16 row, int16 col)
{
    if (row < 0 || row >= TF_IMG_H || col < 0 || col >= TF_IMG_W)
        return 0u;
    return (Image_Binarize[row][col] == Image_WHITE) ? 1u : 0u;
}

static inline uint8 is_left_edge(int16 row, int16 col)
{
    if (col < 1 || col >= TF_IMG_W - 1) return 0u;
    return (!is_white(row, col - 1) && is_white(row, col) && is_white(row, col + 1)) ? 1u : 0u;
}

static inline uint8 is_right_edge(int16 row, int16 col)
{
    if (col < 1 || col >= TF_IMG_W - 1) return 0u;
    return (is_white(row, col - 1) && is_white(row, col) && !is_white(row, col + 1)) ? 1u : 0u;
}

/* ========================================================================
 * Edge scanning functions (boundary-safe)
 * ======================================================================== */
static int16 scan_left_edge_right(int16 row, int16 start, int16 end)
{
    if (start < 1) start = 1;
    if (end >= TF_IMG_W - 1) end = TF_IMG_W - 2;
    if (start > end) return TF_INVALID;
    for (int16 c = start; c <= end; c++)
        if (is_left_edge(row, c)) return c;
    return TF_INVALID;
}

static int16 scan_left_edge_left(int16 row, int16 start, int16 end)
{
    if (start >= TF_IMG_W - 1) start = TF_IMG_W - 2;
    if (end < 1) end = 1;
    if (start < end) return TF_INVALID;
    for (int16 c = start; c >= end; c--)
        if (is_left_edge(row, c)) return c;
    return TF_INVALID;
}

static int16 scan_right_edge_left(int16 row, int16 start, int16 end)
{
    if (start >= TF_IMG_W - 1) start = TF_IMG_W - 2;
    if (end < 1) end = 1;
    if (start < end) return TF_INVALID;
    for (int16 c = start; c >= end; c--)
        if (is_right_edge(row, c)) return c;
    return TF_INVALID;
}

static int16 scan_right_edge_right(int16 row, int16 start, int16 end)
{
    if (start < 1) start = 1;
    if (end >= TF_IMG_W - 1) end = TF_IMG_W - 2;
    if (start > end) return TF_INVALID;
    for (int16 c = start; c <= end; c++)
        if (is_right_edge(row, c)) return c;
    return TF_INVALID;
}

static inline uint8 valid_pair(int16 lb, int16 rb)
{
    if (lb == TF_INVALID || rb == TF_INVALID) return 0u;
    int16 w = rb - lb;
    return (w >= TF_MIN_TRACK_WIDTH && w <= TF_MAX_TRACK_WIDTH) ? 1u : 0u;
}

/* ========================================================================
 * Baseline detection at bottom row
 * ======================================================================== */
static uint8 find_jidian1(void)
{
    const int16 row = (int16)TF_JIDIAN_ROW;
    const int16 mid = (int16)TF_IMG_CENTER;
    const int16 qtr = (int16)(TF_IMG_W / 4);
    const int16 tqtr = (int16)(TF_IMG_W * 3 / 4);
    int16 lb = TF_INVALID, rb = TF_INVALID;

    if (is_white(row, mid - 1) && is_white(row, mid) && is_white(row, mid + 1))
    {
        lb = scan_left_edge_left(row, mid, 1);
        rb = scan_right_edge_right(row, mid, TF_IMG_W - 2);
    }
    else if (is_white(row, qtr - 1) && is_white(row, qtr) && is_white(row, qtr + 1))
    {
        lb = scan_left_edge_left(row, qtr, 1);
        rb = scan_right_edge_right(row, qtr, TF_IMG_W - 2);
    }
    else if (is_white(row, tqtr - 1) && is_white(row, tqtr) && is_white(row, tqtr + 1))
    {
        lb = scan_left_edge_left(row, tqtr, 1);
        rb = scan_right_edge_right(row, tqtr, TF_IMG_W - 2);
    }
    else
    {
        lb = scan_left_edge_right(row, 1, TF_IMG_W - 2);
        if (lb != TF_INVALID)
            rb = scan_right_edge_left(row, TF_IMG_W - 2, lb + TF_MIN_TRACK_WIDTH);
    }

    if (!valid_pair(lb, rb)) return 0u;
    g_tf.left_jidian = (uint8)lb;
    g_tf.right_jidian = (uint8)rb;
    return 1u;
}

/* ========================================================================
 * Row-by-row edge search (local range from previous row)
 * ======================================================================== */
static uint8 search_row_edges(int16 row, int16 prev_lb, int16 prev_rb,
                              int16 *out_lb, int16 *out_rb)
{
    int16 lb = TF_INVALID, rb = TF_INVALID;
    const int16 R = TF_LOCAL_RANGE;
    const int16 MID = TF_IMG_CENTER;

    /* Left edge search: local 鈫� widen 鈫� full scan */
    lb = scan_left_edge_right(row, prev_lb - R, prev_lb + R);
    if (lb == TF_INVALID) lb = scan_left_edge_left(row, prev_lb + R, prev_lb - R);
    if (lb == TF_INVALID) lb = scan_left_edge_left(row, MID, 1);

    /* Right edge search: local 鈫� widen 鈫� full scan */
    rb = scan_right_edge_left(row, prev_rb + R, prev_rb - R);
    if (rb == TF_INVALID) rb = scan_right_edge_right(row, prev_rb - R, prev_rb + R);
    if (rb == TF_INVALID) rb = scan_right_edge_right(row, MID, TF_IMG_W - 2);

    *out_lb = lb;
    *out_rb = rb;
    return valid_pair(lb, rb);
}

/* ========================================================================
 * Initialization
 * ======================================================================== */
void track_fusion_init(void)
{
    uint16 i, j;
    for (i = 0; i < TF_IMG_H; i++)
    {
        g_tf.left_edge[i] = TF_INVALID;
        g_tf.right_edge[i] = TF_INVALID;
        g_tf.center_line[i] = TF_INVALID;
        g_tf.row_valid[i] = 0u;
    }
    g_tf.error = 0;
    g_tf.lookahead_error = 0;
    g_tf.error_trend = 0;
    g_tf.valid_row_count = 0u;
    g_tf.line_lost = 0u;
    g_tf.left_jidian = (uint8)TF_IMG_CENTER;
    g_tf.right_jidian = (uint8)TF_IMG_CENTER;
    Image_Threshold = 60;
    threshold_bias = 0;
    s_otsu_cnt = (uint8)TF_OTSU_INTERVAL;

    for (i = 0; i < COMP_H; i++)
        for (j = 0; j < COMP_W; j++)
        {
            Image_Binarize[i][j] = Image_BLACK;
            image_0[i][j] = 0u;
        }
}

/* ========================================================================
 * Weighted center-line error calculation
 * ======================================================================== */
static int16 calc_weighted_center(int16 start_row, int16 end_row, uint8 top_heavy)
{
    int32 sum = 0, wtotal = 0;
    for (int16 row = start_row; row >= end_row; row--)
    {
        if (g_tf.row_valid[row])
        {
            int32 w = top_heavy
                ? (int32)(start_row - row + 1)
                : (int32)(row - end_row + 1);
            sum += (int32)g_tf.center_line[row] * w;
            wtotal += w;
        }
    }
    return (wtotal > 0) ? (int16)(sum / wtotal) : (int16)TF_IMG_CENTER;
}

/* ========================================================================
 * Per-frame update: compress -> Otsu -> binarize -> denoise -> scan edges
 * ======================================================================== */
void track_fusion_update(void)
{
    /* 1. Clear per-frame data */
    for (uint8 i = 0; i < TF_IMG_H; i++)
    {
        g_tf.left_edge[i] = TF_INVALID;
        g_tf.right_edge[i] = TF_INVALID;
        g_tf.center_line[i] = TF_INVALID;
        g_tf.row_valid[i] = 0u;
    }
    g_tf.error = 0;
    g_tf.valid_row_count = 0u;
    g_tf.line_lost = 0u;

    /* 2. Compress 188x120 -> 94x60 */
    compress_image();

    /* 3. Otsu adaptive threshold */
    s_otsu_cnt++;
    if (s_otsu_cnt >= TF_OTSU_INTERVAL)
    {
        s_otsu_cnt = 0u;
        Image_Threshold = calc_otsu();
    }

    /* 4. Binarize */
    binarize_image();

    /* 5. Denoise (4-neighbor filter) */
    denoise_binary_image();

    /* 6. Find baseline at bottom row */
    if (!find_jidian1())
    {
        g_tf.line_lost = 1u;
        return;
    }

    const int16 jrow = (int16)TF_JIDIAN_ROW;
    int16 prev_lb = (int16)g_tf.left_jidian;
    int16 prev_rb = (int16)g_tf.right_jidian;

    g_tf.left_edge[jrow] = prev_lb;
    g_tf.right_edge[jrow] = prev_rb;
    g_tf.center_line[jrow] = (prev_lb + prev_rb) / 2;
    g_tf.row_valid[jrow] = 1u;
    g_tf.valid_row_count = 1u;

    /* 7. Row-by-row upward edge search */
    uint8 miss_streak = 0u;
    int16 end_row = (int16)TF_SEARCH_END_ROW;
    s_first_miss_row = -1;
    s_center_buf_cnt = 0u;
    s_last_valid_center = (prev_lb + prev_rb) / 2;

    for (int16 row = jrow - 1; row >= end_row; row--)
    {
        int16 lb, rb;
        if (search_row_edges(row, prev_lb, prev_rb, &lb, &rb))
        {
            g_tf.left_edge[row] = lb;
            g_tf.right_edge[row] = rb;
            g_tf.center_line[row] = (lb + rb) / 2;
            g_tf.row_valid[row] = 1u;
            g_tf.valid_row_count++;
            prev_lb = lb;
            prev_rb = rb;
            s_last_valid_center = g_tf.center_line[row];
            s_center_buf[s_center_buf_cnt % IP_COL_BUF_SIZE] = g_tf.center_line[row];
            s_center_buf_cnt++;
            miss_streak = 0u;
        }
        else
        {
            g_tf.left_edge[row] = lb;
            g_tf.right_edge[row] = rb;
            g_tf.row_valid[row] = 0u;
            miss_streak++;
            if (miss_streak == 2 && s_first_miss_row < 0)
                s_first_miss_row = row + 1;  /* row+1 = last valid row before miss */
            if (miss_streak >= TF_MAX_MISS_ROWS) break;
        }
    }

    if (g_tf.valid_row_count == 0u)
    {
        g_tf.line_lost = 1u;
        return;
    }

    /* 8. Weighted center-line error (bottom-heavy for normal tracking) */
    int16 avg_center = calc_weighted_center(jrow, end_row, 0);
    g_tf.error = -(avg_center - (int16)TF_IMG_CENTER);
    g_tf.line_lost = 0u;

    /* 9. Lookahead error (top-heavy for curve prediction) */
    {
        int16 la_center = calc_weighted_center(
            (int16)TF_LOOKAHEAD_START_ROW, (int16)TF_LOOKAHEAD_END_ROW, 1);
        g_tf.lookahead_error = -(la_center - (int16)TF_IMG_CENTER);
        g_tf.error_trend = g_tf.lookahead_error - g_tf.error;
    }

    /* 10. Scale errors to original coordinate system (x2) for PID compatibility */
    g_tf.error = g_tf.error * 2;
    g_tf.lookahead_error = g_tf.lookahead_error * 2;
    g_tf.error_trend = g_tf.error_trend * 2;
}

/* ========================================================================
 * Right angle pre-detection (far-field, for early speed reduction)
 * ======================================================================== */
uint8 g_ra_pre_flag = 0u;

void right_angle_pre_detect(void)
{
    static uint8 s_on_cnt = 0;
    static uint8 s_off_cnt = 0;

    uint8 right_lost = 0u;
    uint8 left_lost = 0u;

    for (int16 i = (int16)RA_PRE_START_ROW;
         i > (int16)RA_PRE_END_ROW; i--)
    {
        if (!g_tf.row_valid[i])
            continue;

        if (g_tf.right_edge[i] >= (int16)(TF_IMG_W - RA_PRE_EDGE_MARGIN))
            right_lost++;

        if (g_tf.left_edge[i] <= (int16)RA_PRE_EDGE_MARGIN)
            left_lost++;
    }

    uint8 detected = (right_lost >= RA_PRE_LOST_THRESH ||
                      left_lost >= RA_PRE_LOST_THRESH) ? 1u : 0u;

    if (detected)
    {
        s_on_cnt++;
        s_off_cnt = 0;
        if (s_on_cnt >= 2)
            g_ra_pre_flag = 1u;
    }
    else
    {
        s_off_cnt++;
        s_on_cnt = 0;
        if (s_off_cnt >= 5)
            g_ra_pre_flag = 0u;
    }
}

/* ========================================================================
 * Inflection point detection (scan upward from lost row) + box drawing
 *
 * Method: when row-by-row scanning loses track, scan upward from that
 * row to find white pixels at the leftmost/rightmost image columns.
 * White at left edge  鈫� left turn,  inflection at right side of track
 * White at right edge 鈫� right turn, inflection at left side of track
 * White at both edges 鈫� cross,      inflection at center
 * ======================================================================== */

uint8 g_ra_flag = 0u;
IntersectionResult_t g_inter_result;
uint8 g_ip_max_row = 0u;

static uint8 s_inter_lock_cnt = 0u;
static uint8 s_inter_cooldown_cnt = 0u;
static uint8 s_inter_confirm_cnt = 0u;
static uint8 s_inter_candidate = 0u;

/* Find inflection point: scan upward from lost row,
 * find lowest white pixel at left/right image boundary.
 * Returns which side has white via found_side: 1=right, 2=left, 3=both */
static uint8 find_ip_from_lost_row(int16 lost_row, int16 last_center,
                                    InflectionPoint_t *ip, uint8 *found_side)
{
    ip->valid = 0u; ip->row = 0; ip->col = 0;
    *found_side = 0u;

    int16 left_white_row = -1;
    int16 right_white_row = -1;

    for (int16 row = lost_row; row >= (int16)TF_SEARCH_END_ROW; row--)
    {
        /* Check leftmost column for white */
        if (left_white_row < 0 && Image_Binarize[row][0] == Image_WHITE)
            left_white_row = row;

        /* Check rightmost column for white */
        if (right_white_row < 0 && Image_Binarize[row][TF_IMG_W - 1] == Image_WHITE)
            right_white_row = row;

        /* Both found, no need to scan further */
        if (left_white_row >= 0 && right_white_row >= 0)
            break;
    }

    /* Take the one closer to camera (larger row number) */
    int16 ip_row = -1;
    if (left_white_row >= 0 && right_white_row >= 0)
    {
        ip_row = (left_white_row > right_white_row) ? left_white_row : right_white_row;
        *found_side = 3u; /* both sides */
    }
    else if (right_white_row >= 0)
    {
        ip_row = right_white_row;
        *found_side = 1u; /* right side has road 鈫� right turn */
    }
    else if (left_white_row >= 0)
    {
        ip_row = left_white_row;
        *found_side = 2u; /* left side has road 鈫� left turn */
    }

    if (ip_row < 0) return 0u;

    ip->valid = 1u;
    ip->row = ip_row;
    ip->col = last_center;
    return 1u;
}

/* Main intersection detection */
void detect_intersection(void)
{
    if (s_inter_cooldown_cnt > 0u)
    {
        s_inter_cooldown_cnt++;
        if (s_inter_cooldown_cnt >= INTER_COOLDOWN_FRAMES) s_inter_cooldown_cnt = 0u;
        return;
    }

    /* Find inflection point from lost row */
    InflectionPoint_t ip;
    ip.valid = 0u; ip.row = 0; ip.col = 0;
    uint8 found_side = 0u; /* 1=right, 2=left, 3=both */

    if (s_first_miss_row >= 0)
        find_ip_from_lost_row(s_first_miss_row, s_last_valid_center, &ip, &found_side);

    /* 鐢ㄥ�掓暟绗琋涓湁鏁堣鐨勪腑鐐逛綔涓烘鍒楀彿 */
    if (ip.valid && s_center_buf_cnt > 0u)
    {
        uint8 offset = (uint8)ip_col_offset;
        uint8 total = s_center_buf_cnt;
        if (offset >= total) offset = total - 1;
        uint8 idx = (uint8)((total - 1 - offset) % IP_COL_BUF_SIZE);
        ip.col = s_center_buf[idx];
    }

    g_inter_result.left_ip = ip;
    g_inter_result.right_ip = ip;

    g_ip_max_row = ip.valid ? (uint8)ip.row : 0u;

    if (s_inter_lock_cnt > 0u)
    {
        if (g_ra_flag == 0u)
        { s_inter_lock_cnt = 0u; s_inter_cooldown_cnt = 1u; }
        else
        {
            s_inter_lock_cnt++;
            if (s_inter_lock_cnt >= INTER_MAX_LOCK_FRAMES)
            { g_ra_flag = 0u; s_inter_lock_cnt = 0u; s_inter_cooldown_cnt = 1u; }
        }
        return;
    }

    if (!ip.valid) return;

    /* Inflection point too far, skip box drawing */
    if (g_ip_max_row < INTER_BOX_START_ROW) return;

    /* Build bounding box: extend to image edges on left/right so white detection works */
    uint8 detected = 0u;

    int16 cy = ip.row;

    int16 cx = ip.col;

    int16 b_top = cy - BOX_HEIGHT / 2;
    int16 b_bottom = cy + BOX_HEIGHT / 2;
    int16 b_left = cx - BOX_WIDTH / 2;
    int16 b_right = cx + BOX_WIDTH / 2;

    if (b_top < 0) b_top = 0;
    if (b_bottom >= TF_IMG_H) b_bottom = TF_IMG_H - 1;
    if (b_left < 0) b_left = 0;
    if (b_right >= TF_IMG_W) b_right = TF_IMG_W - 1;

    g_inter_result.box_top = (uint8)b_top;
    g_inter_result.box_bottom = (uint8)b_bottom;
    g_inter_result.box_left = (uint8)b_left;
    g_inter_result.box_right = (uint8)b_right;

    /* Check each box edge for continuous white pixels */
    uint8 top_has = 0u, left_has = 0u, right_has = 0u;
    {
        uint8 streak;
        /* Top edge */
        streak = 0u;
        for (int16 c = b_left; c <= b_right; c++)
        {
            if (Image_Binarize[b_top][c] == Image_WHITE)
            { streak++; if (streak >= INTER_BOX_MIN_STREAK) { top_has = 1u; break; } }
            else { streak = 0u; }
        }
        /* Left edge */
        streak = 0u;
        for (int16 r = b_top; r <= b_bottom; r++)
        {
            if (Image_Binarize[r][b_left] == Image_WHITE)
            { streak++; if (streak >= INTER_BOX_MIN_STREAK) { left_has = 1u; break; } }
            else { streak = 0u; }
        }
        /* Right edge */
        streak = 0u;
        for (int16 r = b_top; r <= b_bottom; r++)
        {
            if (Image_Binarize[r][b_right] == Image_WHITE)
            { streak++; if (streak >= INTER_BOX_MIN_STREAK) { right_has = 1u; break; } }
            else { streak = 0u; }
        }
    }

    /* 鍒嗙被锛堟杈规娴嬶級锛氫笂+宸�=3  涓�+鍙�=4  宸�+鍙�=5  涓�+宸�+鍙�=6
     * 鍗曡竟鐢� found_side锛堝浘鍍忚竟鐣屾娴嬶級锛氬彸=1  宸�=2 */
    if (top_has && left_has && right_has)
        detected = 6u;
    else if (top_has && left_has)
        detected = 3u;
    else if (top_has && right_has)
        detected = 4u;
    else if (left_has && right_has)
        detected = 5u;
    else if (found_side == 1u)
        detected = 1u;
    else if (found_side == 2u)
        detected = 2u;

    g_inter_result.detected_type = detected;

    /* 3-frame confirmation */
    if (detected != 0u && detected == s_inter_candidate)
    {
        s_inter_confirm_cnt++;
        if (s_inter_confirm_cnt >= 3u)
        { g_ra_flag = detected; s_inter_lock_cnt = 1u; s_inter_confirm_cnt = 0u; s_inter_candidate = 0u; }
    }
    else if (detected != 0u)
    { s_inter_candidate = detected; s_inter_confirm_cnt = 1u; }
    else
    { s_inter_confirm_cnt = 0u; s_inter_candidate = 0u; }

    /* Scale inflection row back to original coordinate system (x2) for RA menu params */
    g_ip_max_row = g_ip_max_row * 2;
}
