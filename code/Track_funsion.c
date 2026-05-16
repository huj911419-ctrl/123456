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
 * Binary image helper
 * ======================================================================== */
static inline uint8 is_white(int16 row, int16 col)
{
    if (row < 0 || row >= TF_IMG_H || col < 0 || col >= TF_IMG_W)
        return 0u;
    return (Image_Binarize[row][col] == Image_WHITE) ? 1u : 0u;
}

/* Same row may be visited multiple times: left = min x, right = max x */
static inline void maze_set_left_edge(int16 row, int16 col, int8 dir)
{
    if (g_tf.left_edge[row] == TF_INVALID || col < g_tf.left_edge[row])
    {
        g_tf.left_edge[row] = col;
        g_tf.left_dir[row] = dir;
    }
}

static inline void maze_set_right_edge(int16 row, int16 col, int8 dir)
{
    if (g_tf.right_edge[row] == TF_INVALID || col > g_tf.right_edge[row])
    {
        g_tf.right_edge[row] = col;
        g_tf.right_dir[row] = dir;
    }
}

/* Stop when left and right tracers meet (same row, columns within 1) */
static inline uint8 maze_tracers_met(int16 row, int16 right_cx)
{
    if (g_tf.left_edge[row] == TF_INVALID)
        return 0u;
    int16 d = right_cx - g_tf.left_edge[row];
    if (d < 0) d = -d;
    return (d <= 1) ? 1u : 0u;
}

/* ========================================================================
 * Maze edge tracing on binary image
 * ======================================================================== */

static const int8 maze_face_dir[4][2] = {{0,-1},{1,0},{0,1},{-1,0}};
static const int8 maze_left_front[4][2] = {{-1,-1},{1,-1},{1,1},{-1,1}};
static const int8 maze_right_front[4][2] = {{1,-1},{1,1},{-1,1},{-1,-1}};

#define MAZE_MAX_STEPS 300
#define MAZE_MAX_IP 10

/* Inflection points found during maze tracing (direction changes) */
static InflectionPoint_t s_maze_ips[MAZE_MAX_IP];
static uint8 s_maze_ip_cnt = 0u;

/* Record maze inflection point (direction change during tracing) */
static void maze_ip_add(int16 row, int16 col)
{
    if (s_maze_ip_cnt >= MAZE_MAX_IP) return;
    if (row < 0 || row >= TF_IMG_H || col < 0 || col >= TF_IMG_W) return;
    s_maze_ips[s_maze_ip_cnt].valid = 1u;
    s_maze_ips[s_maze_ip_cnt].row = row;
    s_maze_ips[s_maze_ip_cnt].col = col;
    s_maze_ip_cnt++;
}

static void maze_trace_edges(void)
{
    const uint8 start_row = (uint8)TF_JIDIAN_ROW;
    int16 l_start_x = -1, r_start_x = -1;

    for (int16 c = TF_IMG_CENTER; c >= 1; c--)
    {
        if (is_white((int16)start_row, c) && !is_white((int16)start_row, c - 1))
        {
            l_start_x = c;
            break;
        }
    }
    for (int16 c = TF_IMG_CENTER; c < TF_IMG_W - 1; c++)
    {
        if (is_white((int16)start_row, c) && !is_white((int16)start_row, c + 1))
        {
            r_start_x = c;
            break;
        }
    }

    if (l_start_x < 0 || r_start_x < 0 || l_start_x >= r_start_x)
    {
        g_tf.valid_row_count = 0u;
        s_maze_ip_cnt = 0u;
        return;
    }

    s_maze_ip_cnt = 0u;

    g_tf.left_jidian = (uint8)l_start_x;
    g_tf.right_jidian = (uint8)r_start_x;

    /* Left trace */
    {
        int16 cx = l_start_x + 1, cy = (int16)start_row;
        uint8 dir = 0;
        uint8 turn_cnt = 0;
        uint16 steps = 0;
        int8 prev_growth_dir = 0;

        maze_set_left_edge((int16)start_row, cx, 0);

        while (steps++ < MAZE_MAX_STEPS)
        {
            int16 fr = cy + maze_face_dir[dir][1];
            int16 fc = cx + maze_face_dir[dir][0];
            uint8 f_ok = is_white(fr, fc);
            if (!f_ok)
            {
                dir = (dir + 1) & 3;
                if (++turn_cnt >= 4) break;
                continue;
            }

            int16 lfr = cy + maze_left_front[dir][1];
            int16 lfc = cx + maze_left_front[dir][0];
            uint8 lf_ok = is_white(lfr, lfc);
            int8 dx, dy;
            if (lf_ok)
            {
                dx = maze_left_front[dir][0];
                dy = maze_left_front[dir][1];
                dir = (dir + 3) & 3;
            }
            else
            {
                dx = maze_face_dir[dir][0];
                dy = maze_face_dir[dir][1];
            }

            int16 nx = cx + dx, ny = cy + dy;
            if (nx < 0 || nx >= TF_IMG_W || ny < 0 || ny >= TF_IMG_H) break;

            if (steps >= 5 && ny == (int16)start_row &&
                nx == l_start_x + 1) break;

            cx = nx; cy = ny;
            turn_cnt = 0;

            int8 growth_dir = (int8)(dx * 3 - dy);
            maze_set_left_edge(cy, cx, growth_dir);

            if (prev_growth_dir != 0 && growth_dir != prev_growth_dir)
                maze_ip_add(cy, cx);
            prev_growth_dir = growth_dir;

            if (cy <= (int16)TF_SEARCH_END_ROW) break;
        }
    }

    /* Right trace */
    {
        int16 cx = r_start_x - 1, cy = (int16)start_row;
        uint8 dir = 0;
        uint8 turn_cnt = 0;
        uint16 steps = 0;
        int8 prev_growth_dir = 0;

        maze_set_right_edge((int16)start_row, cx, 0);

        while (steps++ < MAZE_MAX_STEPS)
        {
            int16 fr = cy + maze_face_dir[dir][1];
            int16 fc = cx + maze_face_dir[dir][0];
            uint8 f_ok = is_white(fr, fc);
            if (!f_ok)
            {
                dir = (dir + 3) & 3;
                if (++turn_cnt >= 4) break;
                continue;
            }

            int16 rfr = cy + maze_right_front[dir][1];
            int16 rfc = cx + maze_right_front[dir][0];
            uint8 rf_ok = is_white(rfr, rfc);
            int8 dx, dy;
            if (rf_ok)
            {
                dx = maze_right_front[dir][0];
                dy = maze_right_front[dir][1];
                dir = (dir + 1) & 3;
            }
            else
            {
                dx = maze_face_dir[dir][0];
                dy = maze_face_dir[dir][1];
            }

            int16 nx = cx + dx, ny = cy + dy;
            if (nx < 0 || nx >= TF_IMG_W || ny < 0 || ny >= TF_IMG_H) break;

            if (steps >= 5 && ny == (int16)start_row &&
                nx == r_start_x - 1) break;

            cx = nx; cy = ny;
            turn_cnt = 0;

            int8 growth_dir = (int8)(dx * 3 - dy);
            maze_set_right_edge(cy, cx, growth_dir);

            if (prev_growth_dir != 0 && growth_dir != prev_growth_dir)
                maze_ip_add(cy, cx);
            prev_growth_dir = growth_dir;

            if (maze_tracers_met(cy, cx))
                break;

            if (cy <= (int16)TF_SEARCH_END_ROW) break;
        }
    }

    /* Build center line and row validity */
    uint16 valid_cnt = 0u;
    int16 half_w = (TF_MIN_TRACK_WIDTH + TF_MAX_TRACK_WIDTH) / 4;

    for (int16 row = (int16)start_row; row >= (int16)TF_SEARCH_END_ROW; row--)
    {
        uint8 has_l = (g_tf.left_edge[row] != TF_INVALID) ? 1u : 0u;
        uint8 has_r = (g_tf.right_edge[row] != TF_INVALID) ? 1u : 0u;

        if (has_l && has_r)
        {
            int16 w = g_tf.right_edge[row] - g_tf.left_edge[row];
            if (w >= TF_MIN_TRACK_WIDTH && w <= TF_MAX_TRACK_WIDTH)
            {
                g_tf.center_line[row] = (g_tf.left_edge[row] + g_tf.right_edge[row]) / 2;
                g_tf.row_valid[row] = 1u;
                valid_cnt++;
            }
            else
            {
                g_tf.center_line[row] = (g_tf.left_edge[row] + g_tf.right_edge[row]) / 2;
                g_tf.row_valid[row] = 0u;
            }
        }
        else if (has_l)
        {
            g_tf.center_line[row] = g_tf.left_edge[row] + half_w;
            g_tf.row_valid[row] = 0u;
        }
        else if (has_r)
        {
            g_tf.center_line[row] = g_tf.right_edge[row] - half_w;
            g_tf.row_valid[row] = 0u;
        }
        else
        {
            g_tf.center_line[row] = TF_INVALID;
            g_tf.row_valid[row] = 0u;
        }
    }

    g_tf.valid_row_count = valid_cnt;
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
        g_tf.left_dir[i] = 0;
        g_tf.right_dir[i] = 0;
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
 * Per-frame update: compress -> Otsu -> binarize -> denoise -> maze trace
 * ======================================================================== */
void track_fusion_update(void)
{
    /* 0. Frame sync */
    while (!mt9v03x_finish_flag)
        ;
    mt9v03x_finish_flag = 0;

    /* 1. Clear per-frame data */
    for (uint8 i = 0; i < TF_IMG_H; i++)
    {
        g_tf.left_edge[i] = TF_INVALID;
        g_tf.right_edge[i] = TF_INVALID;
        g_tf.center_line[i] = TF_INVALID;
        g_tf.row_valid[i] = 0u;
        g_tf.left_dir[i] = 0;
        g_tf.right_dir[i] = 0;
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

    /* 6. Maze edge tracing */
    maze_trace_edges();

    if (g_tf.valid_row_count == 0u)
    {
        g_tf.line_lost = 1u;
        return;
    }

    /* 7. Weighted center-line error (bottom-heavy for normal tracking) */
    int16 jrow = (int16)TF_JIDIAN_ROW;
    int16 end_row = (int16)TF_SEARCH_END_ROW;

    int16 avg_center = calc_weighted_center(jrow, end_row, 0);
    g_tf.error = -(avg_center - (int16)TF_IMG_CENTER);
    g_tf.line_lost = 0u;

    /* 8. Lookahead error (top-heavy for curve prediction) */
    {
        int16 la_center = calc_weighted_center(
            (int16)TF_LOOKAHEAD_START_ROW, (int16)TF_LOOKAHEAD_END_ROW, 1);
        g_tf.lookahead_error = -(la_center - (int16)TF_IMG_CENTER);
        g_tf.error_trend = g_tf.lookahead_error - g_tf.error;
    }

    /* 9. Scale errors to original coordinate system (x2) for PID compatibility */
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
 * Inflection point detection (maze direction change) + box drawing
 * ======================================================================== */

uint8 g_ra_flag = 0u;
IntersectionResult_t g_inter_result;
uint8 g_ip_max_row = 0u;

static uint8 s_inter_lock_cnt = 0u;
static uint8 s_inter_cooldown_cnt = 0u;
static uint8 s_inter_confirm_cnt = 0u;
static uint8 s_inter_candidate = 0u;

/* Main intersection detection: use maze-detected inflection points + box + classification */
void detect_intersection(void)
{
    if (s_inter_cooldown_cnt > 0u)
    {
        s_inter_cooldown_cnt++;
        if (s_inter_cooldown_cnt >= INTER_COOLDOWN_FRAMES) s_inter_cooldown_cnt = 0u;
        return;
    }

    /* Find inflection point closest to camera (largest row) from maze results */
    InflectionPoint_t best_ip;
    best_ip.valid = 0u; best_ip.row = 0; best_ip.col = 0;

    for (uint8 i = 0; i < s_maze_ip_cnt; i++)
    {
        if (s_maze_ips[i].valid && s_maze_ips[i].row > best_ip.row)
            best_ip = s_maze_ips[i];
    }

    /* Find left-most and right-most inflection points for display */
    InflectionPoint_t left_ip, right_ip;
    left_ip.valid = 0u; left_ip.row = 0; left_ip.col = TF_IMG_W;
    right_ip.valid = 0u; right_ip.row = 0; right_ip.col = 0;

    for (uint8 i = 0; i < s_maze_ip_cnt; i++)
    {
        if (!s_maze_ips[i].valid) continue;
        if (s_maze_ips[i].col < left_ip.col)  left_ip  = s_maze_ips[i];
        if (s_maze_ips[i].col > right_ip.col) right_ip = s_maze_ips[i];
    }

    g_inter_result.left_ip  = left_ip.valid  ? left_ip  : best_ip;
    g_inter_result.right_ip = right_ip.valid ? right_ip : best_ip;

    g_ip_max_row = best_ip.valid ? (uint8)best_ip.row : 0u;

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

    if (!best_ip.valid) return;

    /* Inflection point too far, skip box drawing */
    if (g_ip_max_row < INTER_BOX_START_ROW) return;

    /* Build bounding box around inflection point(s) */
    uint8 detected = 0u;

    int16 cx, cy;
    cx = best_ip.col;
    cy = best_ip.row;

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

    /* Check top/left/right edges of box for continuous white pixels (road exists) */
    uint8 top_has_road = 0u;
    uint8 left_has_road = 0u;
    uint8 right_has_road = 0u;

    {
        uint8 streak = 0u;
        for (int16 c = b_left; c <= b_right; c++)
        {
            if (Image_Binarize[b_top][c] == Image_WHITE)
            {
                streak++;
                if (streak >= INTER_BOX_MIN_STREAK) { top_has_road = 1u; break; }
            }
            else { streak = 0u; }
        }
    }
    {
        uint8 streak = 0u;
        for (int16 r = b_top; r <= b_bottom; r++)
        {
            if (Image_Binarize[r][b_left] == Image_WHITE)
            {
                streak++;
                if (streak >= INTER_BOX_MIN_STREAK) { left_has_road = 1u; break; }
            }
            else { streak = 0u; }
        }
    }
    {
        uint8 streak = 0u;
        for (int16 r = b_top; r <= b_bottom; r++)
        {
            if (Image_Binarize[r][b_right] == Image_WHITE)
            {
                streak++;
                if (streak >= INTER_BOX_MIN_STREAK) { right_has_road = 1u; break; }
            }
            else { streak = 0u; }
        }
    }

    /* Direction classification
     * 1=right only  2=left only  3=left+right  4=cross(top+left+right)
     * 5=top+right   6=top+left+right */
    if (top_has_road && left_has_road && right_has_road)
        detected = 6u;
    else if (top_has_road && right_has_road)
        detected = 5u;
    else if (left_has_road && right_has_road)
        detected = 3u;
    else if (right_has_road)
        detected = 1u;
    else if (left_has_road)
        detected = 2u;
    else if (top_has_road)
        detected = 4u;

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
