#include "Track_funsion.h"

/* ========================================================================
 * 全局变量定义
 * ======================================================================== */

TrackFusion_t g_tf;
uint8 bin_image[TF_IMG_H][TF_IMG_W];
uint8 compressed_gray[COMP_H][COMP_W];

/* Otsu内部变量 */
static uint8 s_otsu_cnt = 0;
static uint16 s_hist[256];

/* ========================================================================
 * 图像压缩：188×120 → 94×60（2×2块平均）
 * ======================================================================== */
static void compress_image(void)
{
    for (uint16 i = 0; i < COMP_H; i++)
    {
        uint8 *src0 = &mt9v03x_image[i * 2][0];
        uint8 *src1 = &mt9v03x_image[i * 2 + 1][0];
        uint8 *dst = compressed_gray[i];

        for (uint16 j = 0; j < COMP_W; j++)
        {
            uint16 sum = (uint16)src0[0] + (uint16)src0[1]
                       + (uint16)src1[0] + (uint16)src1[1];
            *dst++ = (uint8)((sum + 2) >> 2);
            src0 += 2;
            src1 += 2;
        }
    }
}

/* ========================================================================
 * 大津法(Otsu)自适应阈值计算（基于压缩图像）
 * ======================================================================== */
static uint8 calc_otsu(void)
{
    float imgsize = (float)((uint32)COMP_H * COMP_W);

    for (uint16 i = 0; i < 256; i++) s_hist[i] = 0;
    for (uint8 i = 0; i < COMP_H; i++)
        for (uint8 j = 0; j < COMP_W; j++)
            s_hist[compressed_gray[i][j]]++;

    float Mg = 0.0f;
    for (uint16 i = 0; i < 256; i++)
        Mg += (float)i * s_hist[i] / imgsize;

    float max_sigma = 0.0f;
    uint8 best = 128u;
    float pk = 0.0f;
    float mk = 0.0f;

    for (uint16 i = 0; i < 255; i++)
    {
        float p = s_hist[i] / imgsize;
        pk += p;
        mk += (float)i * p;

        if (pk < 1e-6f || pk > 1.0f - 1e-6f) continue;
        float sigma = (mk - Mg * pk) * (mk - Mg * pk) / (pk * (1.0f - pk));
        if (sigma > max_sigma) { max_sigma = sigma; best = (uint8)i; }
    }
    return best;
}

/* ========================================================================
 * 图像二值化（基于压缩图像）
 * ======================================================================== */
static void binarize_image(void)
{
    for (uint16 i = 0; i < COMP_H; i++)
        for (uint16 j = 0; j < COMP_W; j++)
            bin_image[i][j] = (compressed_gray[i][j] > g_tf.threshold) ? 255u : 0u;
}

/* ========================================================================
 * 二值化图像去噪（3×3邻域滤波）
 * ======================================================================== */
static void denoise_binary_image(void)
{
    static uint8 prev_row[COMP_W];
    static uint8 curr_row[COMP_W];

    for (uint8 j = 0; j < COMP_W; j++)
        prev_row[j] = bin_image[0][j];

    for (uint8 i = 1; i < COMP_H - 1; i++)
    {
        for (uint8 j = 0; j < COMP_W; j++)
            curr_row[j] = bin_image[i][j];

        for (uint8 j = 1; j < COMP_W - 1; j++)
        {
            uint8 count = 0;
            if (prev_row[j-1] == 255u) count++;
            if (prev_row[j  ] == 255u) count++;
            if (prev_row[j+1] == 255u) count++;
            if (curr_row[j-1] == 255u) count++;
            if (curr_row[j+1] == 255u) count++;
            if (bin_image[i+1][j-1] == 255u) count++;
            if (bin_image[i+1][j  ] == 255u) count++;
            if (bin_image[i+1][j+1] == 255u) count++;

            if (curr_row[j] == 255u && count < 2)
                bin_image[i][j] = 0u;
            else if (curr_row[j] == 0u && count > 2)
                bin_image[i][j] = 255u;
        }

        for (uint8 j = 0; j < COMP_W; j++)
            prev_row[j] = curr_row[j];
    }
}

/* ========================================================================
 * 二值化图像上的边缘检测
 * ======================================================================== */
static inline uint8 is_white(uint8 row, int16 col)
{
    if (col < 0 || col >= TF_IMG_W) return 0u;
    return (bin_image[row][col] == 255u) ? 1u : 0u;
}

static inline uint8 is_left_edge(uint8 row, int16 col)
{
    if (col < 1 || col >= TF_IMG_W - 1) return 0u;
    return (!is_white(row, col - 1) && is_white(row, col) && is_white(row, col + 1)) ? 1u : 0u;
}

static inline uint8 is_right_edge(uint8 row, int16 col)
{
    if (col < 1 || col >= TF_IMG_W - 1) return 0u;
    return (is_white(row, col - 1) && is_white(row, col) && !is_white(row, col + 1)) ? 1u : 0u;
}

/* ========================================================================
 * 边界扫描函数
 * ======================================================================== */
static int16 scan_left_edge_right(uint8 row, int16 start, int16 end)
{
    if (start < 1) start = 1;
    if (end >= TF_IMG_W - 1) end = TF_IMG_W - 2;
    if (start > end) return TF_INVALID;
    for (int16 c = start; c <= end; c++)
        if (is_left_edge(row, c)) return c;
    return TF_INVALID;
}

static int16 scan_left_edge_left(uint8 row, int16 start, int16 end)
{
    if (start >= TF_IMG_W - 1) start = TF_IMG_W - 2;
    if (end < 1) end = 1;
    if (start < end) return TF_INVALID;
    for (int16 c = start; c >= end; c--)
        if (is_left_edge(row, c)) return c;
    return TF_INVALID;
}

static int16 scan_right_edge_left(uint8 row, int16 start, int16 end)
{
    if (start >= TF_IMG_W - 1) start = TF_IMG_W - 2;
    if (end < 1) end = 1;
    if (start < end) return TF_INVALID;
    for (int16 c = start; c >= end; c--)
        if (is_right_edge(row, c)) return c;
    return TF_INVALID;
}

static int16 scan_right_edge_right(uint8 row, int16 start, int16 end)
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
 * 底部基点检测
 * ======================================================================== */
static uint8 find_jidian1(void)
{
    const uint8 row = TF_JIDIAN_ROW;
    const int16 mid = TF_IMG_CENTER;
    const int16 qtr = TF_IMG_W / 4;
    const int16 tqtr = TF_IMG_W * 3 / 4;
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
 * 逐行搜索边界
 * ======================================================================== */
static uint8 search_row_edges(uint8 row, int16 prev_lb, int16 prev_rb,
                              int16 *out_lb, int16 *out_rb)
{
    int16 lb = TF_INVALID, rb = TF_INVALID;
    const int16 R = TF_LOCAL_RANGE;
    const int16 MID = TF_IMG_CENTER;

    lb = scan_left_edge_right(row, prev_lb - R, prev_lb + R);
    if (lb == TF_INVALID) lb = scan_left_edge_left(row, prev_lb + R, prev_lb - R);
    if (lb == TF_INVALID) lb = scan_left_edge_left(row, MID, 1);

    rb = scan_right_edge_left(row, prev_rb + R, prev_rb - R);
    if (rb == TF_INVALID) rb = scan_right_edge_right(row, prev_rb - R, prev_rb + R);
    if (rb == TF_INVALID) rb = scan_right_edge_right(row, MID, TF_IMG_W - 2);

    *out_lb = lb;
    *out_rb = rb;
    return valid_pair(lb, rb);
}

/* ========================================================================
 * 初始化
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
    g_tf.threshold = 128u;
    s_otsu_cnt = (uint8)TF_OTSU_INTERVAL;

    for (i = 0; i < COMP_H; i++)
        for (j = 0; j < COMP_W; j++)
        {
            bin_image[i][j] = 0u;
            compressed_gray[i][j] = 0u;
        }
}

/* ========================================================================
 * 加权中心线计算（辅助函数）
 * ======================================================================== */
static int16 calc_weighted_center(uint8 start_row, uint8 end_row, uint8 top_heavy)
{
    int32 sum = 0, wtotal = 0;
    for (int16 row = (int16)start_row; row >= (int16)end_row; row -= TF_ROW_STEP)
    {
        if (g_tf.row_valid[row])
        {
            int32 w = top_heavy
                ? (int32)((int16)start_row - row + 1)
                : (int32)(row - (int16)end_row + 1);
            sum += (int32)g_tf.center_line[row] * w;
            wtotal += w;
        }
    }
    return (wtotal > 0) ? (int16)(sum / wtotal) : (int16)TF_IMG_CENTER;
}

/* ========================================================================
 * 每帧更新：压缩 → Otsu → 二值化 → 去噪 → 循迹 → 前瞻
 * ======================================================================== */
void track_fusion_update(void)
{
    /* 0. 清空本帧数据 */
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

    /* 1. 压缩图像 188×120 → 94×60 */
    compress_image();

    /* 2. Otsu自适应阈值 */
    s_otsu_cnt++;
    if (s_otsu_cnt >= TF_OTSU_INTERVAL)
    {
        s_otsu_cnt = 0u;
        int16 raw = (int16)calc_otsu() + threshold_bias;
        if (raw < 20) raw = 20;
        if (raw > 240) raw = 240;
        g_tf.threshold = (uint8)raw;
    }

    /* 3. 图像二值化 */
    binarize_image();

    /* 4. 去噪（3×3邻域滤波） */
    denoise_binary_image();

    /* 5. 寻找基线 */
    if (!find_jidian1())
    {
        g_tf.line_lost = 1u;
        return;
    }

    const uint8 jrow = TF_JIDIAN_ROW;
    int16 prev_lb = (int16)g_tf.left_jidian;
    int16 prev_rb = (int16)g_tf.right_jidian;

    g_tf.left_edge[jrow] = prev_lb;
    g_tf.right_edge[jrow] = prev_rb;
    g_tf.center_line[jrow] = (prev_lb + prev_rb) / 2;
    g_tf.row_valid[jrow] = 1u;
    g_tf.valid_row_count = 1u;

    /* 6. 向上搜索边缘 */
    uint8 miss_streak = 0u;
    int16 end_row = (int16)TF_SEARCH_END_ROW;

    for (int16 row = (int16)jrow - 1; row >= end_row; row -= TF_ROW_STEP)
    {
        int16 lb, rb;
        if (search_row_edges((uint8)row, prev_lb, prev_rb, &lb, &rb))
        {
            g_tf.left_edge[row] = lb;
            g_tf.right_edge[row] = rb;
            g_tf.center_line[row] = (lb + rb) / 2;
            g_tf.row_valid[row] = 1u;
            g_tf.valid_row_count++;
            prev_lb = lb;
            prev_rb = rb;
            miss_streak = 0u;
        }
        else
        {
            g_tf.left_edge[row] = lb;
            g_tf.right_edge[row] = rb;
            g_tf.row_valid[row] = 0u;
            if (++miss_streak >= TF_MAX_MISS_ROWS) break;
        }
    }

    /* 7. 加权中心线偏差 */
    if (g_tf.valid_row_count == 0u)
    {
        g_tf.line_lost = 1u;
        return;
    }

    int16 avg_center = calc_weighted_center(jrow, TF_SEARCH_END_ROW, 0);
    g_tf.error = -(avg_center - (int16)TF_IMG_CENTER);
    g_tf.line_lost = 0u;

    /* 8. 前瞻行偏差（弯道趋势预判） */
    {
        int16 la_center = calc_weighted_center(TF_LOOKAHEAD_START_ROW, TF_LOOKAHEAD_END_ROW, 1);
        g_tf.lookahead_error = -(la_center - (int16)TF_IMG_CENTER);
        g_tf.error_trend = g_tf.lookahead_error - g_tf.error;
    }

    /* 9. 缩放误差回原坐标系（×2），保持PID参数不变 */
    g_tf.error = g_tf.error * 2;
    g_tf.lookahead_error = g_tf.lookahead_error * 2;
    g_tf.error_trend = g_tf.error_trend * 2;
}


/* ========================================================================
 * 拐点法路口检测
 * ======================================================================== */
uint8 g_ra_flag = 0u;
IntersectionResult_t g_inter_result;

static uint8 s_inter_lock_cnt = 0u;
static uint8 s_inter_cooldown_cnt = 0u;
static uint8 s_inter_confirm_cnt = 0u;
static uint8 s_inter_candidate = 0u;

uint8 g_ip_max_row = 0u;
uint8 g_debug_detected = 0u;

/* ================================================================
 * 拐点前边缘稳定性检查：跳变前window行内边缘方差小
 * ================================================================ */
static uint8 check_edge_stable_before(int16 *edge_arr, int16 row, uint8 window)
{
    int32 sum = 0;
    uint8 valid_cnt = 0;

    for (int16 i = 1; i <= window; i++)
    {
        int16 idx = row + i;
        if (idx >= TF_IMG_H) continue;
        if (edge_arr[idx] == TF_INVALID) return 0u;
        sum += edge_arr[idx];
        valid_cnt++;
    }
    if (valid_cnt < window) return 0u;

    int16 mean = (int16)(sum / valid_cnt);
    int32 variance = 0;
    for (int16 i = 1; i <= window; i++)
    {
        int16 idx = row + i;
        if (idx >= TF_IMG_H) continue;
        int16 diff = edge_arr[idx] - mean;
        variance += (int32)diff * diff;
    }

    return (variance <= IP_STABLE_VAR_MAX) ? 1u : 0u;
}

/* ================================================================
 * 拐点后方向持续性检查：跳变后window行持续同方向偏移
 * ================================================================ */
static uint8 check_edge_continue_after(int16 *edge_arr, int16 row, uint8 window, int16 *out_score)
{
    if (edge_arr[row] == TF_INVALID || edge_arr[row + 1] == TF_INVALID)
        return 0u;

    int16 first_jump = edge_arr[row] - edge_arr[row + 1];
    if (first_jump == 0) return 0u;

    uint8 direction = (first_jump > 0) ? 1u : 0u;
    int32 total_jump = 0;
    uint8 valid_cnt = 0;

    for (int16 i = 0; i < window; i++)
    {
        int16 idx = row - i;
        if (idx < 0 || idx + 1 >= TF_IMG_H) continue;
        if (edge_arr[idx] == TF_INVALID || edge_arr[idx + 1] == TF_INVALID)
            return 0u;

        int16 jump = edge_arr[idx] - edge_arr[idx + 1];
        if (jump == 0) continue;
        if ((jump > 0 ? 1u : 0u) != direction)
            return 0u;

        total_jump += (jump >= 0) ? jump : -jump;
        valid_cnt++;
    }

    if (valid_cnt < IP_MIN_CONTINUE) return 0u;

    *out_score = (int16)(total_jump / valid_cnt);
    return 1u;
}

/* ================================================================
 * 单边缘拐点扫描（增强版）：多行验证 + 综合评分
 * ================================================================ */
static uint8 scan_one_side(int16 *edge_arr, int16 start_row, int16 end_row, InflectionPoint_t *ip)
{
    int16 best_score = 0;
    int16 best_row = -1;

    for (int16 row = start_row; row >= end_row + IP_CHECK_WINDOW; row--)
    {
        if (edge_arr[row] == TF_INVALID || edge_arr[row + 1] == TF_INVALID)
            continue;

        int16 jump = edge_arr[row] - edge_arr[row + 1];
        int16 abs_jump = (jump >= 0) ? jump : -jump;
        if (abs_jump <= INTER_JUMP_THRESH)
            continue;

        if (!check_edge_stable_before(edge_arr, row, IP_STABLE_WINDOW))
            continue;

        int16 continue_score = 0;
        if (!check_edge_continue_after(edge_arr, row, IP_CHECK_WINDOW, &continue_score))
            continue;

        int16 score = abs_jump + continue_score / 2;
        if (score > best_score)
        {
            best_score = score;
            best_row = row;
        }
    }

    if (best_row >= 0)
    {
        ip->valid = 1u;
        ip->row = best_row + 1;
        ip->col = edge_arr[best_row + 1];
        return 1u;
    }
    return 0u;
}

/* 拐点检测：相邻行边线跳变 > INTER_JUMP_THRESH */
static void find_inflection_points(InflectionPoint_t *left_ip, InflectionPoint_t *right_ip)
{
    left_ip->valid = 0u; left_ip->row = 0; left_ip->col = 0;
    right_ip->valid = 0u; right_ip->row = 0; right_ip->col = 0;

    scan_one_side(g_tf.left_edge,  (int16)TF_JIDIAN_ROW - 1, (int16)TF_SEARCH_END_ROW, left_ip);
    scan_one_side(g_tf.right_edge, (int16)TF_JIDIAN_ROW - 1, (int16)TF_SEARCH_END_ROW, right_ip);
}

/* 拐点法路口检测主函数 */
void detect_intersection(void)
{
    g_debug_detected = 0u;

    if (s_inter_cooldown_cnt > 0u)
    {
        s_inter_cooldown_cnt++;
        if (s_inter_cooldown_cnt >= INTER_COOLDOWN_FRAMES) s_inter_cooldown_cnt = 0u;
        return;
    }

    InflectionPoint_t left_ip, right_ip;
    find_inflection_points(&left_ip, &right_ip);

    g_inter_result.left_ip = left_ip;
    g_inter_result.right_ip = right_ip;

    {
        uint8 lr = left_ip.valid ? (uint8)left_ip.row : 0u;
        uint8 rr = right_ip.valid ? (uint8)right_ip.row : 0u;
        g_ip_max_row = (lr > rr) ? lr : rr;
    }

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

    if (!left_ip.valid && !right_ip.valid) return;

    /* 拐点太远，跳过画框，只更新 g_ip_max_row */
    if (g_ip_max_row < INTER_BOX_START_ROW) return;

    /* 构建框 */
    uint8 detected = 0u;

    int16 cx, cy;
    if (left_ip.valid && right_ip.valid)
    { cx = (left_ip.col + right_ip.col) / 2; cy = (left_ip.row + right_ip.row) / 2; }
    else if (left_ip.valid)
    { cx = left_ip.col; cy = left_ip.row; }
    else
    { cx = right_ip.col; cy = right_ip.row; }

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

    /* 检查框的上边、左边、右边是否有连续白点（有路）
     * 要求连续白点数 >= INTER_BOX_MIN_STREAK，防止单点噪点误判 */
    uint8 top_has_road = 0u;
    uint8 left_has_road = 0u;
    uint8 right_has_road = 0u;

    {
        uint8 streak = 0u;
        for (int16 c = b_left; c <= b_right; c++)
        {
            if (bin_image[b_top][c] == 255u)
            {
                streak++;
                if (streak >= INTER_BOX_MIN_STREAK) { top_has_road = 1u; break; }
            }
            else
            {
                streak = 0u;
            }
        }
    }
    {
        uint8 streak = 0u;
        for (int16 r = b_top; r <= b_bottom; r++)
        {
            if (bin_image[r][b_left] == 255u)
            {
                streak++;
                if (streak >= INTER_BOX_MIN_STREAK) { left_has_road = 1u; break; }
            }
            else
            {
                streak = 0u;
            }
        }
    }
    {
        uint8 streak = 0u;
        for (int16 r = b_top; r <= b_bottom; r++)
        {
            if (bin_image[r][b_right] == 255u)
            {
                streak++;
                if (streak >= INTER_BOX_MIN_STREAK) { right_has_road = 1u; break; }
            }
            else
            {
                streak = 0u;
            }
        }
    }

    /* 方向判定
     * 1=只有右  2=只有左  3=左右  4=十字(上左右)
     * 5=上+右   6=上+左右 */
    if (top_has_road && left_has_road && right_has_road)
        detected = 6u;                                      // 上+左+右
    else if (top_has_road && right_has_road)
        detected = 5u;                                      // 上+右
    else if (left_has_road && right_has_road)
        detected = 3u;                                      // 左+右
    else if (right_has_road)
        detected = 1u;                                      // 只有右
    else if (left_has_road)
        detected = 2u;                                      // 只有左
    else if (top_has_road)
        detected = 4u;                                      // 只有上

    g_inter_result.detected_type = detected;
    g_debug_detected = detected;

    /* 连续确认 */
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

    /* 缩放拐点行号回原坐标系（×2），保持RA菜单参数不变 */
    g_ip_max_row = g_ip_max_row * 2;
}
