#include "Track_funsion.h"
/* ================================================================
 *  赛道约定：蓝色背景（亮）+ 深色赛道线（暗）
 *    · 赛道像素（灰度 < threshold）→ bin_image = 255（显示为白）
 *    · 背景像素（灰度 >= threshold）→ bin_image = 0 （显示为黑）
 *
 *  边界定义：
 *    · 左边界：从左向右第一个"黑→白"跳变点
 *    · 右边界：从右向左第一个"白→黑"跳变点
 * ================================================================ */

/* ==================== 全局变量 ==================== */

TrackFusion_t g_tf;

/* 二值化图像，和你原来 image[][] 完全一样的用法
 * 255=赛道线  0=背景 */
uint8 bin_image[TF_IMG_H][TF_IMG_W];

/* 大津法内部变量（static放全局区，不占栈）*/
static uint8 s_otsu_cnt = 0;
static uint16 s_hist[256];
static float s_P[256];
static float s_PK[256];
static float s_Mk[256];

/* ================================================================
 *  一、大津法求阈值
 * ================================================================ */
static uint8 calc_otsu(void)
{
    float imgsize = (float)((uint32)TF_IMG_H * TF_IMG_W);

    for (uint16 i = 0; i < 256; i++)
        s_hist[i] = 0;

    for (uint8 i = 0; i < TF_IMG_H; i++)
        for (uint8 j = 0; j < TF_IMG_W; j++)
            s_hist[mt9v03x_image[i][j]]++;

    for (uint16 i = 0; i < 256; i++)
        s_P[i] = s_hist[i] / imgsize;

    s_PK[0] = s_P[0];
    s_Mk[0] = 0.0f;
    for (uint16 i = 1; i < 256; i++)
    {
        s_PK[i] = s_PK[i - 1] + s_P[i];
        s_Mk[i] = s_Mk[i - 1] + (float)i * s_P[i];
    }

    float max_sigma = 0.0f;
    uint8 best = 128u;
    float Mg = s_Mk[255];

    for (uint16 i = 1; i < 255; i++)
    {
        float pk = s_PK[i];
        if (pk < 1e-6f || pk > 1.0f - 1e-6f)
            continue;
        float mk = s_Mk[i];
        float sigma = (mk - Mg * pk) * (mk - Mg * pk) / (pk * (1.0f - pk));
        if (sigma > max_sigma)
        {
            max_sigma = sigma;
            best = (uint8)i;
        }
    }
    return best;
}

/* ================================================================
 *  二、全图二值化（和你原来的 image_binaryzation 一样）
 *  深色像素（赛道线）< threshold → 255
 *  亮色像素（背景）  >= threshold → 0
 * ================================================================ */
static void binarize_image(void)
{
    for (uint16 i = 0; i < TF_IMG_H; i++)
        for (uint16 j = 0; j < TF_IMG_W; j++)
            bin_image[i][j] = (mt9v03x_image[i][j] < g_tf.threshold) ? 255u : 0u;
}

/* ================================================================
 *  三、像素判断（直接查 bin_image，不再实时比较灰度）
 * ================================================================ */
static inline uint8 is_white(uint8 row, int16 col)
{
    return (bin_image[row][(uint8)col] == 255u) ? 1u : 0u;
}

/* ================================================================
 *  四、跳变点检测（三点模板）
 *  左边界：col-1=黑, col=白, col+1=白
 *  右边界：col-1=白, col=白, col+1=黑
 * ================================================================ */
static inline uint8 is_left_edge(uint8 row, int16 col)
{
    if (col < 1 || col >= TF_IMG_W - 1)
        return 0u;
    return (!is_white(row, col - 1) && is_white(row, col) && is_white(row, col + 1)) ? 1u : 0u;
}

static inline uint8 is_right_edge(uint8 row, int16 col)
{
    if (col < 1 || col >= TF_IMG_W - 1)
        return 0u;
    return (is_white(row, col - 1) && is_white(row, col) && !is_white(row, col + 1)) ? 1u : 0u;
}

/* ================================================================
 *  五、四方向扫描函数（带边界保护）
 * ================================================================ */

/* 向右搜索左边界 */
static int16 scan_left_edge_right(uint8 row, int16 start, int16 end)
{
    if (start < 1)
        start = 1;
    if (end >= TF_IMG_W - 1)
        end = TF_IMG_W - 2;
    if (start > end)
        return TF_INVALID;
    for (int16 c = start; c <= end; c++)
        if (is_left_edge(row, c))
            return c;
    return TF_INVALID;
}

/* 向左搜索左边界 */
static int16 scan_left_edge_left(uint8 row, int16 start, int16 end)
{
    if (start >= TF_IMG_W - 1)
        start = TF_IMG_W - 2;
    if (end < 1)
        end = 1;
    if (start < end)
        return TF_INVALID;
    for (int16 c = start; c >= end; c--)
        if (is_left_edge(row, c))
            return c;
    return TF_INVALID;
}

/* 向左搜索右边界 */
static int16 scan_right_edge_left(uint8 row, int16 start, int16 end)
{
    if (start >= TF_IMG_W - 1)
        start = TF_IMG_W - 2;
    if (end < 1)
        end = 1;
    if (start < end)
        return TF_INVALID;
    for (int16 c = start; c >= end; c--)
        if (is_right_edge(row, c))
            return c;
    return TF_INVALID;
}

/* 向右搜索右边界 */
static int16 scan_right_edge_right(uint8 row, int16 start, int16 end)
{
    if (start < 1)
        start = 1;
    if (end >= TF_IMG_W - 1)
        end = TF_IMG_W - 2;
    if (start > end)
        return TF_INVALID;
    for (int16 c = start; c <= end; c++)
        if (is_right_edge(row, c))
            return c;
    return TF_INVALID;
}

/* ================================================================
 *  六、有效边界对检验
 * ================================================================ */
static inline uint8 valid_pair(int16 lb, int16 rb)
{
    if (lb == TF_INVALID || rb == TF_INVALID)
        return 0u;
    int16 w = rb - lb;
    return (w >= TF_MIN_TRACK_WIDTH && w <= TF_MAX_TRACK_WIDTH) ? 1u : 0u;
}

/* ================================================================
 *  七、基点搜索（四级策略）
 * ================================================================ */
static uint8 find_jidian1(void)
{
    const uint8 row = TF_JIDIAN_ROW;
    const int16 mid = TF_IMG_CENTER;
    const int16 qtr = TF_IMG_W / 4;
    const int16 tqtr = TF_IMG_W * 3 / 4;
    int16 lb = TF_INVALID, rb = TF_INVALID;

    /* 策略1：中点在赛道内 */
    if (is_white(row, mid - 1) && is_white(row, mid) && is_white(row, mid + 1))
    {
        lb = scan_left_edge_left(row, mid, 1);
        rb = scan_right_edge_right(row, mid, TF_IMG_W - 2);
    }
    /* 策略2：1/4点在赛道内（赛道偏左）*/
    else if (is_white(row, qtr - 1) && is_white(row, qtr) && is_white(row, qtr + 1))
    {
        lb = scan_left_edge_left(row, qtr, 1);
        rb = scan_right_edge_right(row, qtr, TF_IMG_W - 2);
    }
    /* 策略3：3/4点在赛道内（赛道偏右）*/
    else if (is_white(row, tqtr - 1) && is_white(row, tqtr) && is_white(row, tqtr + 1))
    {
        lb = scan_left_edge_left(row, tqtr, 1);
        rb = scan_right_edge_right(row, tqtr, TF_IMG_W - 2);
    }
    /* 策略4：全行兜底扫描 */
    else
    {
        lb = scan_left_edge_right(row, 1, TF_IMG_W - 2);
        if (lb != TF_INVALID)
            rb = scan_right_edge_left(row, TF_IMG_W - 2, lb + TF_MIN_TRACK_WIDTH);
    }

    if (!valid_pair(lb, rb))
        return 0u;

    g_tf.left_jidian = (uint8)lb;
    g_tf.right_jidian = (uint8)rb;
    return 1u;
}

/* ================================================================
 *  八、单行边线搜索（三级回退）
 * ================================================================ */
static uint8 search_row_edges(uint8 row, int16 prev_lb, int16 prev_rb,
                              int16 *out_lb, int16 *out_rb)
{
    int16 lb = TF_INVALID, rb = TF_INVALID;
    const int16 R = TF_LOCAL_RANGE;
    const int16 MID = TF_IMG_CENTER;

    /* 左边界搜索 */
    lb = scan_left_edge_right(row, prev_lb - R, prev_lb + R); // L1
    if (lb == TF_INVALID)
        lb = scan_left_edge_left(row, prev_lb + R, prev_lb - R); // L2
    if (lb == TF_INVALID)
        lb = scan_left_edge_left(row, MID, 1); // L3

    /* 右边界搜索 */
    rb = scan_right_edge_left(row, prev_rb + R, prev_rb - R); // L1
    if (rb == TF_INVALID)
        rb = scan_right_edge_right(row, prev_rb - R, prev_rb + R); // L2
    if (rb == TF_INVALID)
        rb = scan_right_edge_right(row, MID, TF_IMG_W - 2); // L3

    *out_lb = lb;
    *out_rb = rb;
    return valid_pair(lb, rb);
}

/* ================================================================
 *  九、对外接口
 * ================================================================ */

void track_fusion_init(void)
{
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
    g_tf.left_jidian = (uint8)TF_IMG_CENTER;
    g_tf.right_jidian = (uint8)TF_IMG_CENTER;
    g_tf.threshold = 128u;
    s_otsu_cnt = (uint8)TF_OTSU_INTERVAL;

    /* 清空二值化图像 */
    for (uint16 i = 0; i < TF_IMG_H; i++)
        for (uint16 j = 0; j < TF_IMG_W; j++)
            bin_image[i][j] = 0u;
}

void track_fusion_update(void)
{
    /* 0. 清空上帧结果 */
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

    /* 1. 周期性大津阈值更新 */
    s_otsu_cnt++;
    if (s_otsu_cnt >= TF_OTSU_INTERVAL)
    {
        s_otsu_cnt = 0u;
        int16 raw = (int16)calc_otsu() + TF_THRESHOLD_BIAS;
        if (raw < 20)
            raw = 20;
        if (raw > 240)
            raw = 240;
        g_tf.threshold = (uint8)raw;
    }

    /* 2. 全图二值化（阈值更新后立刻执行）
     *    遍历所有像素，深色=赛道线=255，亮色=背景=0
     *    和你原来的 image_binaryzation 完全一样的逻辑 */
    binarize_image();

    /* 3. 基点搜索 */
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

    /* 4. 逐行向上边界跟随 */
    uint8 miss_streak = 0u;
    int16 end_row = (int16)TF_SEARCH_END_ROW;

    for (int16 row = (int16)jrow - 1; row >= end_row; row--)
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
            g_tf.row_valid[row] = 0u;
            if (++miss_streak >= TF_MAX_MISS_ROWS)
                break;
        }
    }

    /* 5. 加权中线误差计算 */
    if (g_tf.valid_row_count == 0u)
    {
        g_tf.line_lost = 1u;
        return;
    }

    int32 weighted_sum = 0, weight_total = 0;
    for (int16 row = (int16)jrow; row >= end_row; row--)
    {
        if (g_tf.row_valid[row])
        {
            int32 w = (int32)(row - end_row + 1);
            weighted_sum += (int32)g_tf.center_line[row] * w;
            weight_total += w;
        }
    }

    int16 avg_center = (weight_total > 0)
                           ? (int16)(weighted_sum / weight_total)
                           : (int16)TF_IMG_CENTER;

    g_tf.error = avg_center - (int16)TF_IMG_CENTER;
    g_tf.line_lost = 0u;
}


// -------- 直角检测相关 --------
#define RIGHT_ANGLE_THRESHOLD 35
#define RIGHT_ANGLE_FRAMES 2

static uint8 s_right_angle_cnt = 0u;
uint8 g_right_angle_flag = 0u; // 全局变量，pid那边要用
int8 g_right_angle_dir = 0;    // 全局变量，pid那边要用

void right_angle_detect(void)
{
    if (g_tf.line_lost)
    {
        s_right_angle_cnt = 0u;
        g_right_angle_flag = 0u;
        return;
    }

    int16 abs_err = g_tf.error >= 0 ? g_tf.error : -g_tf.error;

    if (abs_err >= RIGHT_ANGLE_THRESHOLD)
    {
        s_right_angle_cnt++;
        if (s_right_angle_cnt >= RIGHT_ANGLE_FRAMES)
        {
            g_right_angle_flag = 1u;
            g_right_angle_dir = g_tf.error > 0 ? 1 : -1;
        }
    }
    else
    {
        s_right_angle_cnt = 0u;
        g_right_angle_flag = 0u;
    }
}